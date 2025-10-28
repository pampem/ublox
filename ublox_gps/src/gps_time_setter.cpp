/*
  Copyright (c) 2025 Masashi Izumita
  All rights reserved.
*/

#include <rclcpp/rclcpp.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include <spawn.h>
#include <sys/wait.h>
extern char **environ;

class GpsTimeSetter : public rclcpp::Node
{
public:
  GpsTimeSetter() : Node("gps_time_setter")
  {
    navpvt_topic_       = declare_parameter<std::string>("navpvt_topic", "ublox_gps_node/navpvt");
    min_fix_type_       = declare_parameter<int>("min_fix_type", 3);           // 0: no fix, 3: 3D fix
    tacc_threshold_ns_  = declare_parameter<int>("tacc_threshold_ns", 50000);  // 50us
    once_only_          = declare_parameter<bool>("once_only", true);
    helper_path_        = declare_parameter<std::string>("helper_path", "/usr/local/sbin/settime_utc");

    sub_ = create_subscription<ublox_msgs::msg::NavPVT>(
      navpvt_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GpsTimeSetter::on_nav_pvt, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "gps_time_setter: waiting NavPVT on '%s' (min_fix_type=%d, tAcc<=%d ns, helper='%s')",
      navpvt_topic_.c_str(), min_fix_type_, tacc_threshold_ns_, helper_path_.c_str());
  }

private:
  // NavPVT -> ISO8601 UTC（Z終端）; nano は±の可能性があるので正規化
  static std::string make_iso8601_utc(const ublox_msgs::msg::NavPVT &m)
  {
    int64_t sec  = static_cast<int64_t>(m.sec);
    int64_t nsec = static_cast<int64_t>(m.nano);

    if (nsec >= 1000000000LL) { sec += nsec / 1000000000LL; nsec %= 1000000000LL; }
    else if (nsec < 0) {
      int64_t borrow = (-nsec + 999999999LL) / 1000000000LL;
      sec  -= borrow;
      nsec += borrow * 1000000000LL;
    }

    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(4) << (int)m.year  << '-'
        << std::setw(2) << (int)m.month << '-'
        << std::setw(2) << (int)m.day   << 'T'
        << std::setw(2) << (int)m.hour  << ':'
        << std::setw(2) << (int)m.min   << ':'
        << std::setw(2) << (int)sec;

    if (nsec > 0) {
      // 9桁で出力 → 末尾の0を削る
      std::ostringstream nss; nss << std::setw(9) << std::setfill('0') << nsec;
      std::string ns = nss.str();
      while (!ns.empty() && ns.back() == '0') ns.pop_back();
      oss << '.' << ns;
    }
    oss << 'Z';
    return oss.str();
  }

  // posix_spawn で /usr/local/sbin/settime_utc を直接呼ぶ
  bool call_settime_helper(const std::string &iso8601_utc)
  {
    const char *helper = helper_path_.c_str();
    char *argvv[3];
    argvv[0] = const_cast<char*>(helper);
    argvv[1] = const_cast<char*>(iso8601_utc.c_str());
    argvv[2] = nullptr;

    pid_t pid;
    int rc = posix_spawn(&pid, helper, nullptr, nullptr, argvv, environ);
    if (rc != 0) {
      RCLCPP_ERROR(get_logger(), "posix_spawn failed (%d): %s", rc, std::strerror(rc));
      return false;
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
      int e = errno;
      RCLCPP_ERROR(get_logger(), "waitpid failed: %s", std::strerror(e));
      return false;
    }

    if (WIFEXITED(status)) {
      int code = WEXITSTATUS(status);
      if (code == 0) return true;
      RCLCPP_ERROR(get_logger(), "helper exited with code %d", code);
    } else if (WIFSIGNALED(status)) {
      RCLCPP_ERROR(get_logger(), "helper killed by signal %d", WTERMSIG(status));
    } else {
      RCLCPP_ERROR(get_logger(), "helper ended abnormally");
    }
    return false;
  }

  void on_nav_pvt(const ublox_msgs::msg::NavPVT::SharedPtr msg)
  {
    if (did_set_ && once_only_) return;

    // 基本バリデーション
    if (msg->fix_type < min_fix_type_) {
      RCLCPP_DEBUG(get_logger(), "fix_type=%u < %d, skip", msg->fix_type, min_fix_type_);
      return;
    }
    if (msg->t_acc > static_cast<uint32_t>(tacc_threshold_ns_)) {
      RCLCPP_DEBUG(
        get_logger(), "tAcc=%u ns > threshold %d ns, skip", msg->t_acc, tacc_threshold_ns_);
      return;
    }
    if (msg->year < 2020) {
      RCLCPP_DEBUG(get_logger(), "year=%u looks invalid, skip", msg->year);
      return;
    }

    // ISO8601（UTC）へ変換
    const std::string iso = make_iso8601_utc(*msg);
    RCLCPP_INFO(
      get_logger(),
      "Setting system clock via helper to %s (tAcc=%u ns, fix_type=%u)",
      iso.c_str(), msg->t_acc, msg->fix_type);

    // 実際にヘルパーを呼ぶ
    if (!call_settime_helper(iso)) {
      RCLCPP_ERROR(get_logger(), "settime helper failed; system clock NOT changed");
      return;
    }

    did_set_ = true;
    RCLCPP_INFO(get_logger(), "System time set successfully. Exiting.");

    if (once_only_) {
      rclcpp::WallRate(std::chrono::milliseconds(5)).sleep();  // ログ flush
      rclcpp::shutdown();
    }
  }

  // params
  std::string navpvt_topic_;
  int         min_fix_type_;
  int         tacc_threshold_ns_;
  bool        once_only_;
  std::string helper_path_;

  // state
  bool did_set_{false};

  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsTimeSetter>());
  rclcpp::shutdown();
  return 0;
}
