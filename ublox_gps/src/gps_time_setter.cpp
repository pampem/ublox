/*
  Copyright (c) 2025 Masashi Izumita

  All rights reserved.
*/

#include <rclcpp/rclcpp.hpp>

#include <ublox_msgs/msg/nav_pvt.hpp>

#include <sys/time.h>  // clock_settime

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>

#include <string>

class GpsTimeSetter : public rclcpp::Node
{
public:
  GpsTimeSetter() : Node("gps_time_setter")
  {
    navpvt_topic_ = declare_parameter<std::string>("navpvt_topic", "ublox_gps_node/navpvt");
    min_fix_type_ = declare_parameter<int>("min_fix_type", 3);  // 0: no fix, 3: 3D fix
    tacc_threshold_ns_ = declare_parameter<int>("tacc_threshold_ns", 50000);  // 50us
    once_only_ = declare_parameter<bool>("once_only", true);

    sub_ = create_subscription<ublox_msgs::msg::NavPVT>(
      navpvt_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GpsTimeSetter::on_nav_pvt, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "gps_time_setter: waiting NavPVT on '%s' (min_fix_type=%d, tAcc<=%d ns)",
      navpvt_topic_.c_str(), min_fix_type_, tacc_threshold_ns_);
  }

private:
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

    // NavPVT -> timespec（UTC）
    // NavPVT の nano は -1e9..1e9 の可能性があるので正規化
    std::tm tm{};
    tm.tm_year = static_cast<int>(msg->year) - 1900;  // years since 1900
    tm.tm_mon = static_cast<int>(msg->month) - 1;     // 0-11
    tm.tm_mday = static_cast<int>(msg->day);
    tm.tm_hour = static_cast<int>(msg->hour);
    tm.tm_min = static_cast<int>(msg->min);
    tm.tm_sec = static_cast<int>(msg->sec);

    // timegm: glibc あり。UTC として mktime する
    time_t secs = timegm(&tm);
    if (secs == static_cast<time_t>(-1)) {
      RCLCPP_WARN(get_logger(), "timegm failed; not setting system clock");
      return;
    }

    int64_t nsec = msg->nano;  // could be negative
    while (nsec < 0) {
      nsec += 1000000000L;
      secs -= 1;
    }
    while (nsec >= 1000000000L) {
      nsec -= 1000000000L;
      secs += 1;
    }

    struct timespec ts;
    ts.tv_sec = secs;
    ts.tv_nsec = nsec;

    // 実際に OS 時刻を設定
    if (clock_settime(CLOCK_REALTIME, &ts) != 0) {
      int e = errno;
      RCLCPP_ERROR(get_logger(), "clock_settime failed: %s (need CAP_SYS_TIME?)", std::strerror(e));
      return;
    }

    did_set_ = true;
    RCLCPP_INFO(
      get_logger(),
      "System clock set to %04u-%02u-%02u %02u:%02u:%02u.%09ldZ (tAcc=%u ns, fix_type=%u)",
      msg->year, msg->month, msg->day, msg->hour, msg->min, msg->sec, nsec, msg->t_acc,
      msg->fix_type);

    // 一度だけ設定して終了（パラメータで切替可）
    if (once_only_) {
      // 少し待ってから shutdown（ログ flush 用）
      rclcpp::WallRate(std::chrono::milliseconds(5)).sleep();
      rclcpp::shutdown();
    }
  }

  // params
  std::string navpvt_topic_;
  int min_fix_type_;
  int tacc_threshold_ns_;
  bool once_only_;

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
