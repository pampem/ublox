# How to use gps_time_setter

以下ファイルを作ってビルドし特権付与。

## /usr/local/sbin/settime_utc.c

```c
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

static int parse_iso8601_utc(const char *s, struct timespec *ts) {
  // 期待形式: YYYY-MM-DDTHH:MM:SS[.nnnnnnnnn]Z
  // 1) 'Z' 必須、2) '.' は任意、3) 9桁までのナノ秒
  size_t len = strlen(s);
  if (len < 20) return -1;
  if (s[len-1] != 'Z') return -1;

  // 小数部を探す
  const char *dot = strchr(s, '.');
  char buf[32]; // "YYYY-MM-DDTHH:MM:SS" までを切り出し
  if (dot) {
    size_t mainlen = (size_t)(dot - s);
    if (mainlen >= sizeof(buf)) return -1;
    memcpy(buf, s, mainlen);
    buf[mainlen] = '\0';
  } else {
    // 末尾の 'Z' を除いた全体が main
    size_t mainlen = len - 1;
    if (mainlen >= sizeof(buf)) return -1;
    memcpy(buf, s, mainlen);
    buf[mainlen] = '\0';
  }

  struct tm tm = {0};
  // UTC として解釈（timegm 使用）
  if (strptime(buf, "%Y-%m-%dT%H:%M:%S", &tm) == NULL) return -1;
  time_t sec = timegm(&tm);
  if (sec == (time_t)-1) return -1;

  long nsec = 0;
  if (dot) {
    const char *frac = dot + 1;
    size_t fraclen = (size_t)((s + len - 1) - frac); // 'Z' の直前まで
    if (fraclen == 0) return -1;
    if (fraclen > 9) fraclen = 9; // 9桁に丸め
    // 数字以外が混ざっていないか軽くチェック
    for (size_t i = 0; i < fraclen; ++i) {
      if (frac[i] < '0' || frac[i] > '9') return -1;
    }
    // 右側0詰めでナノ秒に
    char fracbuf[10] = {0};
    memcpy(fracbuf, frac, fraclen);
    for (size_t i = fraclen; i < 9; ++i) fracbuf[i] = '0';
    nsec = strtol(fracbuf, NULL, 10);
  }

  ts->tv_sec = sec;
  ts->tv_nsec = nsec;
  return 0;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s 2025-10-28T06:15:33.123456789Z\n", argv[0]);
    return 2;
  }
  struct timespec ts;
  if (parse_iso8601_utc(argv[1], &ts) != 0) {
    fprintf(stderr, "Invalid time format: %s\n", argv[1]);
    return 3;
  }
  if (clock_settime(CLOCK_REALTIME, &ts) != 0) {
    perror("clock_settime");
    return (errno == EPERM) ? 4 : 5;
  }
  return 0;
}
```

## Build & Privilege
```bash
# ビルド（動的リンクでOK／最小にしたいなら -static でも可）
gcc -O2 -Wall /usr/local/sbin/settime_utc.c -o /usr/local/sbin/settime_utc

# root 所有・実行可能に（任意）
sudo chown root:root /usr/local/sbin/settime_utc
sudo chmod 0755      /usr/local/sbin/settime_utc

# このバイナリだけに能力を付与（ROSノードには付けない！）
sudo setcap cap_sys_time+ep /usr/local/sbin/settime_utc
getcap /usr/local/sbin/settime_utc
# => /usr/local/sbin/settime_utc cap_sys_time=ep

# テスト（例）
/usr/local/sbin/settime_utc 2025-10-28T06:15:33Z
date -u
```

### Launch

```bash
ros2 launch ublox_gps gnss_with_time_setter.launch.py
```

### Confirmation

```bash
date
```
