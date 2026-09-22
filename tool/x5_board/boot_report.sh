#!/bin/sh
# 开机后 N 秒把整个启动现场存成一个文件。为什么需要：切到手机热点后 PC 够不着板子，
# 事后才能取；而 journald 开机头约 55 秒的日志是丢的，事后 journalctl 根本看不到那段。
# 跑法：开机自启(board-bootreport.service)，或手动 `sh boot_report.sh now`。
set -u
DELAY=${BOOTREPORT_DELAY:-120}
[ "${1:-}" = "now" ] && DELAY=0
DIR=/userdata/x5/logs
OUT="$DIR/bootreport_$(date '+%Y%m%d_%H%M%S').txt"
mkdir -p "$DIR"
[ "$DELAY" -gt 0 ] && sleep "$DELAY"

s() { echo; echo "======== $* ========"; }
{
  echo "采集于 $(date '+%F %T')  板子 uptime=$(cut -d. -f1 /proc/uptime)s"
  s "USB 枚举时间线(单调秒，最可靠 —— 内核环形缓冲不受时钟跳变影响)"
  dmesg 2>/dev/null | grep -iE "usb [0-9]-1:|cdc_ncm|xhci" | tail -25
  s "网络时间线(单调秒。⚠️ 墙钟在开机中途会跳，只能信单调时钟)"
  journalctl -b --no-pager -o short-monotonic 2>/dev/null \
    | grep -iE "udhcpc|netheal|mdns|usbrole|board-app|DHCPv4" | tail -40
  s "当前网络"
  ifconfig 2>/dev/null | grep -A1 "^en\|^wl\|^usb"; echo "--"; route -n 2>/dev/null
  echo "-- 默认路由条数(应为 1) --"; route -n | grep -c "^0.0.0.0"
  s "netheal 自己的日志"; tail -30 "$DIR/../netheal.log" 2>/dev/null || tail -30 /userdata/x5/netheal.log 2>/dev/null
  s "对时(含相机时间戳自检 —— 相机比对时早起就会差一个跳变量，同步 slop 只有 0.06s)"
  tail -25 "$DIR/autotime.log" 2>/dev/null
  s "链路质量(板子->USB->ESP32->WiFi->AP 整条)"
  tail -15 "$DIR/link_quality.tsv" 2>/dev/null
  s "服务启动耗时"; systemd-analyze blame 2>/dev/null | head -10
  # 🔴 board-app 失败时唯一说得出原因的东西。它只写 stderr -> journald，而 journald 在这块板上
  # 活不过断电：2026-09-22 热点冷启动它连失败 4 次，事后零线索。所以趁 journal 还在赶紧抄一份。
  s "board-app 本次开机的全部输出(失败原因只在这里)"
  systemctl status board-app --no-pager -n 0 2>&1 | head -12
  journalctl -u board-app -b --no-pager -o short-monotonic 2>&1 | tail -60
  s "app_start.sh 自己的早退记录(跨重启保留)"; tail -25 /userdata/x5/logs/app_start.log 2>/dev/null
  s "启动失败过的单元"; systemctl list-units --state=failed --no-pager --no-legend 2>&1 | head -10
  s "ESP32(冷启动里程碑 / 稳定性 / 卡死现场 / 各项丢包计数)"
  cd /userdata/x5 && . ./env.sh >/dev/null 2>&1
  export ROS_LOCALHOST_ONLY=1
  timeout 40 python3 /userdata/x5/esp_status.py N 8 2>&1 | tail -45
  s "相机时间戳(>1s 就说明 planning 会零回调，导航和局部视图全空)"
  timeout 30 python3 /userdata/x5/check_cam_stamp.py 6 1.0 2>&1
  s "节点"; timeout 15 ros2 node list 2>/dev/null
} > "$OUT" 2>&1
echo "$OUT"
