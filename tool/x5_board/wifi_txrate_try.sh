#!/bin/sh
# 带参数重载 USB 网卡驱动，试着抬高发送方向的吞吐（在板上跑，不是 PC 上跑）。
#
# 🔴 C 口现在是 host 模式，没有有线兜底 —— 网卡起不来板子就彻底失联。
# 跑之前必须先起 wifi_watchdog.sh（不通就自动重连，重连无效就重启）。
#
# 用法（一定要脱离 ssh 会话，否则链路一断脚本就被杀）：
#   setsid nohup sh wifi_txrate_try.sh > /tmp/try.log 2>&1 < /dev/null &
#   sh wifi_txrate_try.sh restore      # 退回原样（无参数加载）
set -e

KO=/root/8188gu.ko
CONNECT=/etc/init.d/looper/wifi-connect.sh

# system_init.sh 是裸 insmod，所以「原样」就是不带任何参数
[ "$1" = "restore" ] && PARAMS="" || PARAMS="rtw_wifi_spec=1 rtw_lowrate_two_xmit=0"

echo "卸载 8188gu…"
rmmod 8188gu 2>/dev/null || true
sleep 2
echo "加载: insmod $KO $PARAMS"
insmod "$KO" $PARAMS
sleep 2

# 必须自己重连：光 insmod 只会让接口出现，不会关联到 AP
echo "重连 WiFi…"
sh "$CONNECT" || true          # 该脚本最后一步用了板上不存在的 ip(1)，会非零退出但网已经通了

echo "完成。看发送速率：python3 /tmp/txrate_probe.py"
