#!/usr/bin/env bash
# Read-only: collect everything that survives a dropout or a hard power cut.
# Run on the board. Nothing here writes, restarts, or moves anything.
#
# 只看能活过断电的东西：/userdata 上的文件和 pstore。rootfs 的 journal 每次开机清空
# （见 docs/x5/board_bringup.md），所以 journalctl 在这里没有价值。
set -u
LOGDIR="${LOGDIR:-/userdata/x5/logs}"
W="${1:-600}"          # 回看多少秒的健康行

echo "===== 1. 现在什么状态 ====="
date; uptime
echo "boot 次数（pstore 里的记录数，能跨硬复位）: $(ls /sys/fs/pstore 2>/dev/null | wc -l)"
ls -l /sys/fs/pstore 2>/dev/null | tail -5

echo; echo "===== 2. 内核有没有崩过 ====="
if [ -d /sys/fs/pstore ] && [ -n "$(ls -A /sys/fs/pstore 2>/dev/null)" ]; then
    for f in /sys/fs/pstore/*; do echo "--- $f"; head -c 1200 "$f"; echo; done
else
    echo "pstore 空 —— 上次不是 panic/oops（正常断电或掉网都不会留东西）"
fi

echo; echo "===== 3. 链路与温度时间线（board_health，写在 /userdata，活得过断电） ====="
BH="${LOGDIR}/board_health.log"
if [ -f "$BH" ]; then
    echo "文件 $(stat -c '%s 字节, 最后写入 %y' "$BH")"
    echo "--- 最后 40 行"
    tail -40 "$BH"
    echo "--- carrier / link 变化过的行（掉线的直接证据）"
    grep -a -E 'carrier=0|link=(down|no)|usb=0' "$BH" | tail -20 || echo "没有 carrier=0 / link=down —— 接口一直在，那么问题在射频或对端"
else
    echo "⚠ 没有 $BH —— board-health.service 没在跑？"
    systemctl is-active board-health.service 2>/dev/null
fi

echo; echo "===== 4. 内核里的网卡/USB 事件 ====="
dmesg 2>/dev/null | grep -aiE 'wlan|8188|8189|rtl|usb .*(disconnect|reset|new)|oom|Out of memory' | tail -25 \
  || echo "dmesg 取不到（权限或已被覆盖）"

echo; echo "===== 5. 当前射频状态 ====="
for i in /sys/class/net/*/carrier; do n=$(basename "$(dirname "$i")"); echo "  $n carrier=$(cat "$i" 2>/dev/null)"; done
iw dev 2>/dev/null | grep -E 'Interface|ssid|channel' || echo "  没有 iw"
iw dev wlan0 link 2>/dev/null | head -12
cat /proc/net/wireless 2>/dev/null

echo; echo "===== 6. 上位机↔ESP32 的链路事件与电压（活在 /userdata 的节点日志里） ====="
LATEST=$(ls -1 "${LOGDIR}"/nodes/*diffcar_control* 2>/dev/null | tail -1)
if [ -n "${LATEST:-}" ]; then
    echo "--- $(basename "$LATEST")"
    grep -a -iE 'teleop|静默|silent|link|battery|volt|K 0|K 1' "$LATEST" | tail -15
    grep -a -oE 'V=[0-9.]+' "$LATEST" | tail -5
else
    echo "没有 diffcar_control 日志 —— nav 没开过，所以串口没人写"
    echo "⚠ 这也解释了 ESP32 的橙灯：橙 = UART1 静默 5 s = 上位机没在写串口"
    echo "  nav 没开的时候橙灯是**正常的**，不是故障。网络指示是**青灯**，不是橙灯。"
fi

echo; echo "===== 7. app 状态 ====="
systemctl is-active board-app.service 2>/dev/null
ps -o pid,etime,comm 2>/dev/null | grep -E 'uvicorn|python3' | head -6
(netstat -tln 2>/dev/null || ss -tln 2>/dev/null) | grep 8000 || echo "8000 没在监听"
