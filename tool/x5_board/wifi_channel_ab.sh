#!/usr/bin/env bash
# Measure every DEEP-RD AP back to back, in one short window, then restore the original
# connection. Run detached -- it drops the link on purpose, so ssh will not survive it.
#
# 为什么要 A/B 而不是看历史日志：ch1/ch6/ch11 的对照分别来自 11:0x / 13:5x / 14:1x，
# 时间不同、干扰不同，结论是被时间混淆的（2026-08-27 用户指出）。这个脚本把三个 AP
# 压在几分钟内轮测，时间就不再是变量。
#
# 用法（在板上）：
#   nohup bash /userdata/x5/wifi_channel_ab.sh > /userdata/x5/logs/wifi_ab.log 2>&1 &
# 结果：/userdata/x5/logs/wifi_ab.log ；结束后自动恢复原来的连接方式。
set -u
IFACE="${IFACE:-$(ls /sys/class/net | grep '^wl' | head -1)}"
SSID="${SSID:-DEEP-RD}"
PASS="${PASS:-07310731}"
PC="${PC:-192.168.19.51}"
DWELL="${DWELL:-45}"          # 每个 AP 测多少秒
WIFI_UP="${WIFI_UP:-/etc/init.d/looper/wifi-connect.sh}"
OUT="/userdata/x5/logs/wifi_ab_result.txt"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

restore() {
    log "恢复原来的连接方式（$WIFI_UP）"
    pkill -f "wpa_supplicant.*-i *${IFACE}" 2>/dev/null
    pkill -f "udhcpc.*-i *${IFACE}" 2>/dev/null
    sleep 1
    sh "$WIFI_UP" >/dev/null 2>&1
    sleep 8
    log "恢复完成：$(iw dev "$IFACE" link 2>/dev/null | head -3 | tr '\n' ' ')"
}
# 任何退出路径都要恢复，否则板子会留在一个钉死的、可能连不上的 AP 上。
trap restore EXIT INT TERM

log "接口=$IFACE  目标 PC=$PC  每个 AP 测 ${DWELL}s"
log "先记下当前关联：$(iw dev "$IFACE" link 2>/dev/null | head -3 | tr '\n' ' ')"

log "扫描 $SSID 的所有 AP"
SCAN=$(iw dev "$IFACE" scan 2>/dev/null)
CANDS=$(echo "$SCAN" | awk -v s="$SSID" '
  /^BSS /{b=$2; sub(/\(.*/,"",b)}
  /freq: /{f=$2}
  /signal: /{g=$2}
  /SSID: /{ if (index($0,s)) printf "%s %s %s\n", b, f, g }')
# 历史上见过的三个，扫不到时兜底
[ -z "$CANDS" ] && CANDS="a0:69:d9:5b:cc:13 2437 ?
a0:69:d9:5b:f0:13 2412 ?
a0:69:d9:5b:f0:a3 2462 ?"
log "候选："; echo "$CANDS" | sed 's/^/    /'

: > "$OUT"
printf '%-20s %-6s %-8s %-9s %-9s %-9s %s\n' BSSID freq rssi 丢包率 rtt均值 rtt抖动 备注 >> "$OUT"

while read -r B F G; do
    [ -z "${B:-}" ] && continue
    log "=== 切到 $B (freq=$F, 扫描时 rssi=$G)"
    pkill -f "wpa_supplicant.*-i *${IFACE}" 2>/dev/null
    pkill -f "udhcpc.*-i *${IFACE}" 2>/dev/null
    sleep 2
    CONF=/tmp/wpa_ab.conf
    { echo "ctrl_interface=/var/run/wpa_supplicant"
      wpa_passphrase "$SSID" "$PASS" | sed "/^network={/a\\\tbssid=$B\n\tfreq_list=$F\n\tscan_freq=$F"
    } > "$CONF"
    wpa_supplicant -B -i "$IFACE" -c "$CONF" >/dev/null 2>&1
    sleep 6
    udhcpc -i "$IFACE" -n -q -t 8 >/dev/null 2>&1
    IP=$(ip -4 addr show "$IFACE" 2>/dev/null | sed -n 's/.*inet \([0-9.]*\).*/\1/p' | head -1)
    LINK=$(iw dev "$IFACE" link 2>/dev/null)
    RSSI=$(echo "$LINK" | sed -n 's/.*signal: \(-*[0-9]*\).*/\1/p' | head -1)
    RATE=$(echo "$LINK" | sed -n 's/.*tx bitrate: \([0-9.]*\).*/\1/p' | head -1)
    if [ -z "${IP:-}" ]; then
        printf '%-20s %-6s %-8s %-9s %-9s %-9s %s\n' "$B" "$F" "${RSSI:-?}" - - - "拿不到 IP" >> "$OUT"
        log "  拿不到 IP，跳过"
        continue
    fi
    log "  已关联 ip=$IP rssi=${RSSI:-?} rate=${RATE:-?}Mbit/s，开始测 ${DWELL}s"
    N=$((DWELL * 5))
    P=$(ping -i 0.2 -c "$N" -W 1 "$PC" 2>&1 | tail -3)
    LOSS=$(echo "$P" | sed -n 's/.*, \([0-9.]*\)% packet loss.*/\1/p')
    AVG=$(echo "$P"  | sed -n 's|.*/\([0-9.]*\)/[0-9.]*/[0-9.]* ms.*|\1|p')
    MDEV=$(echo "$P" | sed -n 's|.*/[0-9.]*/[0-9.]*/\([0-9.]*\) ms.*|\1|p')
    FA=$(cat /proc/net/wireless 2>/dev/null | tail -1 | awk '{print $3}')
    printf '%-20s %-6s %-8s %-9s %-9s %-9s %s\n' \
        "$B" "$F" "${RSSI:-?}" "${LOSS:-?}%" "${AVG:-?}ms" "${MDEV:-?}ms" \
        "rate=${RATE:-?}Mbit qual=${FA:-?}" >> "$OUT"
    log "  丢包 ${LOSS:-?}%  rtt均值 ${AVG:-?}ms  抖动 ${MDEV:-?}ms"
done <<< "$CANDS"

log "=== 结果 ==="
cat "$OUT"
