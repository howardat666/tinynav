#!/bin/sh
# 板上自动对时。X5 没有 RTC，每次上电时钟都回到 2025-08-26，不修会静默毁掉
# PC 端节点的时间戳配对和所有日志（见 board_bringup.md §2）。
#
# 装成开机服务后就不用再从 PC 跑 sync_board_time.sh：
#   cp board_autotime.sh /userdata/x5/ && cp board-autotime.service /etc/systemd/system/
#   systemctl enable --now board-autotime
set -u

IFACE=wlx80ea07cb5d4a
GW=192.168.19.254
# 不用网关做时间源：实测它自己比公网 NTP 慢 822 秒
SERVERS="ntp.aliyun.com cn.pool.ntp.org ntp1.aliyun.com"
FLAG=/etc/init.d/looper/setting/is_time_sync
LOG=/userdata/x5/logs/autotime.log
WAIT_NET=60

mkdir -p "$(dirname "$LOG")"
log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

log "=== autotime 启动 ==="

# 1. 等网卡拿到 IP（开机时 wifi-connect.sh 可能还没跑完）
i=0
while [ $i -lt $WAIT_NET ]; do
    ifconfig "$IFACE" 2>/dev/null | grep -q "inet " && break
    i=$((i + 1)); sleep 1
done
if ! ifconfig "$IFACE" 2>/dev/null | grep -q "inet "; then
    log "没网($IFACE 无 IP)，放弃对时"; exit 0
fi

# 2. 默认路由：DHCP 装不上，因为死掉的 usb0 那条一直占着 metric 0
if ! route -n | awk '$1=="0.0.0.0"{print $8}' | grep -q "$IFACE"; then
    busybox ip route del default 2>/dev/null
    busybox ip route add default via "$GW" dev "$IFACE" 2>/dev/null \
        && log "默认路由改到 $IFACE via $GW"
fi

# 3. 对时
ok=0
for s in $SERVERS; do
    if timeout 15 ntpdate -b "$s" >> "$LOG" 2>&1; then
        log "对时成功: $s -> $(date '+%F %T')"; ok=1; break
    fi
    log "对时失败: $s"
done
[ $ok -eq 1 ] || { log "所有 NTP 都失败，退出"; exit 0; }

# 4. insight_full 的时钟偏移只在这个 flag 发生 0->1 跳变时重算。
#    光是"值为 1"没用——它已经是 1 了，相机会一直用开机那个错的偏移打时间戳。
if [ -f "$FLAG" ]; then
    mkdir -p /userdata/fixbak
    [ -f /userdata/fixbak/is_time_sync.orig ] || cp -a "$FLAG" /userdata/fixbak/is_time_sync.orig
    echo 0 > "$FLAG"; sleep 3        # 监控线程每秒轮询一次，给足余量
    echo 1 > "$FLAG"; sleep 3
    log "已让 $FLAG 跳变 0->1"
else
    log "警告: 找不到 $FLAG，相机时间戳可能仍是错的"
fi

log "=== 完成: $(date '+%F %T') ==="
