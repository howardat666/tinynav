#!/bin/sh
# 板上自动对时。X5 没有 RTC，每次上电时钟都回到 2025-08-26，不修会静默毁掉
# PC 端节点的时间戳配对和所有日志（见 board_bringup.md §2）。
#
# 装成开机服务后就不用再从 PC 跑 sync_board_time.sh：
#   cp board_autotime.sh /userdata/x5/ && cp board-autotime.service /etc/systemd/system/
#   systemctl enable --now board-autotime
set -u

# 不用网关做时间源：实测它自己比公网 NTP 慢 822 秒
SERVERS="ntp.aliyun.com cn.pool.ntp.org ntp1.aliyun.com"
FLAG=/etc/init.d/looper/setting/is_time_sync
LOG=/userdata/x5/logs/autotime.log
WAIT_NET=60

mkdir -p "$(dirname "$LOG")"
log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

# 网卡名由 MAC 生成(enx...)，换板子或从 USB 网卡换成 ESP32 网桥就变，所以探测不写死。
# 只认 en*/wl*：usb0 是 gadget 口，有 IP 也出不了网。
detect_iface() {
    ifconfig 2>/dev/null | awk '
        /^[^ \t]/ { n=""; if ($1 ~ /^(en|wl)/) { n=$1; sub(/:$/, "", n) } }
        /inet /   { if (n != "") { print n; exit } }'
}

log "=== autotime 启动 ==="

# 1. 等网卡拿到 IP（开机时 wifi-connect.sh / USB 枚举可能还没跑完）
IFACE=""
i=0
while [ $i -lt $WAIT_NET ]; do
    IFACE=$(detect_iface)
    [ -n "$IFACE" ] && break
    i=$((i + 1)); sleep 1
done
if [ -z "$IFACE" ]; then
    log "没网(等 ${WAIT_NET}s 没有任何 en*/wl* 拿到 IP)，放弃对时"; exit 0
fi
log "出网网卡 $IFACE (等了 ${i}s)"

# 2. 默认路由：ip_config.sh 无条件给 usb0 加了一条 metric 0 的，而 host 模式下 gadget
#    口没有载波，公网流量全被它吞成黑洞 —— 不删就对不上时。
busybox ip route del default dev usb0 2>/dev/null && log "删掉 usb0 的黑洞默认路由"
if ! route -n | awk '$1=="0.0.0.0"{print $8}' | grep -q "$IFACE"; then
    GW=$(route -n | awk -v i="$IFACE" '$1=="0.0.0.0" && $8==i {print $2; exit}')
    [ -n "$GW" ] || GW=192.168.19.254
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

# 5. 🔴 验证而不是相信。2026-09-22 实测：上面那个 0->1 跳变【打了但没生效】——
#    深度图 header 戳仍滞后 209.024 s，几乎正好等于 ntpdate 的 +208.877 s 跳变量。
#    相机比对时早起 20 秒，它缓存的墙钟基准没跟着跳。而 planning 的同步 slop 只有 0.06 s
#    ⇒ 零回调：局部视图全空、**导航完全不工作**，且一行报错都没有。
#    所以这里实测一次，还是偏就重启相机固件（唯一确定能让它重取基准的办法）。
CHK=/userdata/x5/check_cam_stamp.py
CTL=/etc/init.d/looper/ota_project/scripts/insight-ctl
[ -x "$CTL" ] || CTL=/etc/init.d/ota_project/scripts/insight-ctl
if [ -f "$CHK" ]; then
    . /userdata/x5/env.sh 2>/dev/null || true
    export ROS_LOCALHOST_ONLY=1
    out=$(timeout 30 python3 "$CHK" 8 1.0 2>&1); rc=$?
    log "相机时间戳自检: $out (rc=$rc)"
    if [ "$rc" = "1" ] && [ -x "$CTL" ]; then
        log "时间戳仍偏，重启相机固件"
        sh "$CTL" s99 restart >> "$LOG" 2>&1
        sleep 25
        out=$(timeout 30 python3 "$CHK" 8 1.0 2>&1); rc=$?
        log "重启后复验: $out (rc=$rc)"
    fi
else
    log "警告: 找不到 $CHK，时间戳没法自检"
fi

log "=== 完成: $(date '+%F %T') ==="
