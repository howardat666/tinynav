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
    # 🔴 光有地址不够：2026-09-22 热点冷启动时网卡 0 秒就有个旧地址，而 DHCP 到 82 秒
    # 才 bound，ntpdate 在没有 DNS 的窗口里三个服务器全报 name resolution 失败就放弃了。
    [ -n "$IFACE" ] && route -n | awk '$1=="0.0.0.0"{f=1} END{exit !f}' && break
    IFACE=""
    i=$((i + 1)); sleep 1
done
if [ -z "$IFACE" ]; then
    # 🔴 退 1 而不是 0：服务写着 Restart=on-failure，退 0 的话那条重试规则从来没生效过。
    log "没网(等 ${WAIT_NET}s 没等到默认路由)，放弃对时"; exit 1
fi
log "出网网卡 $IFACE (等了 ${i}s)"

# 2. 默认路由：ip_config.sh 无条件给 usb0 加了一条 metric 0 的，而 host 模式下 gadget
#    口没有载波，公网流量全被它吞成黑洞 —— 不删就对不上时。
busybox ip route del default dev usb0 2>/dev/null && log "删掉 usb0 的黑洞默认路由"
if ! route -n | awk '$1=="0.0.0.0"{print $8}' | grep -q "$IFACE"; then
    # 🔴 别写死办公网网关：换到手机热点(10.140.21.x)后那个地址根本不可达，
    # 装上去只是多一条黑洞路由。不知道网关就交给 netheal 的 udhcpc 去补。
    GW=$(route -n | awk -v i="$IFACE" '$1=="0.0.0.0" && $8==i {print $2; exit}')
    if [ -n "$GW" ]; then
        busybox ip route add default via "$GW" dev "$IFACE" 2>/dev/null \
            && log "默认路由改到 $IFACE via $GW"
    else
        log "$IFACE 上还没有网关，不瞎装路由"
    fi
fi

# 3. 对时
ok=0
for s in $SERVERS; do
    if timeout 15 ntpdate -b "$s" >> "$LOG" 2>&1; then
        log "对时成功: $s -> $(date '+%F %T')"; ok=1; break
    fi
    log "对时失败: $s"
done
[ $ok -eq 1 ] || { log "所有 NTP 都失败，退出(退 1 让 systemd 过 60s 再来)"; exit 1; }

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

# 5. 相机时间戳的验证与修复【不在这里】。autotime 跑在 board-app 之前，那时
#    looper_bridge_node 还没起、/slam/depth 根本不存在 —— 2026-09-22 实测这里只会打
#    "没收到 /slam/depth，判不了"，白占 10 秒关键路径而且什么都没验到。
#    真正的自检+修复在 board-camstamp.service（After=board-app），见 fix_cam_stamp.sh。

log "=== 完成: $(date '+%F %T') ==="
