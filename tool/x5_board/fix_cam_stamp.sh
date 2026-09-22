#!/bin/sh
# 相机时间戳的自检与修复。必须在 board-app 起来之后跑：/slam/depth 由 looper_bridge_node
# 发，而 bridge 是 board-app 拉起来的 —— 放在 autotime 里只会打"没收到，判不了"。
#
# 为什么必须修：相机固件 insight_full 在启动时缓存墙钟基准，**之后系统时钟跳变它不跟**。
# 板子没有 RTC，开机先恢复旧时间、等网络通了才 NTP 一跳（实测见过 +208.88s 和 +91.49s），
# 相机若比对时先起，戳就永远停在跳变前。planning 的同步 slop 只有 0.06s ⇒ 零回调：
# 局部视图/热力图全空、**导航也完全不工作**，而且一行报错都没有。2026-09-22 实测。
set -u
LOG=/userdata/x5/logs/camstamp.log
CHK=/userdata/x5/check_cam_stamp.py
FLAG=/etc/init.d/looper/setting/is_time_sync
CTL=/etc/init.d/looper/ota_project/scripts/insight-ctl
[ -x "$CTL" ] || CTL=/etc/init.d/ota_project/scripts/insight-ctl
mkdir -p "$(dirname "$LOG")"
log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

. /userdata/x5/env.sh 2>/dev/null || true
export ROS_LOCALHOST_ONLY=1

log "=== 启动 ==="
# 等话题真出现。bridge 起来要时间，而"没收到"和"戳不对"是两回事，不能混。
i=0
while [ $i -lt 60 ]; do
    timeout 20 python3 "$CHK" 4 1.0 >/dev/null 2>&1
    [ $? -ne 2 ] && break
    i=$((i + 5)); sleep 5
done

out=$(timeout 30 python3 "$CHK" 8 1.0 2>&1); rc=$?
log "自检: $out (rc=$rc)"
[ "$rc" = "0" ] && { log "=== 正常，不动 ==="; exit 0; }
[ "$rc" = "2" ] && { log "=== 等了 ${i}s 仍收不到深度，放弃（不是时间戳问题） ==="; exit 0; }

# 第 1 级：让相机重算偏移。只有 0->1 的跳变有用，光是"值为 1"没用。
if [ -f "$FLAG" ]; then
    echo 0 > "$FLAG"; sleep 3; echo 1 > "$FLAG"; sleep 8
    out=$(timeout 30 python3 "$CHK" 8 1.0 2>&1); rc=$?
    log "切 is_time_sync 后: $out (rc=$rc)"
    [ "$rc" = "0" ] && { log "=== 第1级修好 ==="; exit 0; }
fi

# 第 2 级：重启相机固件。实测唯一确定能让它重取基准的办法（209.019s -> 0.163s）。
if [ -x "$CTL" ]; then
    log "仍偏，重启相机固件"
    sh "$CTL" s99 restart >> "$LOG" 2>&1
    sleep 30
    out=$(timeout 30 python3 "$CHK" 8 1.0 2>&1); rc=$?
    log "重启后: $out (rc=$rc)"
fi
log "=== 完成 rc=$rc ==="
