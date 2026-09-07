#!/bin/bash
# 三种调度方案，随时可切、全部可逆（重启即恢复默认）。
#   baseline : 恢复现状 —— 相机 nice 0，我们的节点 -5/-10
#   camfirst : 相机提到 nice -15（仍是 CFS，不是实时调度，不会绝对抢占）
#   affinity : 核绑定分区 —— 相机独占 0-2 核，我们的节点用 3-7 核，互不排队
#   restore  : 解除核绑定并恢复 nice
set -u
SCHEME="${1:-baseline}"

cam_pids() { pgrep -x cam-service; pgrep -x insight_full; }
our_pids() { pgrep -f "[t]inynav/tinynav/core|[t]inynav/tinynav/platforms|[t]inynav/tool/looper_bridge|[u]vicorn app.backend"; }

renice_all() {   # $1=nice  $2...=pids   逐线程做，renice -p PID 改不到其余线程
    local n="$1"; shift
    local c=0
    for p in "$@"; do
        for t in /proc/$p/task/*; do
            renice -n "$n" -p "$(basename "$t")" >/dev/null 2>&1 && c=$((c+1))
        done
    done
    echo "$c"
}
affine_all() {   # $1=掩码  $2...=pids
    local m="$1"; shift
    local c=0
    for p in "$@"; do
        taskset -a -p "$m" "$p" >/dev/null 2>&1 && c=$((c+1))
    done
    echo "$c"
}

case "$SCHEME" in
  baseline)
    echo "  相机恢复 nice 0 : $(renice_all 0 $(cam_pids)) 个线程"
    echo "  全部解除核绑定  : $(affine_all 0xff $(cam_pids) $(our_pids)) 个进程"
    ;;
  camfirst)
    echo "  相机提到 nice -15 : $(renice_all -15 $(cam_pids)) 个线程"
    echo "  全部解除核绑定    : $(affine_all 0xff $(cam_pids) $(our_pids)) 个进程"
    ;;
  affinity)
    echo "  相机恢复 nice 0   : $(renice_all 0 $(cam_pids)) 个线程"
    echo "  相机绑 0-2 核     : $(affine_all 0x07 $(cam_pids)) 个进程"
    echo "  我们的绑 3-7 核   : $(affine_all 0xf8 $(our_pids)) 个进程"
    ;;
  restore)
    echo "  相机恢复 nice 0 : $(renice_all 0 $(cam_pids)) 个线程"
    echo "  解除核绑定      : $(affine_all 0xff $(cam_pids) $(our_pids)) 个进程"
    ;;
  *) echo "未知方案 $SCHEME"; exit 1 ;;
esac

echo "--- 当前 nice / 亲和性 ---"
for p in $(cam_pids) $(our_pids); do
    nm=$(tr '\0' ' ' < /proc/$p/cmdline | grep -oE 'insight_full|cam-service|looper_bridge_node|map_node|planning_node|cmd_vel_control|diffcar_control|uvicorn' | head -1)
    printf "  %-18s nice=%-4s cpus=%s\n" "${nm:-$p}" \
        "$(awk '{print $19}' /proc/$p/stat)" \
        "$(taskset -p $p 2>/dev/null | grep -oE '[0-9a-f]+$')"
done
