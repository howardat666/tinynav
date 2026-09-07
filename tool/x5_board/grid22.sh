#!/bin/bash
# 2x2 分离实验：相机在动 vs CPU 压力。nav 关着，直接发 /cmd_vel 原地转。
set -u
SECS=${1:-50}
NSTRESS=${2:-6}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1
BURN='import time
t=time.monotonic()
x=0.0
while time.monotonic()-t < 900:
    x += 1.7'

cell() {   # $1=标签  $2=压力个数  $3=转不转
    echo ""
    echo "########## $1 ##########"
    local sp=() hp=""
    for i in $(seq 1 "$2"); do
        nice -n -5 python3 -c "$BURN" >/dev/null 2>&1 &
        sp+=($!)
    done
    if [ "$3" = "1" ]; then
        python3 spin.py $((SECS+20)) 0.4 >/dev/null 2>&1 &
        hp=$!
        sleep 6                       # 转起来，等 vio_status 变成 TRACKING
    fi
    sleep 2
    timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "$1" 2>&1 | grep -v "^\[" \
      | grep -E "load|Image data|VIO restarted|vio_status|vio_image|infra1|insight_full|合计"
    for p in "${sp[@]:-}"; do [ -n "${p:-}" ] && kill -9 "$p" 2>/dev/null; done
    [ -n "$hp" ] && { kill -TERM "$hp" 2>/dev/null; sleep 1; kill -9 "$hp" 2>/dev/null; }
    wait 2>/dev/null
    sleep 12                          # 停稳 + 降温
}

echo "开跑前: $(cat /proc/loadavg | cut -d' ' -f1-3)  温度 $(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)C"
cell "① 静止 + 无压力"        0 0
cell "② 转动 + 无压力  ★关键" 0 1
cell "③ 静止 + 压力$NSTRESS"  $NSTRESS 0
cell "④ 转动 + 压力$NSTRESS"  $NSTRESS 1
echo ""
echo "残留: 压力 $(pgrep -c -f 'x += 1.7' 2>/dev/null || echo 0) / spin $(pgrep -c -f '[s]pin.py' 2>/dev/null || echo 0)"
echo "轮子: $(grep -oE '实测=[0-9.]+m/s' $(readlink /proc/$(pgrep -f '[d]iffcar_control.py')/fd/1) | tail -2 | tr '\n' ' ')"
