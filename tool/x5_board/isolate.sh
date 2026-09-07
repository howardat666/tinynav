#!/bin/bash
# 分离两个变量：CPU 压力 vs 相机在动。四格全测。
set -u
SECS=${1:-50}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1
BURN='import time
t=time.monotonic()
x=0.0
while time.monotonic()-t < 900:
    x += 1.7'

run() {   # $1=标签  $2=压力个数  $3=要不要转(1/0)
    echo ""
    echo "########## $1 ##########"
    local pids=()
    for i in $(seq 1 "$2"); do
        nice -n -5 python3 -c "$BURN" >/dev/null 2>&1 &
        pids+=($!)
    done
    local hp=""
    if [ "$3" = "1" ]; then
        python3 hold_target.py $((SECS+25)) >/dev/null 2>&1 &
        hp=$!
        sleep 6                      # 等它转起来，VIO 进入 TRACKING
    fi
    sleep 2
    timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "$1" 2>&1 | grep -v "^\[" \
      | grep -E "load|Image data|VIO restarted|vio_status|vio_image|insight_full|合计"
    for p in "${pids[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    [ -n "$hp" ] && kill -9 "$hp" 2>/dev/null
    wait 2>/dev/null
    sleep 8                          # 让车停稳、温度回落一点
}

run "① 静止 + 无压力"   0 0
run "② 静止 + 压力3"    3 0
run "③ 转动 + 无压力"   0 1
run "④ 转动 + 压力3"    3 1

echo ""
echo "残留: 压力 $(pgrep -c -f 'x += 1.7' 2>/dev/null || echo 0) / hold $(pgrep -c -f '[h]old_target' 2>/dev/null || echo 0)"
