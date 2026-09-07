#!/bin/bash
# 三种调度方案在相同 CPU 压力下的 A/B。全程不发控制指令，车不会动。
set -u
STRESS=${1:-3}
SECS=${2:-45}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1

BURN='import time
t=time.monotonic()
x=0.0
while time.monotonic()-t < 900:
    x += 1.7'

start_stress() {
    for i in $(seq 1 "$1"); do
        nice -n -5 python3 -c "$BURN" >/dev/null 2>&1 &
    done
    sleep 3
}
stop_stress() {
    for p in $(jobs -p); do kill -9 "$p" 2>/dev/null; done
    wait 2>/dev/null
    sleep 2
}

PLOG=$(readlink /proc/$(pgrep -f "[p]lanning_node.py")/fd/1)

run_one() {
    local scheme="$1"
    echo ""
    echo "################ 方案 $scheme（压力 $STRESS 个 nice -5 进程）################"
    bash /root/car/set_prio.sh "$scheme" | head -3
    start_stress "$STRESS"
    local n0=$(grep -c "decision:" "$PLOG")
    timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "$scheme" 2>&1 | grep -v "^\[" \
        | grep -vE "^  (imu|depth|vio_100hz) " | grep -vE "cam-service|uvicorn|diffcar"
    # 这一段里 planning 的周期与时延
    tail -n +$((n0+1)) "$PLOG" | grep "decision:" \
      | grep -oE "cycle=[0-9.]+s stamp_lag=[0-9.]+s" | sed 's/[a-z_=s]//g' \
      | awk '{c[NR]=$1; l[NR]=$2} END{
          if(NR<3){print "  planning: 样本不足 (" NR ")"; exit}
          asort(c); asort(l);
          printf "  planning 决策 %d 条  cycle p50=%.2f p90=%.2f max=%.2f | stamp_lag p50=%.2f p90=%.2f max=%.2f\n",
                 NR, c[int(NR/2)], c[int(NR*0.9)], c[NR], l[int(NR/2)], l[int(NR*0.9)], l[NR]}'
    stop_stress
}

# 让决策日志有内容：目标锁在车当前位置，距离≈0，车不会动
timeout $(( (SECS+50)*3 + 60 )) python3 hold_target.py $(( (SECS+50)*3 + 30 )) >/dev/null 2>&1 &
HOLD=$!
sleep 4

run_one baseline
run_one camfirst
run_one affinity

echo ""
echo "################ 恢复 ################"
bash /root/car/set_prio.sh restore
kill -9 $HOLD 2>/dev/null; wait 2>/dev/null
echo "残留压力进程: $(pgrep -c -f 'x += 1.7' 2>/dev/null || echo 0)"
