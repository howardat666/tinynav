#!/bin/bash
# ⑨ nav关 + 【抖动】转动，无任何额外压力。对照 ② 的匀速转动（0 失效）。
set -u
SECS=${1:-55}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1
echo -n "nav 节点(应为空): "; pgrep -f "[c]md_vel_control.py|[m]ap_node.py" | tr '\n' ' '; echo
echo ""
echo "########## ⑨ nav关 + 抖动转动 + 无压力 ##########"
python3 spin_jerky.py $((SECS+20)) 1.0 >/dev/null 2>&1 &
SP=$!
sleep 6
timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "⑨ nav关+抖动转动" 2>&1 | grep -v "^\[" \
  | grep -E "load|Image data|VIO restarted|vio_status|vio_image|infra1|insight_full|合计"
kill -TERM $SP 2>/dev/null; sleep 1.5; kill -9 $SP 2>/dev/null; wait 2>/dev/null
sleep 4
echo "残留 spin: $(pgrep -c -f '[s]pin_jerky' 2>/dev/null || echo 0)"
echo "轮子: $(grep -oE '实测=[0-9.]+m/s' $(readlink /proc/$(pgrep -f '[d]iffcar_control.py')/fd/1) | tail -2 | tr '\n' ' ')"
