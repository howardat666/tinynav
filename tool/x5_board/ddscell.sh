#!/bin/bash
# ⑧ nav关 + 转动 + 大消息 DDS 流量（不碰 BPU、不占内存、CPU 也不高）
set -u
SECS=${1:-55}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1
echo -n "nav 节点(应为空): "; pgrep -f "[c]md_vel_control.py|[m]ap_node.py" | tr '\n' ' '; echo
free -m | awk '/Mem:/{print "  memavail " $7 " MB"}'
echo ""
echo "########## ⑧ nav关 + 转动 + 大消息订阅 ##########"
python3 bighog.py $((SECS+30)) >/dev/null 2>&1 &
BH=$!
sleep 6
python3 spin.py $((SECS+20)) 0.4 >/dev/null 2>&1 &
SP=$!
sleep 6
timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "⑧ nav关+转动+大消息" 2>&1 | grep -v "^\[" \
  | grep -E "load|Image data|VIO restarted|vio_status|vio_image|infra1|insight_full|looper_bridge|合计"
kill -TERM $SP 2>/dev/null; sleep 1; kill -9 $SP 2>/dev/null
kill -9 $BH 2>/dev/null; wait 2>/dev/null
sleep 4
echo "残留: bighog $(pgrep -c -f '[b]ighog.py' 2>/dev/null || echo 0) / spin $(pgrep -c -f '[s]pin.py' 2>/dev/null || echo 0)"
echo "轮子: $(grep -oE '实测=[0-9.]+m/s' $(readlink /proc/$(pgrep -f '[d]iffcar_control.py')/fd/1) | tail -2 | tr '\n' ' ')"
