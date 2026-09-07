#!/bin/bash
# nav 开着的两格：静止 / 转动（转动由规划器驱动，就是失效过的那个条件）
set -u
SECS=${1:-55}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1

python3 -c "
import urllib.request
r=urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:8000/nav/nodes/enable',data=b'{}',headers={'Content-Type':'application/json'},method='POST'),timeout=30)
print('enable nav:', r.read().decode()[:60])
"
echo "等 45 秒（numba 编译 + 重定位起来）"; sleep 45
echo -n "nav 节点: "; pgrep -f "[c]md_vel_control.py|[m]ap_node.py" | tr '\n' ' '; echo

echo ""
echo "########## ⑤ nav开 + 静止 ##########"
timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "⑤ nav开+静止" 2>&1 | grep -v "^\[" \
  | grep -E "load|Image data|VIO restarted|vio_status|vio_image|infra1|insight_full|map_node|合计"
sleep 8

echo ""
echo "########## ⑥ nav开 + 转动  ★失效过的条件 ##########"
python3 hold_target.py $((SECS+25)) >/dev/null 2>&1 &
HP=$!
sleep 6
timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "⑥ nav开+转动" 2>&1 | grep -v "^\[" \
  | grep -E "load|Image data|VIO restarted|vio_status|vio_image|infra1|insight_full|map_node|合计"
kill -9 $HP 2>/dev/null; wait 2>/dev/null
sleep 3
echo ""
echo "残留 hold: $(pgrep -c -f '[h]old_target' 2>/dev/null || echo 0)"
echo "轮子: $(grep -oE '实测=[0-9.]+m/s' $(readlink /proc/$(pgrep -f '[d]iffcar_control.py')/fd/1) | tail -2 | tr '\n' ' ')"
