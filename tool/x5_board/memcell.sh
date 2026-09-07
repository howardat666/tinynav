#!/bin/bash
# 把 nav 关掉，用纯内存占用替代 map_node，看是不是内存压力打死 VIO 的
set -u
SECS=${1:-55}
MB=${2:-600}
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1

python3 -c "
import urllib.request
r=urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:8000/nav/nodes/disable',data=b'{}',headers={'Content-Type':'application/json'},method='POST'),timeout=30)
print('disable nav:', r.read().decode()[:60])
"
sleep 8
echo -n "nav 节点(应为空): "; pgrep -f "[c]md_vel_control.py|[m]ap_node.py" | tr '\n' ' '; echo
free -m | awk '/Mem:/{print "  基线 memavail " $7 " MB"}'

echo ""
echo "########## ⑦ nav关 + 转动 + 内存占用 ${MB}MB ##########"
python3 memhog.py "$MB" $((SECS+30)) &
HG=$!
sleep 8
free -m | awk '/Mem:/{print "  占用后 memavail " $7 " MB"}'
python3 spin.py $((SECS+20)) 0.4 >/dev/null 2>&1 &
SP=$!
sleep 6
timeout $((SECS+40)) python3 perf_monitor.py "$SECS" "⑦ nav关+转动+内存${MB}MB" 2>&1 | grep -v "^\[" \
  | grep -E "load|Image data|VIO restarted|vio_status|vio_image|infra1|insight_full|合计"
kill -TERM $SP 2>/dev/null; sleep 1; kill -9 $SP 2>/dev/null
kill -9 $HG 2>/dev/null
wait 2>/dev/null
sleep 5
free -m | awk '/Mem:/{print "  恢复后 memavail " $7 " MB"}'
echo "残留: memhog $(pgrep -c -f '[m]emhog.py' 2>/dev/null || echo 0) / spin $(pgrep -c -f '[s]pin.py' 2>/dev/null || echo 0)"
