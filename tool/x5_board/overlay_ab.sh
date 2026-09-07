#!/bin/bash
# A/B：planning 的可视化发布开/关，看规划周期差多少。可逆。
set -u
cd /root/car
. /userdata/x5/env.sh >/dev/null 2>&1

rate() {
    timeout 40 python3 - <<'PY'
import time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
got=[]
rclpy.init(); n=rclpy.create_node("prate")
n.create_subscription(Path,"/planning/trajectory_path",lambda m: got.append(time.monotonic()),
                      QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
t0=time.monotonic()
while time.monotonic()-t0 < 25: rclpy.spin_once(n, timeout_sec=0.05)
if len(got)>2:
    iv=sorted(got[i]-got[i-1] for i in range(1,len(got)))
    print("  规划周期 %.2f Hz   间隔 p50=%.2fs p90=%.2fs max=%.2fs" %
          (len(got)/25, iv[len(iv)//2], iv[int(len(iv)*0.9)], iv[-1]))
else:
    print("  规划周期 样本不足 (%d)" % len(got))
rclpy.shutdown()
PY
}

echo "########## A: 可视化【开】（现状）##########"
rate
python3 /root/car/cpu_top.py 2>&1 | grep -E "planning_node|uvicorn"

echo ""
echo "########## 关掉可视化并重启 ##########"
grep -q TINYNAV_PUBLISH_PLANNING_OVERLAYS /userdata/x5/env.sh \
  && sed -i 's|^export TINYNAV_PUBLISH_PLANNING_OVERLAYS=.*|export TINYNAV_PUBLISH_PLANNING_OVERLAYS=0|' /userdata/x5/env.sh \
  || echo 'export TINYNAV_PUBLISH_PLANNING_OVERLAYS=0' >> /userdata/x5/env.sh
bash /root/car/appreset.sh >/dev/null 2>&1
sleep 30
P=$(pgrep -f "[p]lanning_node.py"); echo "  生效值: $(tr '\0' '\n' < /proc/$P/environ | grep OVERLAYS)"

echo ""
echo "########## B: 可视化【关】##########"
rate
python3 /root/car/cpu_top.py 2>&1 | grep -E "planning_node|uvicorn"

echo ""
echo "########## 恢复 ##########"
sed -i '/^export TINYNAV_PUBLISH_PLANNING_OVERLAYS=0$/d' /userdata/x5/env.sh
grep -c OVERLAYS /userdata/x5/env.sh
bash /root/car/appreset.sh >/dev/null 2>&1
sleep 25
P=$(pgrep -f "[p]lanning_node.py"); echo "  恢复后: $(tr '\0' '\n' < /proc/$P/environ | grep OVERLAYS)"
