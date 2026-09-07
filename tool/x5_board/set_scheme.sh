#!/bin/bash
# 切换里程计方案。a=全 VIO  b=VIO建图/轮速导航  c=全轮速。可逆：再跑一次换回去即可。
set -eu
S="${1:?用法: set_scheme.sh a|b|c}"
case "$S" in a|b|c) ;; *) echo "只能是 a/b/c"; exit 1 ;; esac
U=/etc/systemd/system/board-app.service
sed -i "s|^Environment=TINYNAV_SCHEME=.*|Environment=TINYNAV_SCHEME=${S}|" "$U"
systemctl daemon-reload
grep -n TINYNAV_SCHEME "$U"
bash /root/car/appreset.sh >/dev/null 2>&1
sleep 32
echo "--- 生效检查：三个 pose_topic 必须一起换 ---"
for pat in "[l]ooper_bridge_node" "[p]lanning_node" "[c]md_vel_control"; do
    p=$(pgrep -f "$pat" | head -1)
    [ -z "$p" ] && { echo "  $pat 没在跑"; continue; }
    nm=$(tr '\0' ' ' < /proc/$p/cmdline | grep -oE 'looper_bridge_node|planning_node|cmd_vel_control' | head -1)
    tp=$(tr '\0' '\n' < /proc/$p/cmdline | grep -A0 -E '^(/camera/camera/vio_image|/wheel/camera_pose|pose_topic:=.*)$' | tail -1)
    echo "  $(printf '%-20s' "$nm") $tp"
done
echo "  app.scheme: $(cat /userdata/x5/logs/app.scheme 2>/dev/null)"
echo "  ODOM_SOURCE: $(tr '\0' '\n' < /proc/$(pgrep -f '[p]lanning_node' | head -1)/environ | grep -E '^TINYNAV_(ODOM|MAP_ODOM)_SOURCE')"
