#!/bin/bash
# 在 docker 里起【上游】那份 planning lab 仿真（宿主机没有 uv，上游脚本用 uv run 起不来）。
#
#   scripts/run_upstream_sim_docker.sh          # 浏览器开 http://localhost:8766
#
# 代码取自 git worktree /home/dm/looper/tinynav-upstream（= upstream/main），和我们的
# x5 分支完全隔离。要和我们那份同时开：
#   scripts/run_upstream_sim_docker.sh &                      # 上游，8766（端口写死）
#   TINYNAV_SIM_PORT=8767 scripts/run_planning_sim_docker.sh & # 我们的，8767
# ROS_DOMAIN_ID 也必须错开（这里 78，我们那份 77），否则两个仿真会互相收到对方的话题。
set -euo pipefail
ROOT="${TINYNAV_UPSTREAM_ROOT:-/home/dm/looper/tinynav-upstream}"
IMAGE="${TINYNAV_IMAGE:-uniflexai/tinynav:latest}"
[ -d "$ROOT/tool/simulator" ] || { echo "找不到上游代码：$ROOT"; exit 1; }

docker rm -f tinynav_sim_upstream >/dev/null 2>&1 || true
exec docker run --rm --net=host --name tinynav_sim_upstream \
  -v "$ROOT":/tinynav -w /tinynav \
  -e PYTHONPATH=/tinynav \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}" \
  -e OPENBLAS_NUM_THREADS=2 -e OMP_NUM_THREADS=2 \
  -e TINYNAV_DB_PATH=/tinynav/tinynav_db \
  --entrypoint bash "$IMAGE" -lc '
set +u; source /opt/ros/humble/setup.bash; set -u
exec python3 tool/simulator/ros_planning_web.py'
