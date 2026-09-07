#!/bin/bash
# 在 docker 里起 planning lab 闭环仿真（宿主机没有 uv，镜像的系统 python 已带齐依赖）。
#
#   scripts/run_planning_sim_docker.sh            # 前台起，浏览器开 http://localhost:8766
#   scripts/run_planning_sim_docker.sh &          # 后台起，然后跑无头场景：
#   python3 tool/simulator/scenarios.py
#
# 起来就已经是**板上那套配置**（tool/simulator/x5_presets.py 的 BOARD_ENV，来自
# app_start.sh）并预载了一个场景，浏览器一开就在跑。换场景用页面左边 Scene 下拉框，
# 或开一个别的：TINYNAV_SIM_SCENE=chair_legs scripts/run_planning_sim_docker.sh
#
# 相机默认 match 档：视场和**角分辨率**都和实机一致（0.4 倍缩放 + step=2，
# 2/123.8 rad 对上板上的 5/309.5 rad），渲染 25 ms/帧。TINYNAV_SIM_CAMERA=full 是
# 逐像素一致但 263 ms/帧，仿真会掉到约 3.8 Hz，只在复核时用。
#
# ROS_DOMAIN_ID 用 77 而不是默认 0：0 上有办公室里别人的 ROS 2 发现报文，一条畸形的
# ParticipantEntitiesInfo 能让 FastCDR 吃掉 1.5 GB。ROS_LOCALHOST_ONLY 同理。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${TINYNAV_IMAGE:-uniflexai/tinynav:latest}"

docker rm -f tinynav_sim >/dev/null 2>&1 || true
exec docker run --rm --net=host --name tinynav_sim \
  -v "$ROOT":/tinynav -w /tinynav \
  -e PYTHONPATH=/tinynav \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}" \
  -e OPENBLAS_NUM_THREADS=2 -e OMP_NUM_THREADS=2 \
  -e TINYNAV_ALLOW_REVERSE="${TINYNAV_ALLOW_REVERSE:-0}" \
  -e TINYNAV_ROUTE_COST="${TINYNAV_ROUTE_COST:-1}" \
  -e TINYNAV_TRAJ_SAMPLES="${TINYNAV_TRAJ_SAMPLES:-15}" \
  -e TINYNAV_SIM_SCENE="${TINYNAV_SIM_SCENE:-wall_ahead}" \
  -e TINYNAV_SIM_PORT="${TINYNAV_SIM_PORT:-8766}" \
  -e TINYNAV_GRID_OFFSET_Z="${TINYNAV_GRID_OFFSET_Z:-0.15}" \
  -e TINYNAV_W_IDLE="${TINYNAV_W_IDLE:-40.0}" \
  -e TINYNAV_IDLE_VX_EPS="${TINYNAV_IDLE_VX_EPS:-1e-6}" \
  -e TINYNAV_SIM_CAMERA="${TINYNAV_SIM_CAMERA:-match}" \
  --entrypoint bash "$IMAGE" -lc '
set +u; source /opt/ros/humble/setup.bash; set -u
exec python3 tool/simulator/ros_planning_web.py'
