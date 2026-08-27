#!/bin/bash
# 在 docker 里起 planning lab 闭环仿真（宿主机没有 uv，镜像的系统 python 已带齐依赖）。
#
#   scripts/run_planning_sim_docker.sh            # 前台起，浏览器开 http://localhost:8766
#   scripts/run_planning_sim_docker.sh &          # 后台起，然后跑无头场景：
#   python3 tool/simulator/scenarios.py
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
  --entrypoint bash "$IMAGE" -lc '
set +u; source /opt/ros/humble/setup.bash; set -u
exec python3 tool/simulator/ros_planning_web.py'
