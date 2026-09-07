#!/usr/bin/env bash
# 判决图手动调参网页。跑在 docker 里（要 numba），数据用 data/obstacle_scenes/*.npz。
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${TINYNAV_TUNER_PORT:-8768}"
NAME=tinynav_verdict_tuner
docker rm -f "$NAME" >/dev/null 2>&1 || true
echo "打开 http://127.0.0.1:${PORT}   (ctrl-c 停)"
exec docker run --rm --name "$NAME" --entrypoint bash \
  -p "${PORT}:${PORT}" \
  -v "${REPO}:/tinynav" -w /tinynav \
  -e TINYNAV_TUNER_PORT="${PORT}" \
  -e TINYNAV_ROBOT_TYPE="${TINYNAV_ROBOT_TYPE:-diffcar}" \
  -e TINYNAV_CAMERA_HEIGHT_M="${TINYNAV_CAMERA_HEIGHT_M:-0.124}" \
  -e TINYNAV_RAYCAST_STEP="${TINYNAV_RAYCAST_STEP:-3}" \
  -e TINYNAV_GRID_OFFSET_Z="${TINYNAV_GRID_OFFSET_Z:-0.1625}" \
  uniflexai/tinynav:latest \
  -lc '. /opt/ros/humble/setup.bash && python3 tool/verdict_tuner.py'
