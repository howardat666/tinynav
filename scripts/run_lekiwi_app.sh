#!/bin/bash
set -euo pipefail

# Start the tinynav web app configured for the LeKiwi base, in one of the three
# comparison configurations.
#
#   bash scripts/run_lekiwi_app.sh vio-vio     # VIO map, VIO navigation (baseline)
#   bash scripts/run_lekiwi_app.sh vio-odom    # VIO map, wheel-odometry navigation
#   bash scripts/run_lekiwi_app.sh odom-odom   # wheel odometry throughout
#
# WHY THE APP AND NOT A TMUX SCRIPT
#   The X5 inside the camera has no tmux and no screen, so scripts/run_*.sh cannot
#   run on the board at all. The app's node manager starts the same processes,
#   supervises them, and exposes bag recording, map building and teleop over HTTP.
#
# WHAT EACH MODE CHANGES
#   Only environment variables; no code path differs. TINYNAV_MAP_ODOM_SOURCE
#   picks the pose the *offline map build* replays out of the bag, and
#   TINYNAV_ODOM_SOURCE picks the pose *navigation* runs on. One recording serves
#   all three, provided it carries both pose topics -- which it does whenever
#   wheel_odometry_node is up during the recording, and it is, because the same
#   node is also the actuator that drove the mapping run.

mode="${1:-}"
case "${mode}" in
    vio-vio)   map_source=vio;   nav_source=vio   ;;
    vio-odom)  map_source=vio;   nav_source=wheel ;;
    odom-odom) map_source=wheel; nav_source=wheel ;;
    *)
        echo "Usage: $0 {vio-vio|vio-odom|odom-odom}" >&2
        echo "  vio-vio    VIO map + VIO navigation (baseline)" >&2
        echo "  vio-odom   VIO map + wheel-odometry navigation" >&2
        echo "  odom-odom  wheel odometry for both" >&2
        exit 1
        ;;
esac

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export TINYNAV_ROBOT_TYPE=lekiwi
export TINYNAV_ACTUATOR=wheel
export TINYNAV_ODOM_SOURCE="${nav_source}"
export TINYNAV_MAP_ODOM_SOURCE="${map_source}"

# Geometry measured on this robot, overridable from the environment. wheel_radius
# came from a driven 3 m straight run against a tape (3.030 m); base_radius from
# two driven spins agreeing to 0.123%. Both are within 2% of the upstream
# defaults, but the wrong one is a scale error on every metre travelled.
export TINYNAV_WHEEL_PORT="${TINYNAV_WHEEL_PORT:-/dev/ttyS3}"
export TINYNAV_WHEEL_RADIUS="${TINYNAV_WHEEL_RADIUS:-0.050385}"
export TINYNAV_BASE_RADIUS="${TINYNAV_BASE_RADIUS:-0.127083}"
export TINYNAV_WHEEL_CAMERA_OFFSET="${TINYNAV_WHEEL_CAMERA_OFFSET:-0.06,0.05,0.18}"

echo "LeKiwi app: map=${map_source} nav=${nav_source} robot=${TINYNAV_ROBOT_TYPE}"
echo "  wheel bus ${TINYNAV_WHEEL_PORT}, r_wheel=${TINYNAV_WHEEL_RADIUS} r_base=${TINYNAV_BASE_RADIUS}"
echo "  camera offset [forward,left,up] = ${TINYNAV_WHEEL_CAMERA_OFFSET}"
echo "  verify once it is up: curl -s localhost:8000/device/platform"

cd "${repo_root}"
exec python3 -m uvicorn app.backend.main:app --host 0.0.0.0 --port "${TINYNAV_APP_PORT:-8000}"
