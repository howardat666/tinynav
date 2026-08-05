#!/bin/bash
set -euo pipefail

# Push the Python sources of this checkout to the X5 inside the Looper camera.
#
#     bash tool/x5_board/sync_to_board.sh              # code only
#     bash tool/x5_board/sync_to_board.sh --with-app   # also app/ (web backend)
#     BOARD=root@169.254.10.1 bash tool/x5_board/sync_to_board.sh
#
# WHY A SCRIPT
#   The board has no rsync and no network route to a git remote, so the checkout
#   at /userdata/x5/tinynav is a hand-copied subset that silently goes stale. It
#   already had: an old wheel_odometry_node, no robot_config.py at all. A stale
#   copy fails as behaviour, not as an error -- the wrong camera offset just tracks
#   badly -- so this exists to make the copy reproducible and to print the md5s
#   back for comparison.
#
# WHAT IT DELIBERATELY DOES NOT TOUCH
#   *.so       the board's aarch64 tinynav_cpp_bind, which this x86 checkout has
#              no counterpart for. Overwriting it bricks every node.
#   models/    TensorRT plans are x86/Jetson artefacts, useless here and large.
#   maps/ voc/ pylibs/ cpp_bind/ pydbow3/   board-side assets, several hundred MB,
#              including the decord shim and the vocabularies.
#   env.sh     belongs to the board.

BOARD="${BOARD:-root@169.254.10.1}"
BOARD_PASS="${BOARD_PASS:-looper@0731}"
BOARD_ROOT="${BOARD_ROOT:-/userdata/x5/tinynav}"
with_app=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-app) with_app=1; shift ;;
        *) echo "Usage: $0 [--with-app]" >&2; exit 1 ;;
    esac
done

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

paths=(tinynav tool tests scripts)
if [[ ${with_app} -eq 1 ]]; then
    paths+=(app/backend)
fi

echo "syncing ${paths[*]} -> ${BOARD}:${BOARD_ROOT}"

# tar over ssh rather than scp -r: one round trip, and --exclude actually works.
tar czf - \
    --exclude='*.so' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='*.plan' \
    --exclude='*.engine' \
    --exclude='tinynav/models' \
    "${paths[@]}" \
| sshpass -p "${BOARD_PASS}" ssh -o StrictHostKeyChecking=no "${BOARD}" \
    "mkdir -p '${BOARD_ROOT}' && tar xzf - -C '${BOARD_ROOT}'"

echo
echo "md5 comparison (local vs board):"
check=(
    tinynav/core/robot_config.py
    tinynav/core/wheel_odometry_node.py
    tinynav/core/planning_node.py
    tinynav/platforms/omni3_kinematics.py
    tinynav/platforms/cmd_vel_control.py
    tool/looper_bridge_node.py
    tool/x5_board/wheel_teleop.py
)
local_sums="$(md5sum "${check[@]}" | awk '{print $1, $2}')"
board_sums="$(sshpass -p "${BOARD_PASS}" ssh -o StrictHostKeyChecking=no "${BOARD}" \
    "cd '${BOARD_ROOT}' && md5sum ${check[*]} 2>&1" | awk '{print $1, $2}')"

fail=0
while read -r sum path; do
    board_sum="$(echo "${board_sums}" | awk -v p="${path}" '$2 == p {print $1}')"
    if [[ "${sum}" == "${board_sum}" ]]; then
        printf '  OK    %s\n' "${path}"
    else
        printf '  DIFF  %s (local %s, board %s)\n' "${path}" "${sum:0:8}" "${board_sum:0:8}"
        fail=1
    fi
done <<< "${local_sums}"

exit "${fail}"
