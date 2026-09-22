#!/bin/bash
set -euo pipefail

# Push the Python sources of this checkout to the X5 inside the Looper camera.
#
#     bash tool/x5_board/sync_to_board.sh              # code only
#     bash tool/x5_board/sync_to_board.sh --with-app   # also app/ (web backend)
#     bash tool/x5_board/sync_to_board.sh --with-web   # also the built Flutter bundle
#     BOARD=root@169.254.10.1 bash tool/x5_board/sync_to_board.sh
#
# --with-web needs app/frontend/build/web to exist, which means it was built first.
# The board has no Flutter and does not need one: `flutter build web` emits plain
# HTML/JS/WASM with no architecture in it, so an x86 host builds a bundle the
# aarch64 board serves unchanged. There is no Flutter on this host either -- it
# lives in the uniflexai/tinynav:latest image at /opt/flutter/bin:
#
#     docker run --rm -v "$PWD/app/frontend":/fe -w /fe --entrypoint bash \
#       uniflexai/tinynav:latest -c \
#       'export PATH=$PATH:/opt/flutter/bin; flutter pub get && flutter build web --release'
#
# (that writes build/ as root; building a copy elsewhere and moving it in avoids
# root-owned files in the checkout.)
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

# WiFi, not the 169.254.10.1 USB link: the type-C port runs as host now (it drives the
# WiFi dongle), so the gadget side has an IP but no carrier and this script just hung.
BOARD="${BOARD:-root@192.168.19.218}"
BOARD_PASS="${BOARD_PASS:-looper@0731}"
BOARD_ROOT="${BOARD_ROOT:-/userdata/x5/tinynav}"
with_app=0
with_web=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-app) with_app=1; shift ;;
        --with-web) with_web=1; shift ;;
        *) echo "Usage: $0 [--with-app] [--with-web]" >&2; exit 1 ;;
    esac
done

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

paths=(tinynav tool tests scripts)
if [[ ${with_app} -eq 1 ]]; then
    paths+=(app/backend)
fi
if [[ ${with_web} -eq 1 ]]; then
    if [[ ! -f app/frontend/build/web/index.html ]]; then
        echo "--with-web: app/frontend/build/web/index.html missing -- build it first (see header)" >&2
        exit 1
    fi
    # 🔴 整个 build/web 是 34 MB，大头是 canvaskit，而这条链路(板子->USB->ESP32->WiFi)
    # 实测只有约 52 KB/s —— 全量必然超时，而中断点落在 canvaskit.wasm 上会把它截断，
    # 网页整个打不开(2026-09-22 踩过)。改动之间实际变的通常只有 main.dart.js。
    # 所以先问板子要 md5，只传对不上的。
    echo "web: 比对 md5，只传变化的文件"
    board_md5="$(sshpass -p "${BOARD_PASS}" ssh -o StrictHostKeyChecking=no "${BOARD}" \
        "cd '${BOARD_ROOT}/app/frontend/build/web' 2>/dev/null && find . -type f -exec md5sum {} +" 2>/dev/null || true)"
    web_changed=()
    while IFS= read -r f; do
        want="$(md5sum "app/frontend/build/web/${f}" | cut -d' ' -f1)"
        have="$(printf '%s\n' "${board_md5}" | awk -v k="./${f}" '$2==k{print $1}')"
        [[ "${want}" == "${have}" ]] || web_changed+=("app/frontend/build/web/${f}")
    done < <(cd app/frontend/build/web && find . -type f -printf '%P\n')
    if [[ ${#web_changed[@]} -eq 0 ]]; then
        echo "web: 板上已是最新，跳过"
    else
        echo "web: 需要传 ${#web_changed[@]} 个文件"
        paths+=("${web_changed[@]}")
    fi
fi

echo "syncing ${paths[*]} -> ${BOARD}:${BOARD_ROOT}"

# tar over ssh rather than scp -r: one round trip, and --exclude actually works.
# z 不是可选项：链路约 52 KB/s，main.dart.js 3.0 MB 压完只剩 0.9 MB。
tar czf - \
    --exclude='*.so' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='*.plan' \
    --exclude='*.engine' \
    --exclude='tinynav/models/*.onnx' \
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
    tinynav/platforms/diffcar_control.py
    tinynav/core/lat_stats.py
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
