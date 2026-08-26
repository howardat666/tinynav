#!/usr/bin/env bash
# Grab one live frame off the board and render it with low_obs_debug.py. Read-only:
# nothing is published, nothing moves. Usage: low_obs_snap.sh [out.png]
set -euo pipefail
BOARD="${BOARD:-root@192.168.19.218}"
PASS="${BOARD_PASS:-looper@0731}"
OUT="${1:-low_obs_snap.png}"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

echo "抓帧..."
sshpass -p "${PASS}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "${BOARD}" \
    "cd /userdata/x5 && . ./env.sh >/dev/null 2>&1; \
     python3 grab_depth.py /userdata/x5/snap_depth.npz 2>&1 | tail -2; \
     python3 grab_scene.py /userdata/x5/snap_scene.npz 2>&1 | tail -2"
sshpass -p "${PASS}" scp -o StrictHostKeyChecking=no \
    "${BOARD}:/userdata/x5/snap_depth.npz" "${BOARD}:/userdata/x5/snap_scene.npz" "${WORK}/"
python3 "$(dirname "$0")/low_obs_debug.py" "${WORK}/snap_depth.npz" \
    --scene-npz "${WORK}/snap_scene.npz" -o "${OUT}"
