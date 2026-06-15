#!/bin/bash
set -euo pipefail

echo "[RTK] starting rtk_bridge_node at $(date -Is)"
echo "[RTK] serial devices:"
ls -l /dev/ttyCH341USB* /dev/ttyTHS* /dev/ttyUSB* 2>/dev/null || true

existing="$(pgrep -af 'rtk_bridge_node.py' || true)"
if [[ -n "$existing" ]]; then
  echo "[RTK][ERROR] existing rtk_bridge_node.py processes before launch:"
  echo "$existing"
  echo "[RTK][ERROR] Stop old bridge processes first; multiple publishers make /rtk/status unusable."
  echo "[RTK][ERROR] Run: pkill -f rtk_bridge_node.py"
  if [[ "${ALLOW_EXISTING_RTK:-0}" != "1" ]]; then
    exit 1
  fi
fi

echo "[RTK] useful debug topics:"
echo "  ros2 topic echo /rtk/io_status --field data"
echo "  ros2 topic echo /rtk/status"
echo "  cat /tmp/rtk_nmea"

uv run python /tinynav/rtk/rtk_bridge_node.py "$@"
