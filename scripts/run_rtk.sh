#!/bin/bash
set -euo pipefail

uv run python /tinynav/rtk/rtk_bridge_node.py "$@"
