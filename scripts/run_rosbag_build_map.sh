#!/bin/bash
set -euo pipefail

rosbag_path=$(uv run hf download --repo-type dataset --cache-dir /tinynav UniflexAI/rosbag2_go2_looper)
map_save_path=/tinynav/output/map_go2_looper

mode="${1:-perception}" # looper_direct | perception
use_rtk_fused_mapping="${USE_RTK_FUSED_MAPPING:-0}"
build_cmd="uv run python /tinynav/tinynav/core/build_map_node.py --map_save_path $map_save_path --bag_file $rosbag_path"
if [[ "${use_rtk_fused_mapping}" == "1" ]]; then
  build_cmd="$build_cmd --use_rtk_fused_odom_for_mapping"
fi

if [[ "${mode}" == "looper_direct" ]]; then
  source_cmd='uv run python /tinynav/tool/looper_bridge_node.py'
elif [[ "${mode}" == "perception" ]]; then
  source_cmd='uv run python /tinynav/tinynav/core/perception_node.py'
else
  echo "Usage: $0 [looper_direct|perception]"
  exit 1
fi

if [[ "${use_rtk_fused_mapping}" == "1" ]]; then
  tmux new-session \; \
    split-window -h \; \
    split-window -v \; \
    select-pane -t 0 \; split-window -v \; \
    select-pane -t 3 \; split-window -v \; \
    select-pane -t 1 \; send-keys "$build_cmd" C-m \; \
    select-pane -t 2 \; send-keys "${source_cmd}" C-m \; \
    select-pane -t 3 \; send-keys 'uv run python /tinynav/rtk/rtk_fusion_node.py' C-m \; \
    select-pane -t 4 \; send-keys 'ros2 run rviz2 rviz2 -d /tinynav/docs/vis.rviz' C-m
else
  tmux new-session \; \
    split-window -h \; \
    split-window -v \; \
    select-pane -t 0 \; split-window -v \; \
    select-pane -t 1 \; send-keys "$build_cmd" C-m \; \
    select-pane -t 2 \; send-keys "${source_cmd}" C-m \; \
    select-pane -t 3 \; send-keys 'ros2 run rviz2 rviz2 -d /tinynav/docs/vis.rviz' C-m
fi
