#!/bin/bash
set -euo pipefail

map_path="${1:-PATH/TO/MAP}"
run_rviz="${RUN_RVIZ:-0}"
run_pois="${RUN_POIS:-0}"

tmux new-session \; \
  split-window -h \; \
  split-window -v \; \
  select-pane -t 0 \; split-window -v \; \
  select-pane -t 3 \; split-window -v \; \
  select-pane -t 0 \; send-keys 'uv run python /tinynav/rtk/rtk_bridge_node.py' C-m \; \
  select-pane -t 1 \; send-keys 'uv run python /tinynav/rtk/rtk_fusion_node.py' C-m \; \
  select-pane -t 2 \; send-keys 'uv run python /tinynav/tinynav/core/perception_node.py' C-m \; \
  select-pane -t 3 \; send-keys 'uv run python /tinynav/tinynav/core/planning_node.py' C-m \; \
  select-pane -t 4 \; send-keys "uv run python /tinynav/tinynav/core/map_node.py --tinynav_map_path $map_path" C-m \; \
  select-pane -t 5 \; send-keys 'uv run python /tinynav/tinynav/platforms/cmd_vel_control.py --ros-args -p odom_topic:=/slam/odometry_fused' C-m

if [[ "$run_rviz" == "1" ]]; then
  tmux split-window -v
  tmux send-keys 'ros2 run rviz2 rviz2 -d /tinynav/docs/vis.rviz' C-m
fi

if [[ "$run_pois" == "1" ]]; then
  tmux split-window -v
  tmux send-keys "sleep 3 && uv run python /tinynav/tool/pub_pois.py --tinynav_map_path $map_path" C-m
fi
