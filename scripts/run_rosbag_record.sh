#!/bin/bash
set -euo pipefail

# Usage: run_rosbag_record.sh [--output DIR]
#   If --output is not given, a timestamped dir is created under XDG_DATA_HOME/tinynav/rosbags.
#
# The VIO topics are the ones current Looper firmware actually publishes, verified
# against a live camera: /camera/camera/vio_image (PoseStamped, 19.99 Hz) and
# /camera/camera/vio_100hz (99.23 Hz). The /insight/* names this used to list do
# not exist any more, so bags recorded before that fix carry no pose at all.
#
# /wheel/camera_pose and /wheel/odometry come from wheel_odometry_node and are
# what makes one recording serve both mapping comparisons: build_map_node takes
# its pose from a topic, so the same bag builds a VIO map and an odometry map by
# pointing looper_bridge_node's --pose-topic at one or the other. They are simply
# absent from the bag if that node is not running, which costs nothing.

output_dir=""
sensor="looper"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --output|-o) output_dir="$2"; shift 2 ;;
        --sensor) sensor="$2"; shift 2 ;;
        *) echo "Usage: $0 [--output DIR] [--sensor looper|realsense]" >&2; exit 1 ;;
    esac
done

case "${sensor}" in
    looper|realsense) ;;
    *) echo "Unknown --sensor '${sensor}' (looper|realsense)" >&2; exit 1 ;;
esac

if [ -z "$output_dir" ]; then
    xdg_data_home="${XDG_DATA_HOME:-$HOME/.local/share}"
    record_root="${xdg_data_home}/tinynav/rosbags"
    timestamp="$(date +%Y%m%d_%H%M%S)"
    output_dir="${record_root}/map_record_${timestamp}"
    mkdir -p "${record_root}"
else
    mkdir -p "$(dirname "$output_dir")"
fi

# Topics every offline map build needs, traced from what actually consumes them.
# looper_bridge_node: infra1 image + depth + infra1 camera_info + a pose topic.
# build_map_node: infra2 camera_info (the only place the stereo baseline comes
# from -- infra1's P[3] is 0), color camera_info, /tf, /tf_static, and the *raw*
# color image, which its own ImageTransportsNode decompresses out of
# color/image_rect_raw/compressed, so only the compressed one has to be recorded.
common_topics=(
    /camera/camera/infra1/camera_info
    /camera/camera/infra1/image_rect_raw
    /camera/camera/infra2/camera_info
    /camera/camera/depth/image_rect_raw
    /camera/camera/color/camera_info
    /camera/camera/color/image_rect_raw/compressed
    /camera/camera/imu
    /tf
    /tf_static
)

if [[ "${sensor}" == "looper" ]]; then
    # infra2/image_rect_raw is deliberately absent: nothing on the Looper path
    # consumes it. The camera supplies depth directly, so no node runs stereo
    # matching, and dropping it saves ~7 MB/s of eMMC write bandwidth
    # (544x640 mono8 at 20 Hz) on a board that is also running the firmware.
    #
    # Both wheel topics are recorded so that one recording can build both maps:
    # build_map_node takes its pose from a topic, so the VIO map and the odometry
    # map differ only in looper_bridge_node's --pose-topic. They are simply absent
    # if wheel_odometry_node is not running, which costs nothing.
    sensor_topics=(
        /camera/camera/vio_image
        /camera/camera/vio_100hz
        /camera/camera/vio_status
        /wheel/camera_pose
        /wheel/odometry
    )
else
    # RealSense: perception_node does its own stereo, so both images are required.
    sensor_topics=(
        /camera/camera/infra2/image_rect_raw
        /camera/camera/color/image_raw
    )
fi

echo "recording (${sensor}) -> ${output_dir}"
# exec, not a plain call. Without it this wrapper stays alive as the parent and a
# SIGINT delivered to the wrapper's pid never reaches `ros2 bag record`, which then
# keeps writing after the caller believes it stopped and never gets to flush
# metadata.yaml -- leaving a .db3 that `ros2 bag info` rejects with "Could not find
# metadata in bag directory". Measured on the board: a bag that looked stopped at
# 368 MB was still growing past 1 GB.
exec ros2 bag record \
    --output "${output_dir}" \
    --max-cache-size 2147483648 \
    "${common_topics[@]}" \
    "${sensor_topics[@]}"
