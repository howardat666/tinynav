#!/usr/bin/env bash
# Environment for running tinynav Python nodes on the D-Robotics X5 inside the
# Looper camera.  Source it, don't execute it:
#
#     . /userdata/x5/env.sh
#
# WHY THIS FILE EXISTS
# --------------------
# The firmware's own /etc/init.d/looper/setting/ros2_env.conf is enough for the
# C++ `insight_full` binary but not for anything Python:
#
#   1. PYTHONPATH lists only `.../humble/lib/python3.10/site-packages`, which
#      holds the ament_* build tooling and a partial `tf2_ros`.  Every package
#      a ROS 2 Python node actually needs -- rclpy, cv_bridge, message_filters,
#      tf2_py, and all the generated message modules -- lives in
#      `.../humble/local/lib/python3.10/dist-packages`, which is absent.  Without
#      it `import rclpy` fails and `tf2_ros` imports but explodes on `tf2_py`.
#   2. AMENT_PREFIX_PATH is not set at all, so `ros2` raises
#      `PackageNotFoundError: ros2cli` even once rclpy is importable.
#   3. ROS_DISTRO is `/opt/ros/humble/humble` (a path, and a wrong one) rather
#      than the distro name `humble`.
#
# Fixing those three is what makes the `ros2` CLI work on this board.  We do it
# in a separate file instead of editing ros2_env.conf because that file belongs
# to the firmware and an OTA would overwrite our edit anyway.
#
# Note both /opt/ros/humble and /opt/tros/humble are symlinks to
# /userdata/hobot/opt/ros/humble.

X5_ROOT="${X5_ROOT:-/userdata/x5}"
ROS_ROOT="${ROS_ROOT:-/opt/ros/humble}"

# --- ROS 2 core ---------------------------------------------------------------
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
# Loopback-only DDS. The office WiFi carries foreign ROS 2 discovery traffic on
# domain 0; one malformed ParticipantEntitiesInfo makes FastCDR allocate ~1.5 GB
# and OOM-kill whatever node received it -- including the camera firmware.
# Must match /etc/init.d/looper/setting/ros2_env.conf or the firmware is invisible.
export ROS_LOCALHOST_ONLY=1
export ROS_DISTRO=humble                 # (3) name, not a path
export AMENT_PREFIX_PATH="${ROS_ROOT}"   # (2) required by the ros2 CLI
export ROS_LOG_DIR=/root/.ros/log

# The board's Python lives outside /usr, and its bin/ (pip, tqdm, f2py, ...) is
# on no default PATH -- pip itself warns about this when installing scripts.
PY_BIN=/userdata/hobot/opt/hobot/Python-3.10.12/install/bin
export PATH="${ROS_ROOT}/bin:${PY_BIN}:${PATH}"

# (1) `local/lib/python3.10/dist-packages` FIRST -- it holds rclpy, cv_bridge,
# message_filters, tf2_py and the message modules.  The site-packages entry
# stays for the ament tooling.
export PYTHONPATH="${ROS_ROOT}/local/lib/python3.10/dist-packages:${ROS_ROOT}/lib/python3.10/site-packages:/userdata/install/lib/python3.10/site-packages"

export LD_LIBRARY_PATH="/userdata/install/lib:${ROS_ROOT}/lib/aarch64-linux-gnu:${ROS_ROOT}/lib:/opt/tros/humble/lib/aarch64-linux-gnu:/opt/tros/humble/lib:/app/lib:/app/pub/lib:/middleware/lib:/middleware/pub/lib:/usr/hobot/lib:/usr/hobot/lib/sensor:/system/lib:/system/usr/lib:/lib:/userdata/hobot/opt/hobot/deps:/userdata/install/lib/example/vio_deps"

# --- our cross-built extensions ----------------------------------------------
# Both .so files carry an $ORIGIN RUNPATH, so their bundled libraries resolve
# without help; the LD_LIBRARY_PATH entries below are only a fallback.
for _d in "${X5_ROOT}/pydbow3" "${X5_ROOT}/cpp_bind"; do
    [ -d "${_d}" ] && export PYTHONPATH="${_d}:${PYTHONPATH}"
    # pydbow3 keeps its libraries in lib/; cpp_bind keeps them flat alongside
    # the extension, so add both shapes.
    [ -d "${_d}/lib" ] && export LD_LIBRARY_PATH="${_d}/lib:${LD_LIBRARY_PATH}"
    [ -d "${_d}" ] && export LD_LIBRARY_PATH="${_d}:${LD_LIBRARY_PATH}"
done
unset _d

# --- pure-Python packages shipped from the PC ---------------------------------
# `sensor_msgs_py` (needed by planning_node.py) is a ROS common_interfaces
# package, not a PyPI one, and the board's ROS install omits it.  It is pure
# Python, so it is copied from a PC ROS install rather than pip-installed.
[ -d "${X5_ROOT}/pylibs" ] && export PYTHONPATH="${X5_ROOT}/pylibs:${PYTHONPATH}"

# --- the tinynav checkout -----------------------------------------------------
[ -d "${X5_ROOT}/tinynav" ] && export PYTHONPATH="${X5_ROOT}/tinynav:${PYTHONPATH}"

# --- numba ---------------------------------------------------------------------
# The A55 cores are slow and llvmlite's first-call JIT is the single biggest
# start-up cost on this board, so give numba a persistent on-disk cache.  Only
# functions decorated with cache=True can use it.
export NUMBA_CACHE_DIR="${X5_ROOT}/cache/numba"
mkdir -p "${NUMBA_CACHE_DIR}" 2>/dev/null || true

# OpenBLAS/numpy would otherwise spawn one thread per core and thrash.
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"

# --- which chassis is bolted on ------------------------------------------------
# The board is dedicated to the ESP32 diff-drive car; app_start.sh still defaults to
# `wheel` (the LeKiwi servo bus), which silently draws a round 300 mm robot in the UI
# and hands the planner LeKiwi's kinematics. Set here rather than in app_start.sh
# because it is a property of this board's hardware, not of the code.
export TINYNAV_ACTUATOR="${TINYNAV_ACTUATOR:-diffcar}"

# 光心离地。2026-09-01 实测（深度图拟合地面平面，内点 91%，残差 RMS 5.4mm）：0.126。
# 低矮障碍判据拿它当地面基准，估错超过 low_obs_h_lo(0.05) 就会把整片地板标成障碍。
export TINYNAV_CAMERA_HEIGHT_M="${TINYNAV_CAMERA_HEIGHT_M:-0.124}"
# 深度图取样步长。默认 5 时前方 1~3 m 单帧召回 68~72%，而障碍要连续两帧才越过阈值 ——
# 2026-09-01 的撞击里 esdf_at_robot 一路 0.41→0.30→0.45→0.14 非单调，就是墙上有洞。
# 3 把召回提到约 85%，代价 14->39 ms（1/step^2），周期 0.21->约 0.26 s。
# 2 要 87 ms，而板上 CPU 已经 7/8 满，加密到 2 换来的召回不够抵周期变长带来的延迟。
export TINYNAV_RAYCAST_STEP="${TINYNAV_RAYCAST_STEP:-3}"
# 栅格 z 相位。0.15 时 origin_z 吸附到 -0.100，地面(z=0)正好压在 k=1/k=2 的层边界上 ——
# 同一条椅子腿认不认取决于地面回波落哪一层，由 ±1mm 噪声决定，最小可见高度在 50/100mm 之间跳。
# 0.1625 让 origin_z=-0.0875，地面落在 k=1 正中间（两边各留 12.5mm），最小可见高度稳定 63mm。
# 2026-09-02 板上真录数据实测：5~10cm 波段格 8->27，而空地假障碍四个相位都是 0（无代价）。
export TINYNAV_GRID_OFFSET_Z="${TINYNAV_GRID_OFFSET_Z:-0.1625}"
# 全链延迟统计的窗口秒数，0=关。每个节点每窗口只打一行，量级可忽略。
export TINYNAV_LAT_LOG_S="${TINYNAV_LAT_LOG_S:-10}"
# 3D 点云。板上实测 vis:voxels 10 ms/周期，是单项最大的可视化开销（mask+heightmap+
# footprint 三个合起来才 7 ms），而它只在局部视图切到 3D 时才被画，平时用得少。
# 另外后端那边每条还要付一次反序列化。要看 3D 就设 1。
export TINYNAV_PUBLISH_VOXELS="${TINYNAV_PUBLISH_VOXELS:-0}"
# 逐候选代价那条日志实测 11~18ms/周期，而延迟数已经挪到独立的 LAT 行，
# 所以这里限速不再影响延迟分析。要看选轨迹的细节就设 0。
export TINYNAV_DECISION_LOG_HZ="${TINYNAV_DECISION_LOG_HZ:-2}"
# 判决图的代价。板上逐段实测 stride=2 共 64 ms（classify 34.9 + tint 17.8 + 投影 9.4），
# stride=4 降到 24 ms。它 2 Hz 渲染而规划 5 Hz，所以 stride=2/hz=2 等于每周期平均
# 摊 26 ms —— 和 raycasting 一个量级，全花在给人看的图上。
# 导航档：stride=4 + hz=1（平均 5 ms）。停车调障碍参数时改回 2 / 2 看细节。
export TINYNAV_HEIGHT_COLOR_STRIDE="${TINYNAV_HEIGHT_COLOR_STRIDE:-4}"
export TINYNAV_HEIGHT_COLOR_HZ="${TINYNAV_HEIGHT_COLOR_HZ:-1.0}"
