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
