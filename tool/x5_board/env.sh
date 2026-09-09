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
# 2026-09-01 撞击里 esdf_at_robot 一路 0.41→0.30→0.45→0.14 非单调，就是墙上有洞。
# 3 把召回提到约 85%，代价 14->39 ms（1/step^2），周期 0.21->约 0.26 s。
export TINYNAV_RAYCAST_STEP="${TINYNAV_RAYCAST_STEP:-3}"
# 栅格 z 相位。0.15 时 origin_z 吸附到 -0.100，地面(z=0)正好压在 k=1/k=2 的层边界上 ——
# 同一条椅子腿认不认取决于地面回波落哪一层，由 +-1mm 噪声决定，最小可见高度在 50/100mm 之间跳。
# 0.1625 让 origin_z=-0.0875，地面落在 k=1 正中间（两边各留 12.5mm），最小可见高度稳定 63mm。
# 2026-09-02 板上真录数据实测：5~10cm 波段格 8->27，而空地假障碍四个相位都是 0（无代价）。
export TINYNAV_GRID_OFFSET_Z="${TINYNAV_GRID_OFFSET_Z:-0.1625}"
# 全链延迟统计的窗口秒数，0=关。每个节点每窗口只打一行，量级可忽略。
export TINYNAV_LAT_LOG_S="${TINYNAV_LAT_LOG_S:-10}"
# 3D 点云。板上实测 vis:voxels 10 ms/周期，是单项最大的可视化开销（mask+heightmap+
# footprint 合起来才 7 ms），而它只在局部视图切到 3D 时才被画。要看 3D 就设 1。
export TINYNAV_PUBLISH_VOXELS="${TINYNAV_PUBLISH_VOXELS:-0}"
# 逐候选代价那条日志实测 11~18ms/周期，而延迟数已挪到独立的 LAT 行，
# 所以这里限速不再影响延迟分析。要看选轨迹的细节就设 0。
export TINYNAV_DECISION_LOG_HZ="${TINYNAV_DECISION_LOG_HZ:-2}"
# 判决图的代价。板上逐段实测 stride=2 共 64 ms（classify 34.9 + tint 17.8 + 投影 9.4），
# stride=4 降到 24 ms。它 2 Hz 渲染而规划 5 Hz，所以 stride=2/hz=2 等于每周期平均
# 摊 26 ms —— 和 raycasting 一个量级，全花在给人看的图上。
# 导航档：stride=4 + hz=1（平均 5 ms）。停车调障碍参数时改回 2 / 2 看细节。
export TINYNAV_HEIGHT_COLOR_STRIDE="${TINYNAV_HEIGHT_COLOR_STRIDE:-4}"
export TINYNAV_HEIGHT_COLOR_HZ="${TINYNAV_HEIGHT_COLOR_HZ:-1.0}"
# 矮障碍层默认关（2026-09-02）：0.05 m 栅格下 z 跨度单独就够 —— 仿真里 5 cm x 0.35 m
# 的椅腿开/关都是 3 个障碍格，它贡献 0；而板上它贡献 185/243 格全是噪声。
# 🔴 原来这里是写死的 `=1`，没有 :- 默认，于是 app_start.sh 和代码里的默认值全都
# 覆盖不了它（这份 env.sh 在 app_start.sh 之前 source）。要 A/B 就 TINYNAV_LOW_OBS=1 起服务。
export TINYNAV_LOW_OBS="${TINYNAV_LOW_OBS:-0}"
export TINYNAV_LOW_OBS_MIN_PTS="${TINYNAV_LOW_OBS_MIN_PTS:-2}"

# 🔴 值和历史注释一度矛盾（注释写"退回 0.25"，值是 0.40），2026-09-09 核对如下。
# 09-02 二次复盘定的 0.25，用的是当时约 1.0 s 的环路延迟。09-07 实测延迟降到 0.685 s，
# 于是按刹车距离 v*T + v^2/(2a) 重算（a=3.0，障碍首次探测中位 0.28 m）：
#   T=0.49s（障碍恰被当帧采到）  0.40 -> 0.223 m  余量 5.7 cm
#   T=0.59s（等半个深度帧）      0.40 -> 0.263 m  余量 1.7 cm
#   T=0.69s（等满一帧，最坏）    0.40 -> 0.303 m  超支 2.3 cm   <- 0.40 的软肋
#   同样最坏情况下 0.333 -> 0.248 m 余量 3.2 cm
# 保留 0.40 是明确决定（2026-09-09），不是遗漏；要收就收到下一档 0.333。
export TINYNAV_MAX_VX="${TINYNAV_MAX_VX:-0.40}"
# max_yaw 0.6 -> 1.0（2026-09-09）。0.6 是按「max_yaw x 环路延迟 = 来不及改的转角」定的，
# 当时环路延迟 1.04 s ⇒ 0.6 给 36°、1.05 给 63°，63 被否。09-07 实测延迟降到 0.685 s
# （相机 0.149 + 决策 0.102 + 重规划 0.20 + 执行器 0.234）⇒ 1.0 只承诺 39°，和当初接受的
# 36° 同量级。买到的是转弯半径：vx=0.40 下 0.667 -> 0.444 m，vx=0.267 下 0.445 -> 0.297 m。
# ⚠️ 它不影响刹车距离（那只跟 vx 有关），但会放大左右翻转的幅度 —— 摆头是另一件事，
# 嫌疑是 smoothness 里 omega 只罚 10（翻一次 3.4 分 ≈ 3.4 cm 路程，等于免费）。
export TINYNAV_MAX_YAW="${TINYNAV_MAX_YAW:-1.0}"
# 楔住后车体自己那格就是障碍格 -> 110 条轨迹被拒 96 条 -> 关着倒车就一个动作都没有，
# 只能人去搬（09-02 一趟里两次）。控制器不会倒车那个 bug 已在 08-27 修掉，
# 而 0.30 m 的净位移上限退回去的正是车 1~2 秒前刚走过的空间。
export TINYNAV_ALLOW_REVERSE="${TINYNAV_ALLOW_REVERSE:-1}"

# 取用策略：回调只存最新、20 Hz 定时器取用，绕开 message_filters 的 FIFO。
# 实测 in_age（进规划回调时的数据龄）死死顶在 max_input_age_s=0.5 上：
# p50 0.39 / p90 0.48 / max 0.50 s，而 /slam/depth 到手时只有 0.163 s
# —— 中间 0.23 s 是纯排队。判据就看 in_age 有没有掉到 ~0.19 s。
# 要退回旧行为：改成 0（队列深度别动，30 改 3 那次把车弄停了）。
export TINYNAV_PLAN_LATEST_ONLY="${TINYNAV_PLAN_LATEST_ONLY:-1}"
