#!/bin/bash
set -uo pipefail

# Record a mapping bag on the X5 inside the Looper camera.
#
#     bash tool/x5_board/map_record.sh start [--name NAME] [--no-wheel]
#     bash tool/x5_board/map_record.sh stop
#     bash tool/x5_board/map_record.sh status
#
# WHY A SCRIPT AND NOT tmux
#   The board has neither tmux nor screen, so the repo's scripts/run_*.sh cannot
#   run here at all. This starts the processes detached with nohup and tees each
#   one to its own log, which is the same thing minus the panes.
#
# WHAT IT STARTS
#   wheel_odometry_node   owns the Feetech bus: reads the encoders, publishes
#                         /wheel/camera_pose into the bag, AND drives the wheels
#                         from /cmd_vel. It is the only thing that may hold the
#                         serial port -- lekiwi_control must not run.
#   ros2 bag record       the topic set in scripts/run_rosbag_record.sh
#
#   The teleop is NOT started here: it needs a terminal. Run it yourself in a
#   second ssh session:
#       . /userdata/x5/env.sh
#       python3 /userdata/x5/tinynav/tool/x5_board/wheel_teleop.py
#
# WHY THE WHEEL NODE MUST RUN EVEN FOR A VIO-ONLY MAP
#   Two reasons. It is the actuator, so without it nothing drives the robot and
#   the run has to be hand-pushed, which slips ~11% on an omni base. And it puts
#   /wheel/camera_pose in the bag, which is what lets the same recording build the
#   odometry map later without a second drive.

X5_ROOT="${X5_ROOT:-/userdata/x5}"
TINYNAV="${TINYNAV:-${X5_ROOT}/tinynav}"
BAG_ROOT="${BAG_ROOT:-${X5_ROOT}/bags}"
LOG_DIR="${LOG_DIR:-${X5_ROOT}/logs}"
PID_DIR="${PID_DIR:-${X5_ROOT}/run}"

WHEEL_PORT="${WHEEL_PORT:-/dev/ttyS3}"
WHEEL_RADIUS="${WHEEL_RADIUS:-0.050385}"
BASE_RADIUS="${BASE_RADIUS:-0.127083}"
CAMERA_OFFSET="${CAMERA_OFFSET:-0.06,0.05,0.18}"

# Minimum free space to start a recording. The Looper set is about 12 MB/s
# (infra1 mono8 at 20 Hz is 7, depth mono16 at 5 Hz is 3.5, compressed colour the
# rest), so 4 GB is roughly 5 minutes with margin. Running out mid-drive loses the
# whole run, and there is no way to tell from inside the robot.
MIN_FREE_MB="${MIN_FREE_MB:-4096}"

mkdir -p "${BAG_ROOT}" "${LOG_DIR}" "${PID_DIR}"

pidfile() { echo "${PID_DIR}/$1.pid"; }

alive() {
    local pf; pf="$(pidfile "$1")"
    [[ -f "${pf}" ]] && kill -0 "$(cat "${pf}")" 2>/dev/null
}

start_proc() {
    local name="$1"; shift
    if alive "${name}"; then
        echo "  ${name}: already running (pid $(cat "$(pidfile "${name}")"))"
        return 0
    fi
    local log="${LOG_DIR}/${name}.log"
    # setsid so the process leads its own group and stop_proc can signal the whole
    # tree. A wrapper script that spawns the real worker as a child would otherwise
    # swallow the signal: the first version of this script "stopped" a recording
    # that then kept writing past 1 GB and never flushed its metadata.
    setsid nohup "$@" > "${log}" 2>&1 &
    echo $! > "$(pidfile "${name}")"
    echo "  ${name}: pid $! -> ${log}"
}

stop_proc() {
    local name="$1" sig="${2:-TERM}"
    local pf; pf="$(pidfile "${name}")"
    if ! alive "${name}"; then
        echo "  ${name}: not running"
        rm -f "${pf}"
        return 0
    fi
    local pid; pid="$(cat "${pf}")"
    # Negative pid = the whole process group, which setsid made equal to the pid.
    kill "-${sig}" "-${pid}" 2>/dev/null || kill "-${sig}" "${pid}" 2>/dev/null
    for _ in $(seq 1 60); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.25
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "  ${name}: did not exit on SIG${sig}, sending KILL"
        kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null
    fi
    rm -f "${pf}"
    echo "  ${name}: stopped"
}

cmd_start() {
    local name="map_$(date +%Y%m%d_%H%M%S)" want_wheel=1
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --name) name="$2"; shift 2 ;;
            --no-wheel) want_wheel=0; shift ;;
            *) echo "Unknown option '$1'" >&2; exit 1 ;;
        esac
    done

    local free_mb
    free_mb="$(df -Pm "${BAG_ROOT}" | awk 'NR==2 {print $4}')"
    if [[ "${free_mb}" -lt "${MIN_FREE_MB}" ]]; then
        echo "refusing to start: only ${free_mb} MB free on ${BAG_ROOT}, want ${MIN_FREE_MB}" >&2
        exit 1
    fi

    if pgrep -f 'lekiwi_control.py' >/dev/null 2>&1; then
        echo "refusing to start: lekiwi_control.py is running and owns the serial bus" >&2
        exit 1
    fi
    if ! pgrep -x insight_full >/dev/null 2>&1; then
        echo "refusing to start: insight_full is not running, so there is no camera data" >&2
        exit 1
    fi

    local bag="${BAG_ROOT}/${name}"
    if [[ -e "${bag}" ]]; then
        echo "refusing to start: ${bag} already exists" >&2
        exit 1
    fi

    echo "recording to ${bag} (${free_mb} MB free)"
    if [[ ${want_wheel} -eq 1 ]]; then
        start_proc wheel_odometry \
            python3 "${TINYNAV}/tinynav/core/wheel_odometry_node.py" --ros-args \
                -p "port:=${WHEEL_PORT}" \
                -p "wheel_radius:=${WHEEL_RADIUS}" \
                -p "base_radius:=${BASE_RADIUS}" \
                -p "camera_offset_xyz:=[${CAMERA_OFFSET}]" \
                -p enable_wheel_command:=true \
                -p cmd_vel_topic:=/cmd_vel
        # Give it time to reach the EEPROM read-back before the recorder latches
        # onto a topic list, so /wheel/camera_pose is present from the first frame.
        sleep 3
        if ! alive wheel_odometry; then
            echo "wheel_odometry died on startup, see ${LOG_DIR}/wheel_odometry.log" >&2
            tail -20 "${LOG_DIR}/wheel_odometry.log" >&2
            exit 1
        fi
    else
        echo "  wheel_odometry: skipped (--no-wheel), the bag will have no /wheel/* topics"
    fi

    start_proc bag_record bash "${TINYNAV}/scripts/run_rosbag_record.sh" \
        --output "${bag}" --sensor looper
    echo "${bag}" > "${PID_DIR}/current_bag"
    sleep 3
    if ! alive bag_record; then
        echo "bag_record died on startup, see ${LOG_DIR}/bag_record.log" >&2
        tail -20 "${LOG_DIR}/bag_record.log" >&2
        exit 1
    fi
    echo
    echo "now drive it. In another ssh session:"
    echo "  . ${X5_ROOT}/env.sh && python3 ${TINYNAV}/tool/x5_board/wheel_teleop.py"
    echo "then: bash $0 stop"
}

cmd_stop() {
    # The recorder first, and with SIGINT, which is what ros2 bag treats as a
    # clean finish -- SIGTERM can leave the sqlite file without its final flush.
    stop_proc bag_record INT
    stop_proc wheel_odometry TERM

    local bag=""
    [[ -f "${PID_DIR}/current_bag" ]] && bag="$(cat "${PID_DIR}/current_bag")"
    if [[ -z "${bag}" || ! -d "${bag}" ]]; then
        echo "no current bag recorded"
        return 0
    fi

    # metadata.yaml is written only on a clean shutdown, and it is what makes the
    # directory a bag rather than a loose .db3. Wait for it rather than assuming,
    # because a bag without it is unreadable and the failure is easy to miss until
    # the map build reports zero keyframes.
    local ok=0
    for _ in $(seq 1 40); do
        [[ -f "${bag}/metadata.yaml" ]] && { ok=1; break; }
        sleep 0.25
    done
    echo
    echo "bag: ${bag}  ($(du -sh "${bag}" | cut -f1))"
    if [[ ${ok} -eq 0 ]]; then
        echo "  !! metadata.yaml missing -- the recorder did not shut down cleanly." >&2
        echo "     This bag is unreadable. Check ${LOG_DIR}/bag_record.log." >&2
        return 1
    fi
    echo "verifying (the pose topics are what decide which maps this bag can build):"
    ros2 bag info "${bag}" 2>&1 | sed 's/^/  /'

    local have_vio have_wheel
    have_vio="$(ros2 bag info "${bag}" 2>/dev/null | grep -c 'vio_image')"
    have_wheel="$(ros2 bag info "${bag}" 2>/dev/null | grep -c 'wheel/camera_pose')"
    echo
    [[ "${have_vio}" -gt 0 ]] \
        && echo "  can build the VIO map      (/camera/camera/vio_image present)" \
        || echo "  !! NO VIO pose: cannot build the VIO map"
    [[ "${have_wheel}" -gt 0 ]] \
        && echo "  can build the odometry map (/wheel/camera_pose present)" \
        || echo "  !! NO wheel pose: cannot build the odometry map (was wheel_odometry running?)"
}

cmd_status() {
    for n in wheel_odometry bag_record; do
        if alive "${n}"; then
            echo "  ${n}: running (pid $(cat "$(pidfile "${n}")"))"
        else
            echo "  ${n}: stopped"
        fi
    done
    [[ -f "${PID_DIR}/current_bag" ]] && {
        local bag; bag="$(cat "${PID_DIR}/current_bag")"
        [[ -d "${bag}" ]] && echo "  bag: ${bag} ($(du -sh "${bag}" | cut -f1))"
    }
    df -h "${BAG_ROOT}" | tail -1 | sed 's/^/  disk: /'
}

case "${1:-}" in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *) echo "Usage: $0 {start [--name NAME] [--no-wheel] | stop | status}" >&2; exit 1 ;;
esac
