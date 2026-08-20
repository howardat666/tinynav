#!/bin/bash
set -uo pipefail

# Run the whole TinyNav app -- backend, web UI and every ROS node -- on the X5
# inside the Looper camera. Runs ON THE BOARD.
#
#     bash tool/x5_board/app_start.sh start a     # VIO map,      VIO nav
#     bash tool/x5_board/app_start.sh start b     # VIO map,      odometry nav
#     bash tool/x5_board/app_start.sh start c     # odometry map, odometry nav
#     bash tool/x5_board/app_start.sh status
#     bash tool/x5_board/app_start.sh log
#     bash tool/x5_board/app_start.sh stop
#
# Then, from the laptop's browser:  http://<board-ip>:8000/ (printed by `status`)
#
# WHY A SCHEME LETTER AND NOT A CONFIG FILE
#   The three schemes are the comparison. Each one is a fixed combination of
#   TINYNAV_MAP_ODOM_SOURCE and TINYNAV_ODOM_SOURCE, and they must be set before
#   the first node starts because they decide which processes exist at all --
#   node_manager reads them at import. A letter on the command line makes the
#   run's configuration visible in the log and impossible to half-change: there is
#   no state to forget to reset between runs.
#
# WHY ROBOT_TYPE AND ACTUATOR ARE PINNED, NOT DERIVED
#   All three schemes drive the same LeKiwi chassis; only the odometry differs.
#   The defaults in node_manager are go2/unitree, so leaving them alone starts
#   unitree_control against a chassis that has no Unitree in it -- observed, and
#   it starts happily and silently does nothing useful. Pin them here.
#
# WHY IT SERVES THE UI ITSELF
#   The X5 is the only computer in this system; the laptop is a browser. uvicorn
#   mounts app/frontend/build/web at '/', so page and API share an origin and
#   there is nothing to configure in the UI. If the bundle is missing the backend
#   still starts, API-only, and says so in the log -- see sync_to_board.sh
#   --with-web for how it gets there.

BOARD_ROOT="${BOARD_ROOT:-/userdata/x5/tinynav}"
LOG_DIR="${LOG_DIR:-/userdata/x5/logs}"
ENV_SH="${ENV_SH:-/userdata/x5/env.sh}"
DB_PATH="${TINYNAV_DB_PATH:-/userdata/x5/tinynav_db}"
PORT="${PORT:-8000}"
PIDFILE="${LOG_DIR}/app.pid"
LOGFILE="${LOG_DIR}/app.log"
SCHEMEFILE="${LOG_DIR}/app.scheme"

usage() { sed -n '3,20p' "$0" >&2; exit 1; }

# Sourcing env.sh is not optional: without it rosbag2_py cannot find
# libtinyxml2.so.9, which lives in /userdata/hobot/opt/hobot/deps. The failure
# surfaces as an import error deep inside a map build, long after startup.
load_env() {
    if [[ ! -f "${ENV_SH}" ]]; then
        echo "missing ${ENV_SH} -- ROS and its deps will not resolve" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    . "${ENV_SH}"
}

app_pid() {
    [[ -f "${PIDFILE}" ]] || return 1
    local pid; pid="$(cat "${PIDFILE}" 2>/dev/null)"
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && { echo "${pid}"; return 0; }
    return 1
}

do_start() {
    local scheme="${1:-}"
    case "${scheme}" in
        a) map_src=vio;   nav_src=vio   ;;
        b) map_src=vio;   nav_src=wheel ;;
        c) map_src=wheel; nav_src=wheel ;;
        *) echo "scheme must be a, b or c (got '${scheme}')" >&2; usage ;;
    esac

    if pid="$(app_pid)"; then
        echo "already running as pid ${pid} (scheme $(cat "${SCHEMEFILE}" 2>/dev/null || echo '?'))" >&2
        echo "stop it first: bash $0 stop" >&2
        exit 1
    fi

    # A stale insight_full means no camera topics at all, which the app reports as
    # an empty sensor list rather than as an error. Cheaper to check here.
    if ! pgrep -x insight_full >/dev/null 2>&1; then
        echo "WARNING: insight_full is not running -- no camera topics will appear" >&2
    fi

    mkdir -p "${LOG_DIR}" "${DB_PATH}"
    load_env

    export TINYNAV_DB_PATH="${DB_PATH}"
    # node_manager's default log directory is /userdata/junlinp/logs -- a path from
    # someone else's machine that happens to be writable here, which is worse than
    # a broken one because the logs go somewhere nobody thinks to look.
    export TINYNAV_LOG_DIR="${TINYNAV_LOG_DIR:-${LOG_DIR}/nodes}"
    mkdir -p "${TINYNAV_LOG_DIR}"
    # Pinned so they never fall back to node_manager's go2/unitree defaults, but
    # overridable: this chassis is not always LeKiwi. Getting the actuator wrong is
    # not merely useless -- with the diff-drive car, wheel_odometry writes Feetech
    # packets into the ESP32's single-character command parser (0x66 is 'f',
    # forward), so a wrong actuator can drive the robot. Observed 2026-08-18.
    export TINYNAV_ROBOT_TYPE="${TINYNAV_ROBOT_TYPE:-lekiwi}"
    export TINYNAV_ACTUATOR="${TINYNAV_ACTUATOR:-wheel}"
    # Pinned for the same reason as the two above. Auto-detection greps `ros2 node
    # list` for /insight_full, and a successful call proves nothing about
    # completeness: DDS discovery is asynchronous and this board's ros2 daemon has
    # been seen returning an empty view while the firmware published 15 topics.
    # A mis-detect is not a degraded mode -- it starts the realsense driver and
    # perception against a Looper, so no looper_bridge runs, no /slam keyframes
    # exist, and navigation relocalizes against nothing with no error anywhere.
    export TINYNAV_SENSOR_MODE="${TINYNAV_SENSOR_MODE:-looper}"
    export TINYNAV_ODOM_SOURCE="${nav_src}"
    export TINYNAV_MAP_ODOM_SOURCE="${map_src}"

    # Colour preview alone wants 3.6 Mbit/s at 5 fps and the board's WiFi transmit
    # path caps near 2.5 (docs/x5/board_bringup.md 2.5). Drops it from the UI's topic
    # list entirely, so nothing subscribes to it either.
    export TINYNAV_DISABLE_COLOR="${TINYNAV_DISABLE_COLOR:-1}"

    # Offline map build limits. Without all four the build is OOM-killed on this
    # board: measured rc=137 at 0.5% progress, 645 MiB resident.
    #   vocabulary  ORBvoc.txt needs 1735 MB, more than the board has, and is not
    #               even shipped here -- so the failure looks like a missing file.
    #               The office k10L5 vocabulary peaks at 407 MB.
    #   play-rate   unpaced playback fills the sync queue faster than the board
    #               drains it. 1.0 = real time, which the board keeps up with.
    #   sync-queue  200 slots x 3 full-res images ~ 280 MB. 20 is 28 MB.
    #   no-vis      the rviz-only publishes cost 12.6 s of the 13.4 s per keyframe
    #               and nothing in the saved map depends on them.
    export TINYNAV_DBOW3_VOCAB="${TINYNAV_DBOW3_VOCAB:-/userdata/x5/voc/voc_office_k10L5.dbow3}"
    export TINYNAV_MAP_PLAY_RATE="${TINYNAV_MAP_PLAY_RATE:-1.0}"
    export TINYNAV_MAP_SYNC_QUEUE="${TINYNAV_MAP_SYNC_QUEUE:-20}"
    export TINYNAV_MAP_VISUALIZATION="${TINYNAV_MAP_VISUALIZATION:-0}"
    # Both now default to these values in node_manager, so these lines only pin them
    # against a future default change. Kept for the same reason the four above are
    # pinned: this file is where a run's configuration is meant to be readable.
    #
    #   save-videos   the rgb and infra1 h264 encodes are 95% of save_image_and_depth --
    #                 measured 95.3 s and 18.5 s over a 575-keyframe build against 5.9 s
    #                 for the depth write. Nothing on the nav path reads either; the
    #                 consumers are convert_to_nerf_format and poi_editor, both PC-side.
    #                 Set to 1 for a map that is going to be exported for 3DGS.
    #   db-sync-every dbm.dumb rewrites the whole .dir index on every sync(), so syncing
    #                 all three shelves per keyframe is ~1700 rename+unlink pairs on
    #                 eMMC. 50 risks losing up to 50 keyframes if the build is killed.
    export TINYNAV_MAP_SAVE_VIDEOS="${TINYNAV_MAP_SAVE_VIDEOS:-0}"
    export TINYNAV_DB_SYNC_EVERY="${TINYNAV_DB_SYNC_EVERY:-50}"
    # The app's local-view layers (obstacle mask + ESDF heatmap + footprint). ON: pinning
    # these off is what made the web UI's middle panel blank -- and it also nulls
    # grid_info, the transform every other layer is drawn through, because node_manager
    # derives it from the mask's OccupancyGrid metadata. Set to 0 only to buy CPU back.
    export TINYNAV_PUBLISH_PLANNING_OVERLAYS="${TINYNAV_PUBLISH_PLANNING_OVERLAYS:-1}"
    export TINYNAV_VERBOSE_TIMER="${TINYNAV_VERBOSE_TIMER:-0}"

    if [[ ! -f "${TINYNAV_DBOW3_VOCAB}" ]]; then
        echo "vocabulary missing: ${TINYNAV_DBOW3_VOCAB} -- map build will fail" >&2
        exit 1
    fi

    {
        echo "=============================================================="
        echo "scheme        : ${scheme}   (map=${map_src}, nav=${nav_src})"
        echo "board uptime  : $(cut -d' ' -f1 /proc/uptime)s"
        echo "robot/actuator: ${TINYNAV_ROBOT_TYPE} / ${TINYNAV_ACTUATOR}"
        echo "db            : ${TINYNAV_DB_PATH}"
        echo "node logs     : ${TINYNAV_LOG_DIR}"
        echo "map vocab     : ${TINYNAV_DBOW3_VOCAB}"
        echo "map build     : rate=${TINYNAV_MAP_PLAY_RATE} queue=${TINYNAV_MAP_SYNC_QUEUE} vis=${TINYNAV_MAP_VISUALIZATION} videos=${TINYNAV_MAP_SAVE_VIDEOS} db_sync=${TINYNAV_DB_SYNC_EVERY}"
        echo "diagnostics   : planning_overlays=${TINYNAV_PUBLISH_PLANNING_OVERLAYS} verbose_timer=${TINYNAV_VERBOSE_TIMER}"
        echo "=============================================================="
    } >> "${LOGFILE}"

    # setsid so the whole thing survives this ssh session closing, and so that
    # stop can signal the process group -- the backend spawns ROS nodes as
    # children and signalling the pid alone orphans them.
    cd "${BOARD_ROOT}" || exit 1
    setsid nohup python3 -m uvicorn app.backend.main:app \
        --host 0.0.0.0 --port "${PORT}" \
        >> "${LOGFILE}" 2>&1 < /dev/null &
    local pid=$!
    echo "${pid}" > "${PIDFILE}"
    echo "${scheme} (map=${map_src}, nav=${nav_src})" > "${SCHEMEFILE}"

    # Startup is ~10 s on this board: rclpy init, then the sensor bridge and
    # planning node. Report what actually came up rather than just the pid.
    sleep 12
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "FAILED to start -- last 30 lines of ${LOGFILE}:" >&2
        tail -30 "${LOGFILE}" >&2
        rm -f "${PIDFILE}"
        exit 1
    fi
    echo "started pid ${pid}, scheme ${scheme} (map=${map_src}, nav=${nav_src})"
    echo
    do_status
}

do_stop() {
    if ! pid="$(app_pid)"; then
        echo "not running"
        rm -f "${PIDFILE}"
        return 0
    fi
    # Negative pid = the process group setsid created, so the ROS children die too.
    kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null
    for _ in $(seq 20); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "did not exit on TERM, sending KILL" >&2
        kill -KILL "-${pid}" 2>/dev/null
        sleep 1
    fi
    rm -f "${PIDFILE}"

    # The process-group kill above does NOT reach the ROS nodes, and this is the whole
    # reason they leak. node_manager._launch_proc spawns every child with
    # preexec_fn=os.setsid, which makes each one a session leader in its own new process
    # group -- so `kill -TERM -<backend_pid>` signals the backend and nothing else.
    # Measured consequence: a cmd_vel_control survived 32 minutes past its backend, at
    # ppid 1, still publishing to /cmd_vel alongside its replacement.
    #
    # Sweeping by name is the honest fix here rather than removing the setsid: the nodes
    # want their own groups so that killing one does not take down its siblings. Names
    # use the [x] trick so the pattern cannot match this script's own pgrep/pkill.
    local leftovers=0
    for pat in 'map_[n]ode\.py' 'planning_[n]ode\.py' 'cmd_vel_[c]ontrol\.py' 'looper_[b]ridge_node\.py'; do
        if pgrep -f "${pat}" >/dev/null 2>&1; then
            pkill -TERM -f "${pat}" 2>/dev/null
            leftovers=$((leftovers + 1))
        fi
    done
    if [[ ${leftovers} -gt 0 ]]; then
        sleep 2
        for pat in 'map_[n]ode\.py' 'planning_[n]ode\.py' 'cmd_vel_[c]ontrol\.py' 'looper_[b]ridge_node\.py'; do
            pgrep -f "${pat}" >/dev/null 2>&1 && pkill -KILL -f "${pat}" 2>/dev/null
        done
        echo "swept ${leftovers} orphaned node group(s)"
    fi

    echo "stopped"
    # wheel_odometry_node is deliberately long-lived and is not in the backend's
    # own shutdown path, so say whether anything is still holding the servo bus.
    # Left out of the sweep above for the same reason.
    pgrep -af 'wheel_odometry_[n]ode' && echo "  ^ wheel odometry still up (expected: it owns the servo bus)"
    return 0
}

do_status() {
    if pid="$(app_pid)"; then
        echo "backend  : running pid ${pid}"
        echo "scheme   : $(cat "${SCHEMEFILE}" 2>/dev/null || echo '?')"
    else
        echo "backend  : not running"
    fi
    # netstat, not ss: this board's busybox userland has no ss at all, and
    # `ss ... | grep -c` on an empty stream cheerfully reports 0 listeners.
    echo "port     : $(netstat -tln 2>/dev/null | grep -c ":${PORT}[[:space:]]") listener(s) on ${PORT}"
    # Derived, not hardcoded: the USB address died when the C port became USB host for
    # the WiFi dongle, and printing it sends you to a page that never loads.
    ip=$(route -n | awk '$1=="0.0.0.0"{print $8; exit}')
    ip=$(ifconfig "${ip:-lo}" 2>/dev/null | awk '/inet /{sub("addr:","",$2); print $2; exit}')
    echo "web UI   : http://${ip:-<board-ip>}:${PORT}/"
    echo "nodes    :"
    pgrep -af 'looper_bridge_node|planning_node|cmd_vel_control|wheel_odometry_node|map_node|unitree' \
        | sed 's/^/  /' || echo "  (none)"
    echo "resources:"
    printf '  cpu %s mC   ddr %s mC   load %s   avail %s kB\n' \
        "$(cat /sys/class/thermal/thermal_zone1/temp 2>/dev/null)" \
        "$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)" \
        "$(cut -d' ' -f1 /proc/loadavg)" \
        "$(awk '/MemAvailable/{print $2}' /proc/meminfo)"
}

case "${1:-}" in
    start)  shift; do_start "${1:-}" ;;
    stop)   do_stop ;;
    status) do_status ;;
    log)    tail -n "${2:-60}" -f "${LOGFILE}" ;;
    *)      usage ;;
esac
