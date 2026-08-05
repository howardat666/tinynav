#!/bin/bash
set -uo pipefail

# Find out whether a sustained full-load run kills the X5 on its own.
#
#     bash tool/x5_board/thermal_load_test.sh run [--bag DIR] [--label TEXT]
#     bash tool/x5_board/thermal_load_test.sh report
#     bash tool/x5_board/thermal_load_test.sh stop
#
# WHY
#   The camera dropped off USB after ~12 minutes of a full-load map build
#   (build_map_node + looper_bridge_node + insight_full, 4+ cores saturated), a
#   clean disconnect with no USB error, and did not come back on its own. The same
#   wiring had been in use for days of light work without trouble, so the variable
#   is load, not the 8-pin harness -- but the 8-pin's wires are soldered, so it can
#   only be connected or disconnected as a whole, and that is the one experiment
#   that separates the two:
#
#       8-pin OUT + full load, dies      -> the harness is exonerated, it is the board
#       8-pin OUT + full load, survives  -> the harness matters under load
#
#   Idle CPU already reads ~73 C against a 110 C critical trip and a 95 C passive
#   one, and the thermal behaviour under sustained load was never measured, so a
#   thermal cutout is the leading candidate. That is what the sampler is for.
#
# WHY THE SAMPLER WRITES TO /userdata
#   The board's journal is volatile: /var/log/journal does not exist, so
#   `journalctl --list-boots` shows only the current boot and the evidence from the
#   previous one is gone. That is exactly what happened the first time, and it is
#   why this appends to a file on /userdata, which survives the power cycle.

X5_ROOT="${X5_ROOT:-/userdata/x5}"
TINYNAV="${TINYNAV:-${X5_ROOT}/tinynav}"
LOG_DIR="${LOG_DIR:-${X5_ROOT}/logs}"
PID_DIR="${PID_DIR:-${X5_ROOT}/run}"
HEALTH="${LOG_DIR}/health.tsv"
VOC="${VOC:-${X5_ROOT}/voc/voc_office_k10L5.dbow3}"
SAMPLE_S="${SAMPLE_S:-2}"

mkdir -p "${LOG_DIR}" "${PID_DIR}"

# Absolute, because the sampler is re-launched as `bash "$SELF" _sampler` through
# setsid and a relative $0 would break if this were invoked as ./thermal_load_test.sh
# from anywhere other than the directory it lives in.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# ---------------------------------------------------------------------------- #
# sampler
# ---------------------------------------------------------------------------- #

sample_once() {
    local up cpu ddr freq load avail
    up=$(awk '{printf "%.0f", $1}' /proc/uptime)
    cpu=$(cat /sys/class/thermal/thermal_zone1/temp 2>/dev/null || echo -1)
    ddr=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo -1)
    freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo -1)
    load=$(awk '{print $1}' /proc/loadavg)
    avail=$(awk '/MemAvailable/{print $2}' /proc/meminfo)
    # Board wall time is meaningless across boots -- there is no battery-backed
    # RTC, so every boot restarts from the same base epoch. `up` is the honest
    # clock here, and a row where it goes backwards marks a reboot.
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$(date +%H:%M:%S)" "${up}" "${cpu}" "${ddr}" "${freq}" "${load}" "${avail}"
}

sampler_loop() {
    local label="$1"
    printf '# ==== sampler start  label=%s  uptime=%ss ====\n' \
        "${label}" "$(awk '{printf "%.0f", $1}' /proc/uptime)" >> "${HEALTH}"
    printf '# time\tuptime_s\tcpu_mC\tddr_mC\tcpu_kHz\tload1\tmemavail_kB\n' >> "${HEALTH}"
    while true; do
        sample_once >> "${HEALTH}"
        sleep "${SAMPLE_S}"
    done
}

# ---------------------------------------------------------------------------- #
# subcommands
# ---------------------------------------------------------------------------- #

cmd_run() {
    local bag="" label="8pin-out-full-load"
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --bag) bag="$2"; shift 2 ;;
            --label) label="$2"; shift 2 ;;
            *) echo "Unknown option '$1'" >&2; exit 1 ;;
        esac
    done

    if [[ -z "${bag}" ]]; then
        bag="$(ls -d "${X5_ROOT}"/bags/*/ 2>/dev/null | head -1)"
        [[ -z "${bag}" ]] && { echo "no bag found under ${X5_ROOT}/bags" >&2; exit 1; }
        bag="${bag%/}"
    fi
    local db3
    db3="$(ls "${bag}"/*.db3 2>/dev/null | head -1)"
    [[ -z "${db3}" ]] && { echo "no .db3 inside ${bag}" >&2; exit 1; }
    [[ -f "${VOC}" ]] || { echo "vocabulary missing: ${VOC}" >&2; exit 1; }

    echo "label      : ${label}"
    echo "bag        : ${db3}"
    echo "health log : ${HEALTH}"
    echo "baseline   : cpu $(( $(cat /sys/class/thermal/thermal_zone1/temp) / 1000 )) C, uptime $(awk '{printf "%.0f", $1}' /proc/uptime) s"
    echo

    # Sampler first, so the pre-load baseline is on disk before anything heats up.
    setsid nohup bash "${SELF}" _sampler "${label}" >/dev/null 2>&1 &
    echo $! > "${PID_DIR}/health.pid"
    sleep 4

    setsid nohup python3 "${TINYNAV}/tool/looper_bridge_node.py" \
        --pose-topic /camera/camera/vio_image \
        > "${LOG_DIR}/load_bridge.log" 2>&1 &
    echo $! > "${PID_DIR}/load_bridge.pid"
    sleep 6

    # Detached, so the ssh session dying with the board does not take the run with
    # it -- and it is going to die if the hypothesis is right.
    setsid nohup python3 "${TINYNAV}/tinynav/core/build_map_node.py" \
        --bag_file "${db3}" \
        --map_save_path "${X5_ROOT}/maps/loadtest" \
        --play-rate 1.0 --sync-queue-size 20 --no-visualization \
        --loop-closure-mode bow --loop-closure-use-bow \
        --dbow3-vocabulary-path "${VOC}" \
        > "${LOG_DIR}/load_build.log" 2>&1 &
    echo $! > "${PID_DIR}/load_build.pid"

    echo "started. Everything is detached and logging to ${LOG_DIR}."
    echo "You can close this ssh session. Come back with:"
    echo "  bash ${SELF} report"
}

cmd_monitor() {
    local label="${1:-idle-monitor}"
    if [[ -f "${PID_DIR}/health.pid" ]] && kill -0 "$(cat "${PID_DIR}/health.pid")" 2>/dev/null; then
        echo "sampler already running (pid $(cat "${PID_DIR}/health.pid"))"
        return 0
    fi
    setsid nohup bash "${SELF}" _sampler "${label}" >/dev/null 2>&1 &
    echo $! > "${PID_DIR}/health.pid"
    echo "sampler started (pid $!), label=${label} -> ${HEALTH}"
    printf 'baseline: cpu %s C, ddr %s C, uptime %ss, load %s\n' \
        "$(( $(cat /sys/class/thermal/thermal_zone1/temp) / 1000 ))" \
        "$(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))" \
        "$(awk '{printf "%.0f", $1}' /proc/uptime)" \
        "$(awk '{print $1}' /proc/loadavg)"
    echo
    echo "This records temperature, frequency, load and free memory only -- no load is"
    echo "applied. Use it to watch what the board is doing while you change the wiring,"
    echo "so a failure leaves evidence on /userdata instead of vanishing with the board."
}

cmd_mark() {
    # A human-placed marker, so the log says what was done and when rather than
    # leaving the correlation to memory. Reproduction notes belong next to the data.
    printf '# MARK  uptime=%ss  %s\n' \
        "$(awk '{printf "%.0f", $1}' /proc/uptime)" "${*:-unlabelled}" >> "${HEALTH}"
    echo "marked: ${*:-unlabelled}"
}

cmd_stop() {
    for n in load_build load_bridge health; do
        local pf="${PID_DIR}/${n}.pid"
        [[ -f "${pf}" ]] || continue
        local pid; pid="$(cat "${pf}")"
        kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null
        rm -f "${pf}"
        echo "  ${n}: signalled"
    done
    sleep 2
    pkill -f build_map_node 2>/dev/null
    pkill -f looper_bridge_node 2>/dev/null
    echo "stopped"
}

cmd_report() {
    echo "=== now ==="
    printf '  uptime %ss, cpu %s C, ddr %s C, freq %s kHz\n' \
        "$(awk '{printf "%.0f", $1}' /proc/uptime)" \
        "$(( $(cat /sys/class/thermal/thermal_zone1/temp 2>/dev/null || echo 0) / 1000 ))" \
        "$(( $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0) / 1000 ))" \
        "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)"

    [[ -f "${HEALTH}" ]] || { echo "no health log at ${HEALTH}"; return 1; }

    echo
    echo "=== did it reboot? (uptime going backwards marks a power cycle) ==="
    awk -F'\t' '
        /^#/ { next }
        { if (prev != "" && $2 + 0 < prev + 0)
              printf "  REBOOT between uptime %ss and %ss (at %s)\n", prev, $2, $1
          prev = $2 }
        END { if (!seen) print "  (scan complete)" }
    ' "${HEALTH}"

    echo
    echo "=== peak temperature, and how close to the trips ==="
    awk -F'\t' '
        /^#/ { next }
        $3 + 0 > pc { pc = $3; pcu = $2 }
        $4 + 0 > pd { pd = $4 }
        $5 + 0 > 0 && ($5 + 0 < minf || minf == 0) { minf = $5 }
        END {
            printf "  cpu peak %.1f C (at uptime %ss)   passive 95 C, critical 110 C\n", pc/1000, pcu
            printf "  ddr peak %.1f C                    passive 95 C\n", pd/1000
            printf "  lowest cpu freq seen %s kHz (1500000 = no throttling)\n", minf
        }
    ' "${HEALTH}"

    echo
    echo "=== last 12 samples before the log ends / the reboot ==="
    grep -v '^#' "${HEALTH}" | tail -12 | awk -F'\t' \
        '{printf "  %s  up=%-6s cpu=%.1fC ddr=%.1fC freq=%s load=%s avail=%sMB\n",
                 $1, $2, $3/1000, $4/1000, $5, $6, int($7/1024)}'

    echo
    echo "=== build progress reached ==="
    grep -oE 'MAPPING_PERCENT:[0-9.]+' "${LOG_DIR}/load_build.log" 2>/dev/null | tail -1 || echo "  (none)"
    echo "  keyframes: $(grep -c 'Synced TinyNavDB' "${LOG_DIR}/load_build.log" 2>/dev/null || echo 0)"
    echo
    echo "=== verdict inputs ==="
    echo "  If cpu peak approached 95-110 C  -> thermal, and the 8-pin is exonerated."
    echo "  If it died with cpu well under 95 C -> not thermal; suspect the supply."
    echo "  If it never died at all           -> the 8-pin matters under load."
}

case "${1:-}" in
    monitor)  shift; cmd_monitor "$@" ;;
    mark)     shift; cmd_mark "$@" ;;
    run)      shift; cmd_run "$@" ;;
    stop)     cmd_stop ;;
    report)   cmd_report ;;
    _sampler) shift; sampler_loop "${1:-unlabelled}" ;;
    *)
        cat >&2 <<USAGE
Usage: $0 <command>

  monitor [LABEL]           start the sampler only, no load. For watching the board
                            while wiring changes are made.
  mark TEXT...              write a marker line into the log ("plugged 8-pin", ...)
  run [--bag DIR] [--label] start the sampler AND a full-load map build
  report                    read the log back: reboots, peak temperature, throttling
  stop                      stop everything

Reproduce before analysing: 'monitor' + 'mark' first, and only reach for 'run'
once a single-variable reproduction has failed to trigger anything.
USAGE
        exit 1 ;;
esac
