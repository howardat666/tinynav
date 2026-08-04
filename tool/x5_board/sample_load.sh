#!/bin/sh
# Sample the load a target process puts on the X5, once a second, to a TSV.
#
#     sample_load.sh <out.tsv> <pattern> [max_seconds]
#
# Columns: t_s  rss_mb  proc_cpu_cores  sys_cpu_cores  avail_mb  temp_c  bpu_pct
#
# `proc_cpu_cores` is the target's own utime+stime delta over the interval,
# expressed in cores (1.0 = one core fully busy). `sys_cpu_cores` is the same for
# the whole machine from /proc/stat, excluding idle and iowait.
#
# Why not `top`/`ps %CPU`: ps reports an average since process start, which hides
# the phase we care about, and top's own sampling adds load comparable to what we
# are trying to measure -- an earlier attempt at this used ~90 awk forks per
# sample and the measurement error came out the same size as the effect.
# Everything here is one read per file per second, with no forks inside the loop
# beyond a single awk.
set -u
OUT=$1
PATTERN=$2
MAX=${3:-1800}

TICKS=$(getconf CLK_TCK 2>/dev/null || echo 100)

printf 't_s\trss_mb\tproc_cpu_cores\tsys_cpu_cores\tavail_mb\ttemp_c\tbpu_pct\n' > "$OUT"

find_pid() {
    # Pick the python process matching the pattern, not the shell or ssh wrapper
    # that happens to carry the same string on its command line.
    for p in $(pgrep -f "$PATTERN" 2>/dev/null); do
        [ -r "/proc/$p/comm" ] || continue
        case "$(cat "/proc/$p/comm" 2>/dev/null)" in
            python*) echo "$p"; return 0 ;;
        esac
    done
    return 1
}

prev_proc=-1
prev_sys_busy=-1
prev_sys_total=-1
i=0
missing=0

while [ "$i" -lt "$MAX" ]; do
    PID=$(find_pid) || PID=""
    if [ -z "$PID" ]; then
        missing=$((missing + 1))
        # Allow a slow start, but stop once the process has clearly exited.
        [ "$i" -gt 10 ] && [ "$missing" -gt 5 ] && break
    else
        missing=0
    fi

    RSS=0
    PROC_TICKS=0
    if [ -n "$PID" ] && [ -r "/proc/$PID/stat" ]; then
        # utime is field 14, stime field 15, rss (pages) field 24.
        set -- $(cut -d' ' -f14,15,24 "/proc/$PID/stat" 2>/dev/null)
        if [ $# -eq 3 ]; then
            PROC_TICKS=$(( $1 + $2 ))
            RSS=$(( $3 * 4 / 1024 ))
        fi
    fi

    set -- $(head -1 /proc/stat)
    shift
    # user nice system idle iowait irq softirq steal
    SYS_TOTAL=0
    n=0
    for v in "$@"; do
        n=$((n + 1))
        [ "$n" -gt 8 ] && break
        SYS_TOTAL=$((SYS_TOTAL + v))
    done
    SYS_IDLE=$(( $4 + $5 ))
    SYS_BUSY=$(( SYS_TOTAL - SYS_IDLE ))

    AVAIL=$(awk '/MemAvailable/{print int($2/1024); exit}' /proc/meminfo)
    T=0
    [ -r /sys/class/thermal/thermal_zone0/temp ] && T=$(cat /sys/class/thermal/thermal_zone0/temp)
    BPU=0
    [ -r /sys/devices/system/bpu/bpu0/ratio ] && BPU=$(cat /sys/devices/system/bpu/bpu0/ratio 2>/dev/null || echo 0)

    if [ "$prev_proc" -ge 0 ]; then
        PC=$(awk -v a="$PROC_TICKS" -v b="$prev_proc" -v t="$TICKS" 'BEGIN{v=(a-b)/t; if(v<0)v=0; printf "%.3f", v}')
        SC=$(awk -v a="$SYS_BUSY" -v b="$prev_sys_busy" -v t="$TICKS" 'BEGIN{v=(a-b)/t; if(v<0)v=0; printf "%.3f", v}')
        printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$i" "$RSS" "$PC" "$SC" "$AVAIL" "$((T / 1000))" "$BPU" >> "$OUT"
    fi
    prev_proc=$PROC_TICKS
    prev_sys_busy=$SYS_BUSY
    prev_sys_total=$SYS_TOTAL

    i=$((i + 1))
    sleep 1
done
