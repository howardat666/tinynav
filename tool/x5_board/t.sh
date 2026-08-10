#!/bin/sh
# Short wrapper for the board test loop.
#
# The commands it replaces were four lines of nested quoting each, which is a real
# cost: an operator who cannot read a command at a glance cannot tell whether it
# ran, and the 2026-08-10 15:00 session ended with the recorders started twice and
# the operator unsure whether anything had been captured at all. Every subcommand
# here prints what it did.
#
#   t.sh rec    start the recorders (prints confirmation)
#   t.sh watch  live one-screen chain health, Ctrl-C to quit
#   t.sh end    stop recorders, kill duplicates, bundle the logs
#   t.sh st     one-shot status
set -e
LOGS=/userdata/x5/logs

live() {  # live <pattern> -> prints "pid cmd" for each matching process
    for p in /proc/[0-9]*; do
        c=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null) || continue
        case "$c" in $1) echo "  pid=$(basename "$p")  $c" ;; esac
    done
}

case "$1" in
rec)
    setsid sh /userdata/x5/run_stats.sh 10 >/dev/null 2>&1 &
    setsid sh /userdata/x5/nav_health_record.sh 15 60 >/dev/null 2>&1 &
    sleep 3
    echo "== 采集器 =="
    live '*run_stats*'
    live '*nav_health_record*'
    echo "== 输出文件 =="
    ls -la --time-style=+%H:%M:%S "$LOGS"/run_stats_*.tsv "$LOGS"/nav_health_*.log 2>/dev/null | tail -2
    ;;
watch)
    while true; do clear; sh /userdata/x5/nav_health.sh; sleep 5; done
    ;;
end)
    sh /userdata/x5/cleanup.sh || true
    sh /userdata/x5/logs_grab.sh
    ;;
st)
    echo "== scheme =="; cat "$LOGS/app.scheme" 2>/dev/null || echo "(none)"
    echo "== 节点 =="; live 'python3*'
    echo "== ttyS3 持有者（必须正好 1 个）=="
    n=0
    for f in /proc/[0-9]*/fd/*; do
        [ "$(readlink "$f" 2>/dev/null)" = "/dev/ttyS3" ] && { echo "  $f"; n=$((n+1)); }
    done
    [ "$n" = 1 ] || echo "  !! 有 $n 个，不是 1 个"
    echo "== 预热 =="
    P=$(ls -t "$LOGS"/nodes/*_planning.txt 2>/dev/null | head -1)
    [ -n "$P" ] && grep -h "kernels ready" "$P" || echo "  (还没跑完)"
    ;;
*)
    echo "用法: t.sh rec | watch | end | st"; exit 1 ;;
esac
