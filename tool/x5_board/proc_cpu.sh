#!/bin/sh
# CPU consumed by one process over a fixed window, from /proc/<pid>/stat.
#
#     sh tool/x5_board/proc_cpu.sh looper_bridge_node 60
#
# WHY NOT ps pcpu
#   ps averages over the whole process lifetime, so a change made two minutes ago is
#   diluted into an hour of history and looks like it did nothing. A utime+stime delta
#   over a known window measures the process as it is now.
#
# WHY THE PATTERN IS BRACKETED HERE RATHER THAN BY THE CALLER
#   `pgrep -f` matches full command lines, including the shell that is running this
#   script and -- over ssh -- the remote command string, which contains the pattern.
#   Combined with `head -1` that silently returns whichever pid happens to be lower:
#   measured 108.3% one minute and 0.0% the next for the same process, because a
#   wrapper shell had the lower pid the second time. Turning "looper" into "[l]ooper"
#   makes the regex match the real process without matching the text of the command
#   that contains it. Doing it inside the script means no caller can forget.
#
#   It also prints the command line it settled on. A measurement tool that can pick the
#   wrong process must show its choice, or a wrong number reads exactly like a right one.
PAT="$1"
SECS="${2:-60}"

[ -z "$PAT" ] && { echo "usage: $0 <process-pattern> [seconds]" >&2; exit 2; }

FIRST=$(printf '%s' "$PAT" | cut -c1)
REST=$(printf '%s' "$PAT" | cut -c2-)
SAFE="[${FIRST}]${REST}"

PID=$(pgrep -f "$SAFE" | head -1)
[ -z "$PID" ] && { echo "no process matching '$PAT'" >&2; exit 1; }

CMD=$(tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | cut -c1-80)
echo "  matched pid $PID: $CMD"

HZ=$(getconf CLK_TCK)
jiffies() { awk '{print $14+$15}' /proc/$1/stat 2>/dev/null; }

A=$(jiffies "$PID")
sleep "$SECS"
B=$(jiffies "$PID")
[ -z "$B" ] && { echo "  process $PID exited during the window" >&2; exit 1; }

awk -v a="$A" -v b="$B" -v s="$SECS" -v hz="$HZ" \
  'BEGIN{printf "  %.2f cpu-seconds over %ss = %.1f%% of one core\n", (b-a)/hz, s, 100*(b-a)/hz/s}'
