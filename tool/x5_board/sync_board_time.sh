#!/usr/bin/env bash
# Sync the Looper camera's X5 clock to this PC's clock.
#
#     ./sync_board_time.sh              # default target root@169.254.10.1
#     HOST=root@... ./sync_board_time.sh
#
# WHY THIS IS NEEDED, EVERY BOOT
# ------------------------------
# The X5 has no battery-backed RTC -- `hwclock -r` reads 1970-01-01 -- and no
# NTP daemon runs (the `ntpdate` binary exists but nothing invokes it).  The
# board therefore boots to whatever date the rootfs was stamped with; it was
# found ~11 months behind this PC.
#
# That matters because ROS 2 message timestamps come from the system clock.  As
# long as every node runs on the board the error is invisible (all timestamps
# are consistently wrong).  The moment a PC-side node joins -- rviz, a bag
# recorder, a relay -- `message_filters` ApproximateTimeSynchronizer compares
# stamps across the two machines, never finds a match, and simply drops
# everything.  It does not warn.  So: sync before mixing PC and board nodes.
#
# A naive `ssh board date -s @$(date +%s)` leaves several seconds of error,
# because the epoch is captured before the SSH handshake completes.  This script
# instead measures the offset the way NTP does -- bracketing the remote reading
# between two local readings -- and corrects for half the round trip, then
# iterates until the residual is small.
set -euo pipefail

HOST="${HOST:-root@169.254.10.1}"
PASS="${PASS:-looper@0731}"
TOLERANCE="${TOLERANCE:-0.25}"   # seconds; loop stops once |offset| is under this
MAX_ROUNDS="${MAX_ROUNDS:-6}"

SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR)
ssh_() { sshpass -p "${PASS}" ssh "${SSH_OPTS[@]}" "${HOST}" "$@"; }

command -v sshpass >/dev/null || { echo "sshpass not installed" >&2; exit 1; }

# Echo the board-minus-PC offset in seconds, RTT-compensated.
measure_offset() {
    local t1 board t2 mid
    t1=$(date +%s.%N)
    board=$(ssh_ 'date +%s.%N')
    t2=$(date +%s.%N)
    # Assume the remote reading happened halfway through the round trip.
    mid=$(awk -v a="${t1}" -v b="${t2}" 'BEGIN{printf "%.6f", (a+b)/2}')
    awk -v r="${board}" -v m="${mid}" 'BEGIN{printf "%.6f", r-m}'
}

abs_lt() { awk -v v="$1" -v t="$2" 'BEGIN{if (v<0) v=-v; exit !(v<t)}'; }

echo "==> target ${HOST}"
offset=$(measure_offset)
echo "==> initial offset: ${offset} s (board minus PC)"

if abs_lt "${offset}" "${TOLERANCE}"; then
    echo "==> already within ${TOLERANCE} s, no correction needed"
    MAX_ROUNDS=0
fi

for round in $(seq 1 "${MAX_ROUNDS}"); do
    # Correct the board by -offset: read its own clock and subtract the error.
    # Doing the arithmetic on the board keeps the SSH latency out of the value.
    ssh_ "python3 -c \"import subprocess, time
target = time.time() - (${offset})
subprocess.run(['date', '-u', '-s', '@%.6f' % target], check=True)\"" >/dev/null

    offset=$(measure_offset)
    echo "==> round ${round}: offset now ${offset} s"
    if abs_lt "${offset}" "${TOLERANCE}"; then
        echo "==> converged"
        break
    fi
done

if ! abs_lt "${offset}" "${TOLERANCE}"; then
    echo "!! still ${offset} s off after ${MAX_ROUNDS} rounds" >&2
    exit 1
fi

# Best effort only: with no RTC battery this will not survive a power cycle,
# which is exactly why this script has to be re-run after every boot.
ssh_ 'hwclock -w' >/dev/null 2>&1 && echo "==> wrote RTC" || echo "==> RTC write skipped (no battery)"

# --- tell insight_full the clock is trustworthy --------------------------------
# Syncing the clock is only half the job.  `insight_full` stamps every message
# from CLOCK_MONOTONIC unless this flag file contains 1, in which case it adds
# (REALTIME - MONOTONIC) and publishes wall-clock stamps instead.  Without it,
# camera stamps sit near the uptime (~800) while any Python node stamping its own
# output with `self.get_clock().now()` uses REALTIME (~1.78e9): TF lookups at an
# image's timestamp raise, and message_filters silently matches nothing.
#
# A monitor thread inside insight_full re-reads the file once a second, so this
# takes effect without a restart.  Setting it only makes sense *after* the sync
# above, which is why the two live in one script.
FLAG=/etc/init.d/looper/setting/is_time_sync
if [ "${SET_TIME_SYNC_FLAG:-1}" = "1" ]; then
    ssh_ "mkdir -p /userdata/fixbak
          [ -f ${FLAG} ] && [ ! -f /userdata/fixbak/is_time_sync.orig ] && cp -a ${FLAG} /userdata/fixbak/is_time_sync.orig
          echo 1 > ${FLAG}"
    echo "==> set ${FLAG}=1 (insight_full will publish wall-clock stamps within ~1 s)"
fi

echo
echo "board: $(ssh_ 'date -u')"
echo "pc:    $(date -u)"
