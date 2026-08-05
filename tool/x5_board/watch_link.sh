#!/bin/bash
set -uo pipefail

# Watch the PC->camera USB link and timestamp every state change.
#
#     bash tool/x5_board/watch_link.sh            # until Ctrl-C
#     bash tool/x5_board/watch_link.sh 600        # for 600 seconds
#
# Runs on the PC, not the board. That is the point: when the camera drops, anything
# running on the board stops with it, but this keeps recording, so the exact moment
# of the disconnect is on record and can be lined up against what was being done.
#
# It watches three things, because they fail at different layers:
#   lsusb      is the device electrically present and enumerating at all
#   interface  did cdc_ncm bind and stay bound
#   ping       is the board's network stack actually answering
#
# A drop in lsusb means the board stopped driving USB -- lost power, reset, or hung.
# lsusb present but ping dead would mean the board is up and the link is not, which
# is a completely different problem.

VID_PID="${VID_PID:-3652:0104}"
BOARD_IP="${BOARD_IP:-169.254.10.1}"
duration="${1:-0}"

start=$(date +%s)
prev=""
printf '%s  watching %s / %s  (Ctrl-C to stop)\n' "$(date +%H:%M:%S)" "${VID_PID}" "${BOARD_IP}"

while true; do
    now=$(date +%s)
    [[ "${duration}" -gt 0 && $((now - start)) -ge "${duration}" ]] && break

    usb="no"; lsusb -d "${VID_PID}" >/dev/null 2>&1 && usb="yes"
    iface="no"; ip -br link show 2>/dev/null | grep -q '^enx' && iface="yes"
    net="no"; ping -c 1 -W 1 "${BOARD_IP}" >/dev/null 2>&1 && net="yes"

    state="usb=${usb} iface=${iface} ping=${net}"
    if [[ "${state}" != "${prev}" ]]; then
        printf '%s  %+5ss  %s\n' "$(date +%H:%M:%S)" "$((now - start))" "${state}"
        prev="${state}"
    fi
    sleep 1
done

printf '%s  done after %ss\n' "$(date +%H:%M:%S)" "$(( $(date +%s) - start ))"
