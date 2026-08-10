#!/bin/sh
# One screen answering "is the navigation chain actually working right now".
# Every number here had to be dug out of five separate log files by hand during the
# 2026-08-10 run; the chain fails silently at four different layers and each layer
# looks like success from the one above it.
N=/userdata/x5/logs/nodes
newest() { ls -1t $N/*_$1.txt 2>/dev/null | head -1; }
W=${1:-120}   # seconds of history to summarise
now=$(date +%s)
since=$((now - W))
# rclpy stamps [INFO] [<epoch>.<ns>]; awk keeps lines newer than $since.
recent() { [ -f "$1" ] && awk -v s="$since" 'match($0,/\[[0-9]+\.[0-9]+\]/){t=substr($0,RSTART+1,RLENGTH-2)+0; if(t>=s) print}' "$1"; }

echo "=== last ${W}s ==="
m=$(newest map_node); p=$(newest planning); c=$(newest cmd_vel_control); w=$(newest wheel_odometry)

echo "-- relocalization (map_node) --"
if [ -n "$m" ]; then
  r=$(recent "$m" | grep "window:" | tail -2 | sed 's/.*\]: //')
  # An empty section is ambiguous -- a stale log file from a previous run still
  # matches the glob, so "no recent lines" must be spelled out rather than shown
  # as blank space that reads like "nothing wrong".
  if [ -n "$r" ]; then echo "$r"; else echo "   nothing in window (map_node stopped, or nav nodes not enabled)"; fi
else echo "   no map_node log at all -- nav nodes have never been enabled"; fi

echo "-- planning --"
if [ -n "$p" ]; then
  nav=$(recent "$p" | grep -c "navigating:")
  reach=$(recent "$p" | grep -c "Target pose reached")
  notgt=$(recent "$p" | grep -c "No target pose")
  echo "   navigating=$nav   reached=$reach   no_target=$notgt"
  recent "$p" | grep "navigating:" | tail -2 | sed 's/.*\]: //'
  [ "$nav" = "0" ] && [ "$reach" != "0" ] && echo "   !! never navigating, always 'reached' -- check the arrival test axes"
fi

echo "-- control (cmd_vel_control) --"
if [ -n "$c" ]; then
  tot=$(recent "$c" | grep -c "sent cmd_vel")
  nz=$(recent "$c" | grep "sent cmd_vel" | grep -vc "vx=0.000 vyaw=0.000")
  acc=$(recent "$c" | grep -c "accepted")
  stale=$(recent "$c" | grep -c "stale")
  exp=$(recent "$c" | grep -c "trajectory expired")
  echo "   cmd_vel: $nz/$tot non-zero   trajectory: $acc fresh / $stale stale   expired: $exp"
  [ "$nz" = "0" ] && echo "   !! robot is commanded to stand still"
fi

echo "-- servo bus (wheel_odometry) --"
if [ -n "$w" ]; then
  f=$(recent "$w" | grep "read failed" | tail -1 | sed 's/.*(\([0-9]*\) total).*/\1/')
  f0=$(recent "$w" | grep "read failed" | head -1 | sed 's/.*(\([0-9]*\) total).*/\1/')
  if [ -n "$f" ] && [ -n "$f0" ]; then echo "   read failures in window: $((f - f0))  (cumulative $f)"
  else echo "   no read failures logged in window"; fi
fi
echo "-- processes --"
# app_start.sh stop has been seen to leave the previous wheel_odometry alive, so two
# of them ended up sharing /dev/ttyS3 across a restart. Nothing reported it: both
# nodes log "wheel odometry up" and the port opens for both. Same class of failure as
# a leaked measurement process, so both are checked here rather than remembered.
h=0
for pp in /proc/[0-9]*; do
  for f in $pp/fd/*; do
    case "$(readlink $f 2>/dev/null)" in */ttyS3) h=$((h+1));; esac
  done
done
echo "   ttyS3 holders: $h"
[ "$h" -gt 1 ] && echo "   !! more than one process holds the servo bus -- run cleanup.sh"
[ "$h" = "0" ] && echo "   !! nobody holds the servo bus -- wheel_odometry is down"
for n in wheel_odometry_node map_node planning_node looper_bridge_node cmd_vel_control; do
  c=0
  for pp in /proc/[0-9]*; do
    case "$(tr '\0' ' ' < $pp/cmdline 2>/dev/null)" in *$n*) c=$((c+1));; esac
  done
  [ "$c" -gt 1 ] && echo "   !! $c copies of $n running"
done
for pp in /proc/[0-9]*; do
  case "$(tr '\0' ' ' < $pp/cmdline 2>/dev/null)" in
    *bus_loss_live*|*servo_usb_ground_ab*|*servo_failure_pattern*)
      echo "   note: diagnostic tool still running (pid ${pp#/proc/}) -- it costs CPU and RAM";;
  esac
done

echo "-- board --"
printf "   temp %sC   load %s   avail %s kB\n" \
  "$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)" \
  "$(awk '{print $1}' /proc/loadavg)" \
  "$(awk '/MemAvailable/{print $2}' /proc/meminfo)"
