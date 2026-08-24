#!/bin/sh
exec 2>/dev/null   # /proc entries vanish mid-scan; those races are not findings
# One screen answering "is the navigation chain actually working right now".
# Every number here had to be dug out of five separate log files by hand during the
# 2026-08-10 run; the chain fails silently at four different layers and each layer
# looks like success from the one above it.
N=/userdata/x5/logs/nodes
# Resolve each node's log from the LIVE process, not from the newest matching file.
# Globbing by mtime picked a killed duplicate's log on 2026-08-10 -- both files carried
# the same mtime to the second, ls -t broke the tie arbitrarily, and the summary
# reported 581 read failures that belonged to a process that no longer existed. Same
# class as tailing app.log for lines that only ever went to logs/nodes/: a confident
# wrong number, which is worse than no number.
#
# Two guards against matching this script's own shell, whose command string contains
# every node name below: require the cmdline to start with python3, and require fd 1
# to point inside logs/nodes/.
newest() {
  for pp in /proc/[0-9]*; do
    c=$(tr '\0' ' ' < $pp/cmdline 2>/dev/null) || continue
    case "$c" in
      python3*$1*)
        l=$(readlink $pp/fd/1 2>/dev/null)
        case "$l" in $N/*) echo "$l"; return 0;; esac ;;
    esac
  done
  # Nothing live: fall back to the newest file so a post-mortem still works, but the
  # caller can tell the difference because the live case never reaches here.
  ls -1t $N/*_$1.txt 2>/dev/null | head -1
}
W=${1:-120}   # seconds of history to summarise
now=$(date +%s)
since=$((now - W))
# rclpy stamps [INFO] [<epoch>.<ns>]; awk keeps lines newer than $since.
recent() { [ -f "$1" ] && awk -v s="$since" 'match($0,/\[[0-9]+\.[0-9]+\]/){t=substr($0,RSTART+1,RLENGTH-2)+0; if(t>=s) print}' "$1"; }

echo "=== last ${W}s ==="
m=$(newest map_node.py); p=$(newest planning_node.py); c=$(newest cmd_vel_control)
# 2026-08-24 起差速车的执行器是 diffcar_control，不再是 wheel_odometry_node。两个都查:
# 换平台时忘了改这里，会让整节静默变成"没有异常"。
w=$(newest wheel_odometry_node.py); dc=$(newest diffcar_control.py)

echo "-- relocalization (map_node) --"
if [ -n "$m" ]; then
  r=$(recent "$m" | grep "window:" | tail -2 | sed 's/.*\]: //')
  # An empty section is ambiguous -- a stale log file from a previous run still
  # matches the glob, so "no recent lines" must be spelled out rather than shown
  # as blank space that reads like "nothing wrong".
  if [ -n "$r" ]; then echo "$r"; else echo "   nothing in window (map_node stopped, or nav nodes not enabled)"; fi
else echo "   no map_node log at all -- nav nodes have never been enabled"; fi

# Rates, which the window line carries but only implicitly: it prints the keyframe
# COUNT and the attempt/success rates, so the keyframe rate -- the one that says
# whether the perception front end is feeding map_node fast enough -- had to be
# divided out by hand every time.
#
# sed rather than awk's match(str, re, arr): that form is a GNU extension and this
# board has busybox awk, where it is a syntax error rather than a wrong answer.
if [ -n "$m" ]; then
  recent "$m" | grep "window:" | tail -3 | while read -r ln; do
    span=$(echo "$ln" | sed -n 's/.*relocalization \([0-9]*\)s window.*/\1/p')
    kf=$(echo "$ln" | sed -n 's/.*window: \([0-9]*\) keyframes.*/\1/p')
    att=$(echo "$ln" | sed -n 's/.*attempts (\([0-9.]*\) Hz).*/\1/p')
    okp=$(echo "$ln" | sed -n 's/.*ok (\([^)]*\)).*/\1/p')
    [ -n "$span" ] && [ "$span" != "0" ] || continue
    khz=$(awk -v a="$kf" -v b="$span" 'BEGIN{printf "%.2f", a/b}')
    echo "   rates over ${span}s: keyframe ${khz} Hz | reloc attempt ${att} Hz | reloc ok ${okp}"
  done
fi

echo "-- planning --"
if [ -n "$p" ]; then
  nav=$(recent "$p" | grep -c "navigating:")
  reach=$(recent "$p" | grep -c "Target pose reached")
  notgt=$(recent "$p" | grep -c "No target pose")
  echo "   navigating=$nav   reached=$reach   no_target=$notgt"
  recent "$p" | grep "navigating:" | tail -2 | sed 's/.*\]: //'
  [ "$nav" = "0" ] && [ "$reach" != "0" ] && echo "   !! never navigating, always 'reached' -- check the arrival test axes"
fi

echo "-- avoidance (planning decision log) --"
if [ -n "$p" ]; then
  # The config line is printed once at startup, so it comes from the whole file, not
  # the window: a run's numbers are unreadable without knowing which parameters made
  # them, and this is the only place they appear.
  grep -h "Robot: " "$p" | tail -1 | sed 's/.*\]: /   /'
  d=$(recent "$p" | grep "decision:")
  n=$(echo "$d" | grep -c "decision:")
  if [ "$n" = "0" ]; then
    echo "   no decisions in window (no target, or planning is not running)"
  else
    rev=$(echo "$d" | grep -c "chose vx=-")
    zero=$(echo "$d" | grep -c "chose vx=+0.000 omega=+0.000")
    echo "   decisions=$n   turn-in-place=$(echo "$d" | grep -c TURN-IN-PLACE)   reverse=$rev   standstill=$zero"
    echo "   escape:$(for r in off blocked heading no-progress; do
        printf ' %s=%s' "$r" "$(echo "$d" | grep -c "escape=$r")"; done)"
    # turns=0 while the gate is turn-only is the silent failure the 2026-08-11 change
    # fixed: the escape hatch fires but has nothing admissible to commit to.
    t0=$(echo "$d" | grep "gate=turn-only" | grep -c "turns=0 ")
    [ "$t0" != "0" ] && echo "   !! $t0 cycles blocked with turns=0 -- escape hatch had nothing to pick"
    for k in blocked= obstacle_cells= esdf_at_robot= front_clearance=; do
      printf '   %-18s' "$k"
      echo "$d" | sed -n "s/.*$k\([^ ]*\).*/\1/p" | sort -V | awk '
        {a[NR]=$1} END {if(NR) printf "p50=%s  min=%s  max=%s  (n=%d)\n", a[int(NR/2)+1], a[1], a[NR], NR; else print "-"}'
    done
    echo "$d" | tail -1 | sed 's/.*\]: /   last: /'
  fi
fi

echo "-- control (cmd_vel_control) --"
if [ -n "$c" ]; then
  tot=$(recent "$c" | grep -c "sent cmd_vel")
  nz=$(recent "$c" | grep "sent cmd_vel" | grep -vc "vx=0.000 vyaw=0.000")
  acc=$(recent "$c" | grep -c "accepted")
  ign=$(recent "$c" | grep -c "ignored")
  # "stale" is a lag *warning*; the path is still accepted. Reporting it next to
  # "fresh" read as a rejection count and sent two sessions chasing a non-problem.
  stale=$(recent "$c" | grep -c "received stale")
  exp=$(recent "$c" | grep -c "trajectory expired")
  echo "   cmd_vel: $nz/$tot non-zero   trajectory: $acc accepted / $ign ignored   expired: $exp   lag-warnings: $stale"
  [ "$nz" = "0" ] && echo "   !! robot is commanded to stand still"
fi

echo "-- actuator --"
if [ -n "$w" ]; then
  f=$(recent "$w" | grep "read failed" | tail -1 | sed 's/.*(\([0-9]*\) total).*/\1/')
  f0=$(recent "$w" | grep "read failed" | head -1 | sed 's/.*(\([0-9]*\) total).*/\1/')
  if [ -n "$f" ] && [ -n "$f0" ]; then echo "   servo bus read failures in window: $((f - f0))  (cumulative $f)"
  else echo "   servo bus: no read failures logged in window"; fi
fi
if [ -n "$dc" ]; then
  sag=$(recent "$dc" | grep -c "battery sagged")
  low=$(recent "$dc" | grep "battery sagged" | sed -n 's/.*sagged to \([0-9.]*\) V.*/\1/p' \
        | sort -n | head -1)
  pcbad=$(recent "$dc" | grep -c "unreachable")
  pcok=$(recent "$dc" | grep -c "reachable again")
  # 固件主动上报的告警：ESP32 偷偷重启 / 堵转 / 失联停车 / 闭环被 CFG_REV 锁住。
  # 这些只在串口上说一次，2026-08-24 之前 diffcar_control 把它们全丢了。
  fw=$(recent "$dc" | grep -c "firmware:")
  echo "   diffcar_control: 电池告警 $sag 次${low:+（最低 ${low} V）}   PC 不可达 $pcbad / 恢复 $pcok"
  if [ "$fw" -gt 0 ]; then
    echo "   !! 固件告警 $fw 条:"; recent "$dc" | grep "firmware:" | tail -5 | sed 's/^/      /'
  else
    echo "   固件告警: 无（重启/堵转/失联/闭环锁 都没发生）"
  fi
else
  echo "   !! 没有 diffcar_control 日志 —— 执行器没起，车不会动"
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
[ "$h" = "0" ] && echo "   !! nobody holds /dev/ttyS3 -- 执行器(diffcar_control)没在跑"
for n in wheel_odometry_node diffcar_control map_node planning_node looper_bridge_node cmd_vel_control; do
  c=0
  for pp in /proc/[0-9]*; do
    case "$(tr '\0' ' ' < $pp/cmdline 2>/dev/null)" in python3*$n*) c=$((c+1));; esac
  done
  [ "$c" -gt 1 ] && echo "   !! $c copies of $n running"
done
for pp in /proc/[0-9]*; do
  case "$(tr '\0' ' ' < $pp/cmdline 2>/dev/null)" in
    python3*bus_loss_live*|python3*servo_usb_ground_ab*|python3*servo_failure_pattern*)
      echo "   note: diagnostic tool still running (pid ${pp#/proc/}) -- it costs CPU and RAM";;
  esac
done

echo "-- board --"
printf "   temp %sC   load %s   avail %s kB\n" \
  "$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)" \
  "$(awk '{print $1}' /proc/loadavg)" \
  "$(awk '/MemAvailable/{print $2}' /proc/meminfo)"
# 链路。掉线两次的判据都在这一行里：fa 阶跃到 >1000 是射频干扰，usb=0 才是网卡掉电。
BH=/userdata/x5/logs/board_health.log
[ -r "$BH" ] && echo "   link: $(tail -1 "$BH" | cut -d' ' -f3-)"
NH=/userdata/x5/logs/board_netheal.log
if [ -r "$NH" ]; then
  n=$(tail -40 "$NH" | grep -c "第.级")
  [ "$n" -gt 0 ] && { echo "   !! netheal 动过手 $n 次:"; tail -4 "$NH" | sed 's/^/      /'; }
fi
