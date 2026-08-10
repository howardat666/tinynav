#!/bin/sh
# Per-node CPU and RSS, board temperature/load/memory, and the servo bus failure
# counter -- one TSV row per period, for the length of a run.
#
#     setsid nohup sh /userdata/x5/run_stats.sh 10 >/dev/null 2>&1 &
#
# WHY THIS EXISTS
#   None of it was recorded anywhere. The node logs carry timings and failures but
#   nothing about what the board was doing, so "was it slow because the CPU was
#   saturated or because memory went" could only be answered by happening to run a
#   command at the right moment. MemAvailable reached 134 MB with no swap during one
#   run and that was only noticed by chance.
#
# WHY NOT ps pcpu
#   ps averages CPU over the whole process lifetime, so a spike two minutes ago is
#   diluted into an hour of history. This differences utime+stime across the period,
#   which measures the process as it is now. Same reasoning as proc_cpu.sh.
exec 2>/dev/null
P=${1:-10}
TICK=$(getconf CLK_TCK 2>/dev/null || echo 100)
out="/userdata/x5/logs/run_stats_$(date +%Y%m%d_%H%M%S).tsv"
state=/tmp/.run_stats_prev
NODES="wheel_odometry_node looper_bridge_node planning_node map_node cmd_vel_control uvicorn insight_full"
printf "time\ttemp_c\tload\tavail_mb" > "$out"
for n in $NODES; do printf "\t%s_cpu\t%s_rss" "$n" "$n" >> "$out"; done
printf "\tservo_fail_cum\n" >> "$out"
echo "recording every ${P}s -> $out"
: > $state
while :; do
  now=$(date +%s)
  line="$(date +%H:%M:%S)\t$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)"
  line="$line\t$(awk '{print $1}' /proc/loadavg)\t$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo)"
  new=""
  for n in $NODES; do
    pid=""; 
    for pp in /proc/[0-9]*; do
      case "$(tr '\0' ' ' < $pp/cmdline 2>/dev/null)" in
        # insight_full is not python; everything else must be, so that this script's
        # own shell -- whose command string contains every name in NODES -- cannot match.
        *insight_full*) [ "$n" = insight_full ] && pid=${pp#/proc/} ;;
        python3*$n*) [ "$n" != insight_full ] && pid=${pp#/proc/} ;;
      esac
      [ -n "$pid" ] && break
    done
    if [ -z "$pid" ]; then line="$line\t-\t-"; continue; fi
    set -- $(cat /proc/$pid/stat 2>/dev/null)
    jif=$(( ${14:-0} + ${15:-0} ))
    rss=$(awk '/VmRSS/{print $2}' /proc/$pid/status 2>/dev/null)
    prev=$(grep "^$n:$pid:" $state 2>/dev/null | cut -d: -f3)
    prevt=$(grep "^$n:$pid:" $state 2>/dev/null | cut -d: -f4)
    if [ -n "$prev" ] && [ "$now" -gt "${prevt:-$now}" ]; then
      cpu=$(awk -v a="$jif" -v b="$prev" -v d="$((now-prevt))" -v t="$TICK" 'BEGIN{printf "%.1f", 100*(a-b)/t/d}')
    else cpu="-"; fi
    line="$line\t$cpu\t$((${rss:-0}/1024))"
    new="$new$n:$pid:$jif:$now\n"
  done
  printf "%b" "$new" > $state
  f=""
  for pp in /proc/[0-9]*; do
    case "$(tr '\0' ' ' < $pp/cmdline 2>/dev/null)" in
      python3*wheel_odometry_node*)
        l=$(readlink $pp/fd/1); f=$(grep "read failed" "$l" 2>/dev/null | tail -1 | sed 's/.*(\([0-9]*\) total).*/\1/') ;;
    esac
  done
  printf "$line\t${f:-0}\n" >> "$out"
  sleep "$P"
done
