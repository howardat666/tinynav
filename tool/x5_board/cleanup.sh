#!/bin/sh
# Kill by pid discovered from /proc, never from `ps | grep`: this script's own
# command string would match the node name and it would kill its own shell.
echo "=== ttyS3 holders before ==="
for p in /proc/[0-9]*; do
  for f in $p/fd/*; do
    case "$(readlink $f 2>/dev/null)" in
      */ttyS3) echo "  HOLDER ${p#/proc/}  started $(date -d @$(stat -c %Y $p) +%H:%M:%S)";;
    esac
  done
done

# 保留最新的串口占用者，杀掉更老的重复实例。两个平台的执行器都查：差速车是
# diffcar_control，旧车型是 wheel_odometry_node —— 换平台时漏改这里，重复实例会
# 一起往 /dev/ttyS3 写，两边的字节互相插进对方的流里（2026-08-21 因此毁掉一次 OTA）。
ACT='*wheel_odom*node.py*|*diffcar_control.py*'
newest=""; newest_t=0
for p in /proc/[0-9]*; do
  case "$(tr '\0' ' ' < $p/cmdline 2>/dev/null)" in
    *wheel_odom*node.py*|*diffcar_control.py*)
      t=$(stat -c %Y $p)
      if [ "$t" -gt "$newest_t" ]; then newest_t=$t; newest=${p#/proc/}; fi ;;
  esac
done
for p in /proc/[0-9]*; do
  pid=${p#/proc/}
  case "$(tr '\0' ' ' < $p/cmdline 2>/dev/null)" in
    *wheel_odom*node.py*|*diffcar_control.py*)
      if [ "$pid" != "$newest" ]; then echo "killing stale duplicate $pid"; kill -9 "$pid"; fi ;;
    *bus_loss_live*|*run_stats.sh*|*nav_health_record.sh*|*servo_usb_ground_ab*|*servo_failure_pattern*)
      echo "stopping recorder/diagnostic $pid"; kill -9 "$pid" ;;
  esac
done
sleep 2
echo "=== after ==="
n=0
for p in /proc/[0-9]*; do
  for f in $p/fd/*; do
    case "$(readlink $f 2>/dev/null)" in
      */ttyS3) echo "  HOLDER ${p#/proc/}"; n=$((n+1));;
    esac
  done
done
echo "  ttyS3 holders = $n  (must be 1)"
grep -E "MemAvailable" /proc/meminfo
