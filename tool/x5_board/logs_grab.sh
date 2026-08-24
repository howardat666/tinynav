#!/bin/sh
# Bundle everything a relocalization+navigation run produced into one tarball.
# Node logs are per-process files under logs/nodes/ and are easy to miss one of.
out="/userdata/x5/run_logs_$(date +%Y%m%d_%H%M%S).tar.gz"
cd /userdata/x5/logs || exit 1
newest() { ls -1t nodes/$1 2>/dev/null | head -1; }
files="app.log"
# diffcar_control 是差速车的执行器（旧车型是 wheel_odometry）。两个都收：换平台时忘了改
# 这里，打出来的包会缺执行器日志而且不报错 —— 和 nav_health 里同一类坑。
for n in map_node diffcar_control wheel_odometry looper_bridge planning cmd_vel_control; do
    f=$(newest "*_${n}.txt")
    [ -n "$f" ] && files="$files $f"
done
# 2026-08-24 新增的三份。它们是掉线/板级问题唯一的时间序列证据，而且是 systemd 服务
# 一直在写、不跟着某一次 run 走，所以以前的打包从来没带上过。
for f in board_health.log board_netheal.log; do
    [ -f "$f" ] && files="$files $f"
done
for pat in "nav_health_*.log" "run_stats_*.tsv"; do
    f=$(ls -1t $pat 2>/dev/null | head -1)
    [ -n "$f" ] && files="$files $f"
done
tar czf "$out" $files 2>/dev/null
echo "wrote $out"
echo "contents:"
for f in $files; do printf "  %-58s %s lines\n" "$f" "$(wc -l < "$f")"; done
