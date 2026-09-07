#!/bin/bash
# 把相机固件和图像采集服务提到我们所有节点之前。逐线程做：Linux 的 setpriority 对
# 多线程进程只作用于 TID == PID 那一个线程，renice -p PID 改不到其余线程。
set -u
for name in cam-service insight_full; do
    for p in $(pgrep -x "$name" 2>/dev/null); do
        n=0
        for t in /proc/$p/task/*; do
            renice -n -15 -p "$(basename "$t")" >/dev/null 2>&1 && n=$((n+1))
        done
        echo "  $name pid=$p 提优先级的线程 $n 个"
    done
done
