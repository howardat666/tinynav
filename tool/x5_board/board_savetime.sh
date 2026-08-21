#!/bin/sh
# 保存/恢复板上时钟。X5 没有 RTC，每次上电回到 2025-08-26；board_autotime.sh 会用 NTP
# 修好，但它要等 WiFi 拿到 IP，比 journald 晚十几秒。那一下往前跳一年会让 journald
# 轮转日志文件，跨重启的日志因此对不起来——排查重启原因时正好用不上。
#
#   save     把当前时间写到 STAMP（由 timer 每 5 分钟调一次 + 关机时调一次）
#   restore  开机极早期用 STAMP 把时钟顶到"上次见过的时间"，只前进不后退
set -u
STAMP=/var/lib/board-lasttime

case "${1:-}" in
save)
    date +%s > "$STAMP".tmp 2>/dev/null && mv "$STAMP".tmp "$STAMP"
    ;;
restore)
    [ -r "$STAMP" ] || exit 0
    saved=$(cat "$STAMP" 2>/dev/null) || exit 0
    case "$saved" in ''|*[!0-9]*) exit 0 ;; esac      # 只接受纯数字，别把垃圾喂给 date
    now=$(date +%s)
    # 只在保存值更新时才顶，否则会把 NTP 已经校准过的时间拽回去。
    # 显式 exit 0：`[ ... ] && cmd` 在测试为假时会让脚本以 1 退出，systemd 就把这个
    # 单元标成 failed —— 而"不需要顶"是最常见的正常情况。
    if [ "$saved" -gt "$now" ]; then
        date -s "@$saved" >/dev/null 2>&1 || true
    fi
    exit 0
    ;;
*)
    echo "用法: $0 save|restore" >&2; exit 2 ;;
esac
