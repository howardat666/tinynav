#!/usr/bin/env bash
# 棘轮：板上运维层的这些数字只许降不许升。`--bless` 重设基线。
# 每条都对应一次真实故障。CI 里跑这个比跑 lint 更值——这一层至今零 lint 零测试。
set -uo pipefail
cd "$(dirname "$0")/../.." || exit 2
BASE=tool/x5_board/invariants.baseline
T=tool/x5_board

measure() {
  # why: app_start.sh 先 source env.sh 再做 export VAR="${VAR:-默认}"，:- 分支永不触发。
  #      这 20 个"默认值"一个都 pin 不住，而注释写着 "Pinned here so they never fall back"。
  printf 'app_start_fake_defaults\t%s\t%s\n' \
    "$(grep -cE 'export [A-Z_]+="\$\{[A-Z_]+:-' $T/app_start.sh)" \
    '假默认值，应随配置分三层一起归零'

  # why: sh() "绝不抛异常"且 rc 从不作判据，dhcp-renew 连续 34 次 rc=-15 淹在 INFO 里。
  printf 'netheal_swallowed_except\t%s\t%s\n' \
    "$(grep -cE '^\s*except [A-Za-z(].*:\s*$' $T/board_netheal.py)" \
    '吞异常的地方，应改成显式忽略或抛出'

  # why: shell=True 把 pkill 和 udhcpc 拼进同一条命令行，pkill 打死了自己。
  printf 'netheal_shell_true\t%s\t%s\n' \
    "$(grep -c 'shell=True' $T/board_netheal.py)" \
    '应改成 argv 列表，从形式上消灭拼接类故障'

  # why: 131 个文件里真正常驻运行的只有 6 个，约 20 个是给已拆硬件写的死代码。
  printf 'x5_board_toplevel_files\t%s\t%s\n' \
    "$(find $T -maxdepth 1 -type f | wc -l)" \
    '顶层文件数，一次性脚本应进 archive/'
}

if [ "${1:-}" = "--bless" ]; then measure > "$BASE"; echo "基线已重设:"; cat "$BASE"; exit 0; fi
[ -f "$BASE" ] || { echo "缺基线，先跑 $0 --bless"; exit 2; }

rc=0
while IFS=$'\t' read -r k now why; do
  was=$(awk -F'\t' -v k="$k" '$1==k{print $2}' "$BASE")
  [ -n "$was" ] || { echo "新指标 $k=$now"; continue; }
  if   [ "$now" -gt "$was" ]; then echo "🔴 $k  $was -> $now  升了。$why"; rc=1
  elif [ "$now" -lt "$was" ]; then echo "🟢 $k  $was -> $now  降了，记得 --bless"
  else echo "   $k  $now"; fi
done < <(measure)
exit $rc
