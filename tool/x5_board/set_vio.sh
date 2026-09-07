#!/bin/bash
# 开关相机固件里的 VIO。true|false。可逆。
# 固件已支持 vio_enabled（LooperHub 54d4d7e），关掉时 VioManager 根本不构造，
# 线程不起、vio_100hz/vio_image 【不 advertise】（故意的：一个 advertise 了却永不发
# 的话题是最难查的静默失败），vio_status 仍然 advertise 并锁存 DISABLED。
set -eu
V="${1:?用法: set_vio.sh true|false}"
case "$V" in true|false) ;; *) echo "只能是 true/false"; exit 1 ;; esac
CFG=/userdata/install/share/insight_full/config/user_params.json
[ -f "${CFG}.bak-vio" ] || cp "$CFG" "${CFG}.bak-vio"
python3 - "$CFG" "$V" <<'PY'
import json, sys
p, v = sys.argv[1], sys.argv[2] == 'true'
d = json.load(open(p))
d['vio_enabled'] = v
json.dump(d, open(p, 'w'), indent=2, sort_keys=True, ensure_ascii=False)
print(f"  写入 vio_enabled={v}")
PY
grep -n vio_enabled "$CFG"
echo "--- 重启相机固件 ---"
/etc/init.d/ota_project/scripts/insight-ctl s99 restart >/dev/null 2>&1 || \
  { echo "  insight-ctl 失败，试 systemctl"; systemctl restart S99all_run 2>/dev/null || true; }
sleep 25
echo -n "  insight_full: "; pgrep -x insight_full >/dev/null && echo "在跑 (pid $(pgrep -x insight_full), 起来 $(ps -o etimes= -p $(pgrep -x insight_full) | tr -d ' ')s)" || echo "🔴 没起来"
