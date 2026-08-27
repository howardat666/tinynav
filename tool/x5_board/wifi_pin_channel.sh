#!/usr/bin/env bash
# Patch the firmware's wifi-connect.sh so wpa_supplicant is controllable and, optionally,
# restricted to one 2.4 GHz channel. Idempotent; keeps a .orig backup; re-run after an OTA.
#
# 为什么要这两样（2026-08-27 实测，见 docs/x5/wifi_diagnosis.md）：
#
# 1. 生成的配置没有 ctrl_interface，wpa_cli 一律 rc=255 —— 运行时**完全没法管理这个连接**，
#    board_netheal 的 `wpa_cli reassociate` 那一级只能是空操作，最便宜的修复手段（1~2 s）
#    因此不可用，只剩 ifconfig down/up（25 s），而实测掉线本身往往比它还短。
# 2. DEEP-RD 在 2.4G 上有三个 AP，客户端自由漫游，而只有一个是稳的：
#      ch1  a0:69:d9:5b:f0:13  -66 dBm  反复速率塌到 CCK 且 ping 不通
#      ch11 a0:69:d9:5b:f0:a3  -54 dBm  同上
#      ch6  a0:69:d9:5b:cc:13  -48 dBm  150/150 包 0% 丢包 72.2 Mbit/s
#    限制扫描频点比钉死 BSSID 安全：同信道换 AP 仍然允许。
#
# 频点从 /userdata/x5/wifi_freq_list 读（一行，空格分隔的 MHz），文件不存在就不限制。
set -euo pipefail
TARGET="${TARGET:-/etc/init.d/looper/wifi-connect.sh}"
MARK="# tinynav-wifi-patch"

[ -f "$TARGET" ] || { echo "找不到 $TARGET" >&2; exit 1; }
if grep -q "$MARK" "$TARGET"; then
    echo "已经打过补丁，不重复。当前生效的频点限制："
    cat /userdata/x5/wifi_freq_list 2>/dev/null || echo "  （无 /userdata/x5/wifi_freq_list，不限制）"
    exit 0
fi
[ -f "${TARGET}.orig" ] || cp -a "$TARGET" "${TARGET}.orig"

python3 - "$TARGET" "$MARK" <<'PY'
import sys
path, mark = sys.argv[1], sys.argv[2]
s = open(path).read()
anchor = 'wpa_passphrase "$SSID" "$PASSWORD" > "$WPA_CONF"'
assert anchor in s, "锚点变了，脚本被改过 —— 先人工看一遍"
add = anchor + f'''

{mark} begin
# 控制套接字：没有它 wpa_cli 一律连不上，运行时无法 reassociate / 换 AP。
if ! grep -q '^ctrl_interface=' "$WPA_CONF"; then
    sed -i '1i ctrl_interface=/var/run/wpa_supplicant' "$WPA_CONF"
fi
# 频点限制（可选）。文件不存在就保持默认的全频段漫游。
_FL="$(cat /userdata/x5/wifi_freq_list 2>/dev/null | tr -d '\\r' | head -1 || true)"
if [ -n "${{_FL:-}}" ]; then
    sed -i "/^network={{/a\\\\\\tfreq_list=${{_FL}}\\\\n\\\\tscan_freq=${{_FL}}" "$WPA_CONF"
fi
{mark} end'''
open(path, 'w').write(s.replace(anchor, add, 1))
print("已插入补丁")
PY
sh -n "$TARGET" && echo "语法检查通过"
echo "备份在 ${TARGET}.orig —— 回滚：cp ${TARGET}.orig ${TARGET}"
