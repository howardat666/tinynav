#!/usr/bin/env python3
"""每 10 秒往 journald 记一行板级健康快照。在板上由 board-health.service 拉起。

2026-08-21 板子掉线过一次,事后查不出原因 —— journal 里只有 systemd 和内核消息,
**没有任何"当时链路和电压什么样"的时间序列**,而那正是唯一能区分"WiFi 掉了"和
"板子死了"的东西。这个服务就是补这一条。

写 stdout 而不是自己的文件:systemd 会收进 journal,和内核/驱动消息**按时间穿插在一起**,
排查时不用对两份时间戳;而且 journald 自带 64MB 上限的轮转,不用自己写清理。

⚠️ 判据速查(下次掉线时看这一行的走向):
  wifi=down 或 rssi 骤降        -> 空口/射频
  wifi 正常但 pc=MISS 连续多行  -> 上游网络(AP/PC),板子自己还活着
  整段日志断裂 + 下一行是开机   -> 板子重启了(掉电/panic;panic 看 /sys/fs/pstore)
  temp 接近 95(度)             -> 降频,不是掉线但会拖慢一切
"""
import os
import socket
import struct
import sys
import time

PERIOD_S = 10.0
WIFI_DIR = "/proc/net/rtl8710bu"
PC = os.environ.get("HEALTH_PING_HOST", "192.168.19.51")


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def wifi():
    # listdir()[0] 不行：这个目录里混着一堆驱动级的普通文件(chplan_id_list 之类)，
    # 接口目录才是我们要的，判据是它下面有 rx_signal
    base = None
    try:
        for name in sorted(os.listdir(WIFI_DIR)):
            cand = os.path.join(WIFI_DIR, name)
            if os.path.isdir(cand) and os.path.exists(os.path.join(cand, "rx_signal")):
                base = cand
                break
    except OSError:
        pass
    if base is None:
        return "wifi=none"
    out = []
    sig = read(os.path.join(base, "rx_signal"))
    for key, tag in (("rssi:", "rssi"), ("signal_qual:", "qual")):
        for line in sig.splitlines():
            if line.startswith(key):
                out.append("%s=%s" % (tag, line.split(":", 1)[1].strip()))
    dbg = read(os.path.join(base, "trx_info_debug"))
    for line in dbg.splitlines():
        if "is_linked" in line:
            for part in line.split(","):
                k, _, v = part.partition("=")
                k = k.strip()
                if k in ("is_linked", "current_igi"):
                    out.append("%s=%s" % ("link" if k == "is_linked" else "igi", v.strip()))
        if "Total False Alarm" in line:
            out.append("fa=%s" % line.rsplit("=", 1)[1].strip())
    tp = read(os.path.join(base, "sta_tp_info"))
    for line in tp.splitlines():
        if "rx_rate :" in line:
            out.append("rx_rate=%s" % line.split("rx_rate :", 1)[1].split(",")[0].strip())
            break
    return " ".join(out) if out else "wifi=unreadable"


def _cksum(data):
    if len(data) % 2:
        data += b"\x00"
    total = sum(struct.unpack("!%dH" % (len(data) // 2), data))
    total = (total & 0xFFFF) + (total >> 16)
    return ~((total & 0xFFFF) + (total >> 16)) & 0xFFFF


def ping_ms(host, timeout=2.5):
    """超时 2.5s 是量出来的:这条链路 p95 RTT 839ms、max 991ms,1s 会把"慢"误判成"丢"。"""
    pid = os.getpid() & 0xFFFF
    body = b"health"
    pkt = struct.pack("!BBHHH", 8, 0, _cksum(struct.pack("!BBHHH", 8, 0, 0, pid, 1) + body),
                      pid, 1) + body
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
        s.settimeout(timeout)
        t0 = time.time()
        s.sendto(pkt, (host, 0))
        while time.time() - t0 < timeout:
            data, _ = s.recvfrom(1024)
            if len(data) >= 28 and data[20] == 0 and data[24:26] == struct.pack("!H", pid):
                return "%.0f" % ((time.time() - t0) * 1000)
        return "MISS"
    except OSError:
        return "ERR"
    finally:
        if s is not None:
            s.close()


def sysline():
    out = []
    up = read("/proc/uptime").split()
    if up:
        out.append("up=%.0f" % float(up[0]))
    la = read("/proc/loadavg").split()
    if la:
        out.append("load=%s" % la[0])
    for line in read("/proc/meminfo").splitlines():
        if line.startswith("MemAvailable:"):
            out.append("memavail=%dM" % (int(line.split()[1]) // 1024))
    temps = []
    for i in range(4):
        t = read("/sys/class/thermal/thermal_zone%d/temp" % i).strip()
        if t.isdigit():
            temps.append("%.1f" % (int(t) / 1000.0))
    if temps:
        out.append("temp=%s" % "/".join(temps))
    return " ".join(out)


def main():
    print("board_health 起动, 每 %.0fs 一行, ping %s" % (PERIOD_S, PC), flush=True)
    while True:
        try:
            print("%s %s pc=%s" % (sysline(), wifi(), ping_ms(PC)), flush=True)
        except Exception as e:                                        # noqa: BLE001
            # 绝不因为一次读失败而退出 —— 这个服务的价值全在"连续"上
            print("health sample failed: %r" % (e,), flush=True)
        time.sleep(PERIOD_S)


if __name__ == "__main__":
    sys.exit(main())
