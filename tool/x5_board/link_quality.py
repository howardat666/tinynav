#!/usr/bin/env python3
"""板子自己测自己的链路质量，写成 TSV。用于**出门在外 PC 够不着板子**的场合。

跑法（板上，后台）：
    setsid nohup python3 /userdata/x5/tinynav/tool/x5_board/link_quality.py > /dev/null 2>&1 &
回来之后在 PC 上：
    python3 tool/x5_board/link_quality.py --report <拉下来的 tsv>

测的是**板子到当前默认网关**这一段，也就是「板子 → USB → ESP32 → WiFi → AP」整条链路。
换网时网关会自己变（每轮重读 /proc/net/route），所以中途切 AP 也不用重启它。
"""
import os
import re
import socket
import struct
import subprocess
import sys
import time

OUT = os.environ.get("LQ_OUT", "/userdata/x5/logs/link_quality.tsv")
PERIOD_S = float(os.environ.get("LQ_PERIOD", "30"))
NPING = int(os.environ.get("LQ_NPING", "20"))
COLS = ["时刻", "网卡", "ifindex", "本机IP", "网关", "发", "收", "丢%", "p50ms", "p90ms", "最大ms"]


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def iface():
    try:
        for n in sorted(os.listdir("/sys/class/net")):
            if n.startswith("enx"):
                return n
        for n in sorted(os.listdir("/sys/class/net")):
            if n.startswith("wl"):
                return n
    except OSError:
        pass
    return None


def gateway():
    import socket
    import struct
    for line in read("/proc/net/route").splitlines()[1:]:
        f = line.split()
        if len(f) > 2 and f[1] == "00000000":
            return socket.inet_ntoa(struct.pack("<L", int(f[2], 16)))
    return None


def ipv4(dev):
    out = subprocess.run("ifconfig %s 2>/dev/null" % dev, shell=True,
                         stdout=subprocess.PIPE).stdout.decode("utf-8", "replace")
    m = re.search(r"inet (?:addr:)?(\d+\.\d+\.\d+\.\d+)", out)
    return m.group(1) if m else None


TIMEOUT_S = 2.0


def _cksum(data):
    if len(data) % 2:
        data += b"\x00"
    total = sum(struct.unpack("!%dH" % (len(data) // 2), data))
    total = (total & 0xFFFF) + (total >> 16)
    return ~((total & 0xFFFF) + (total >> 16)) & 0xFFFF


def ping_once(host, seq):
    """🔴 板上没有 `ping` 命令（精简系统），只能自己发 ICMP。做法抄自 board_netheal.probe。
    返回毫秒，不通返回 None。"""
    pid = os.getpid() & 0xFFFF
    body = b"linkq"
    head = struct.pack("!BBHHH", 8, 0, 0, pid, seq)
    pkt = struct.pack("!BBHHH", 8, 0, _cksum(head + body), pid, seq) + body
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
        s.settimeout(TIMEOUT_S)
        t0 = time.time()
        s.sendto(pkt, (host, 0))
        while time.time() - t0 < TIMEOUT_S:
            data, _ = s.recvfrom(1024)
            # 20 字节 IP 头；type=0 是 echo reply；核对 id 和 seq，否则会收到别人的包
            if (len(data) >= 28 and data[20] == 0
                    and data[24:28] == struct.pack("!HH", pid, seq)):
                return (time.time() - t0) * 1000.0
        return None
    except (socket.timeout, OSError):
        return None
    finally:
        if s is not None:
            s.close()


def measure(gw, n):
    """返回 (发, 收, [rtt...])。"""
    rtt = []
    for i in range(n):
        v = ping_once(gw, i + 1)
        if v is not None:
            rtt.append(v)
        time.sleep(0.3)
    return n, len(rtt), rtt


def pct(v, p):
    if not v:
        return ""
    v = sorted(v)
    k = min(len(v) - 1, int(round((len(v) - 1) * p / 100.0)))
    return "%.1f" % v[k]


def sample():
    dev = iface()
    gw = gateway()
    idx = read("/sys/class/net/%s/ifindex" % dev).strip() if dev else ""
    ip = ipv4(dev) if dev else None
    if not gw:
        return [time.strftime("%H:%M:%S"), dev or "-", idx or "-", ip or "-", "-", "0", "0", "", "", "", ""]
    sent, got, rtt = measure(gw, NPING)
    loss = "%.1f" % (100.0 * (1 - got / float(sent))) if sent else ""
    return [time.strftime("%H:%M:%S"), dev or "-", idx or "-", ip or "-", gw,
            str(sent), str(got), loss, pct(rtt, 50), pct(rtt, 90),
            ("%.0f" % max(rtt)) if rtt else ""]


def run():
    new = not os.path.exists(OUT)
    with open(OUT, "a") as f:
        if new:
            f.write("\t".join(COLS) + "\n")
        while True:
            try:
                row = sample()
            except Exception as e:                      # noqa: BLE001 采集器不能自己死
                row = [time.strftime("%H:%M:%S"), "ERR", repr(e)[:60]] + [""] * 8
            f.write("\t".join(row) + "\n")
            f.flush()
            os.fsync(f.fileno())   # 断电会吃掉缓冲区里的行，而那几行正是判因用的
            time.sleep(PERIOD_S)


def report(path):
    rows = [l.rstrip("\n").split("\t") for l in open(path) if l.strip()]
    if len(rows) < 2:
        print("没有数据")
        return
    body = [r for r in rows[1:] if len(r) >= 11]
    print("样本 %d 条，覆盖 %s ~ %s\n" % (len(body), body[0][0], body[-1][0]))
    # 按「本机IP + 网关」分段：换网就是新的一段
    segs = []
    for r in body:
        key = (r[3], r[4])
        if not segs or segs[-1][0] != key:
            segs.append([key, []])
        segs[-1][1].append(r)
    for (ip, gw), rs in segs:
        loss = [float(r[7]) for r in rs if r[7]]
        p50 = [float(r[8]) for r in rs if r[8]]
        p90 = [float(r[9]) for r in rs if r[9]]
        dead = sum(1 for r in rs if r[7] == "100.0")
        print("ip=%-15s 网关=%-15s  %s~%s  %d 次采样" % (ip, gw, rs[0][0], rs[-1][0], len(rs)))
        if loss:
            print("   丢包 平均%.1f%% 最差%.1f%%   全丢的采样 %d 次(=%.0f 秒断流)"
                  % (sum(loss) / len(loss), max(loss), dead, dead * PERIOD_S))
        if p50:
            print("   延迟 p50 中位%.0fms 最差%.0fms | p90 中位%.0fms 最差%.0fms"
                  % (sorted(p50)[len(p50) // 2], max(p50),
                     sorted(p90)[len(p90) // 2], max(p90)))
        idxs = sorted({r[2] for r in rs})
        if len(idxs) > 1:
            print("   ⚠️ 期间网卡 ifindex 变过: %s（网卡消失过）" % ",".join(idxs))
        print()


if __name__ == "__main__":
    if "--report" in sys.argv:
        report(sys.argv[sys.argv.index("--report") + 1])
    else:
        run()
