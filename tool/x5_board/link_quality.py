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
# 🔴 丢包率在手机热点上是**假数**：安卓热点的网关根本不回 ICMP（2026-09-22 实测，
# 网桥的 ICMP 对帐里几十个请求只换回 3 个回复，而同期数据流了几 MB）。所以必须同时记
# 「数据有没有在流」和「网关在不在 ARP 表里」，否则报表会把一条好链路判成全断。
COLS = ["时刻", "网卡", "ifindex", "本机IP", "网关", "gwARP",
        "发", "收", "丢%", "p50ms", "p90ms", "最大ms", "收包增", "发包增", "收字节增"]


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


def netstat(dev, k):
    v = read("/sys/class/net/%s/statistics/%s" % (dev, k)).strip()
    return int(v) if v.isdigit() else None


def arp_complete(gw):
    """网关在不在 ARP 表里且已解析。ARP 由对方协议栈回，和它回不回 ICMP 无关 ——
    热点网关不回 ping 但一定回 ARP，所以这是比丢包率可靠得多的"对方还在"判据。"""
    for line in read("/proc/net/arp").splitlines()[1:]:
        f = line.split()
        if len(f) >= 4 and f[0] == gw:
            try:
                return "是" if (int(f[2], 16) & 0x2) and f[3] != "00:00:00:00:00:00" else "否"
            except ValueError:
                return "?"
    return "无"


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


_PREV = {}


def sample():
    dev = iface()
    gw = gateway()
    idx = read("/sys/class/net/%s/ifindex" % dev).strip() if dev else ""
    ip = ipv4(dev) if dev else None
    d = ["", "", ""]
    if dev:
        cur = [netstat(dev, "rx_packets"), netstat(dev, "tx_packets"), netstat(dev, "rx_bytes")]
        old = _PREV.get(idx)          # 按 ifindex 存：网卡重建过就不做差，否则算出个巨大的假增量
        if old and all(x is not None for x in cur) and all(x is not None for x in old):
            d = [str(max(0, c - o)) for c, o in zip(cur, old)]
        _PREV.clear()
        _PREV[idx] = cur
    if not gw:
        return ([time.strftime("%H:%M:%S"), dev or "-", idx or "-", ip or "-", "-", "无",
                 "0", "0", "", "", "", ""] + d)
    sent, got, rtt = measure(gw, NPING)
    loss = "%.1f" % (100.0 * (1 - got / float(sent))) if sent else ""
    return ([time.strftime("%H:%M:%S"), dev or "-", idx or "-", ip or "-", gw, arp_complete(gw),
             str(sent), str(got), loss, pct(rtt, 50), pct(rtt, 90),
             ("%.0f" % max(rtt)) if rtt else ""] + d)


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
    body = [r for r in rows[1:] if len(r) >= 12]
    # 采样周期按时间戳现算，不能用模块里的默认值 —— 采集时可能用了别的 LQ_PERIOD，
    # 报表印一个错的周期会让"多少秒断流"整个算错。
    def _sec(x):
        h, m, sec = (int(v) for v in x.split(":"))
        return h * 3600 + m * 60 + sec
    gaps = [(_sec(body[i][0]) - _sec(body[i - 1][0])) % 86400 for i in range(1, len(body))]
    step = sorted(gaps)[len(gaps) // 2] if gaps else PERIOD_S
    print("样本 %d 条，覆盖 %s ~ %s\n" % (len(body), body[0][0], body[-1][0]))
    # 按「本机IP + 网关」分段：换网就是新的一段
    segs = []
    for r in body:
        key = (r[3], r[4])
        if not segs or segs[-1][0] != key:
            segs.append([key, []])
        segs[-1][1].append(r)
    for (ip, gw), rs in segs:
        num = lambda i: [float(r[i]) for r in rs if len(r) > i and r[i]]
        loss, p50, p90 = num(8), num(9), num(10)
        rxp, txp, rxb = num(12), num(13), num(14)
        dead = sum(1 for r in rs if r[8] == "100.0")
        print("ip=%-15s 网关=%-15s  %s~%s  %d 次采样" % (ip, gw, rs[0][0], rs[-1][0], len(rs)))
        # 先说"链路活没活"，再说丢包 —— 顺序是故意的：热点网关不回 ICMP，丢包率会是 100%
        # 而链路完全正常，先看这一行才不会被那个数字带偏。
        if rxp:
            silent = sum(1 for v in rxp if v == 0)
            print("   数据流 每%.0fs 收%.0f包/发%.0f包/%.1fKB(中位)   一个包都没收到的采样 %d 次(=%.0f 秒真静默)"
                  % (step, sorted(rxp)[len(rxp) // 2], sorted(txp)[len(txp) // 2] if txp else 0,
                     (sorted(rxb)[len(rxb) // 2] / 1024.0) if rxb else 0, silent, silent * step))
        arps = [r[5] for r in rs if len(r) > 5]
        if arps:
            bad = sum(1 for a in arps if a != "是")
            print("   网关ARP 已解析 %d/%d 次%s" % (len(arps) - bad, len(arps),
                  "" if not bad else "  ⚠️ 有 %d 次没解析到，那才是真的够不着网关" % bad))
        if loss:
            note = "（网关不回 ICMP 时这一列无意义，以上面两行为准）" if (rxp and sum(rxp) > 0
                    and sum(loss) / len(loss) > 90) else ""
            print("   ICMP 丢包 平均%.1f%% 最差%.1f%%   全丢 %d 次 %s"
                  % (sum(loss) / len(loss), max(loss), dead, note))
        if p50:
            print("   延迟 p50 中位%.0fms 最差%.0fms | p90 中位%.0fms 最差%.0fms"
                  % (sorted(p50)[len(p50) // 2], max(p50),
                     sorted(p90)[len(p90) // 2], max(p90)))
        idxs = sorted({r[2] for r in rs})
        if len(idxs) > 1:
            print("   ⚠️ 期间网卡 ifindex 变过: %s（USB 网卡整个消失过，不是变慢）" % ",".join(idxs))
        print()


if __name__ == "__main__":
    if "--report" in sys.argv:
        i = sys.argv.index("--report") + 1
        # 不传路径就用自己写的那个，别抛 IndexError —— 排查时最常用的就是"报告一下"
        report(sys.argv[i] if i < len(sys.argv) else OUT)
    else:
        run()
