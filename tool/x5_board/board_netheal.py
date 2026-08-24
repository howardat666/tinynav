#!/usr/bin/env python3
"""网络自愈:探不到网关就按梯度把 WiFi 踢回来，全过程记日志。由 board-netheal.service 拉起。

2026-08-24 同一天掉线两次，每次都得跑过去拔电。而 board_health 的数据显示掉线时
**`usb=1 carrier=1 link=1` 全程不变** —— 网卡没离开总线、association 也没断，塌的是
`rx_rate`(MCS7 -> CCK_1M)加 `fa` 误报警阶跃 7 倍。底层没坏，所以踢一下就该回来。

梯度从轻到重，每一级之后重新探测:轻的那几级不会打断正在跑的东西，重的才动网卡。
**每一级是否奏效都记进日志** —— 攒几次就知道哪一级才是真正有用的那一级，这是这个
服务除了自愈之外的第二个目的。

⚠️ 只在"曾经通过"之后才布防(`self.armed`)。开机时 wifi-connect.sh 本来就还没跑完，
那时候去自愈是和它抢。
⚠️ 默认**不重启**:app 不会自动起，重启等于把机器人放倒在那儿等人来。要开就设
NETHEAL_ALLOW_REBOOT=1。
"""
import os
import re
import socket
import struct
import subprocess
import sys
import time

PERIOD_S = 15.0
FAIL_N = 4                 # 连续失败这么多次才动手 = 60 s。实测 p95 RTT 839 ms、
                           # 最大连丢 2 次，所以这个门限很保守
TIMEOUT_S = 2.5
GRACE_S = 90.0             # 开机宽限，别和 wifi-connect.sh 抢
BACKOFF_S = 120.0          # 整条梯度都没救回来之后歇多久再重来
LOG = "/userdata/x5/logs/board_netheal.log"
MAXBYTES = 4 << 20
WIFI_UP = "/etc/init.d/looper/wifi-connect.sh"
USB_DEV = "/sys/bus/usb/devices/1-1"
WIFI_PROC = "/proc/net/rtl8710bu"
ALLOW_REBOOT = os.environ.get("NETHEAL_ALLOW_REBOOT") == "1"
DRY = "--dry-run" in sys.argv
# 真掉线才验梯度就太晚了。--force-fail 假装探测一直失败，配 --dry-run 就能把整条梯度和
# 日志走一遍而不真去动网卡（真动会把跑这条命令的 ssh 一起掐掉）。
FORCE_FAIL = "--force-fail" in sys.argv
PERIOD_S = float(os.environ.get("NETHEAL_PERIOD", PERIOD_S))


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def sh(cmd, timeout=90):
    """跑一条命令，返回 (rc, 输出尾部)。绝不抛异常 —— 自愈失败也得继续活着。"""
    if DRY:
        return 0, "(dry-run) " + cmd
    try:
        p = subprocess.run(cmd, shell=True, timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        return p.returncode, p.stdout.decode("utf-8", "replace")[-300:]
    except subprocess.TimeoutExpired:
        return -1, "timeout"
    except OSError as e:
        return -2, repr(e)


def iface():
    try:
        for n in sorted(os.listdir("/sys/class/net")):
            if n.startswith("wl"):
                return n
    except OSError:
        pass
    return None


def gateway():
    for line in read("/proc/net/route").splitlines()[1:]:
        f = line.split()
        if len(f) > 2 and f[1] == "00000000":
            return socket.inet_ntoa(struct.pack("<L", int(f[2], 16)))
    return None


def ipv4(dev):
    try:                       # 只读命令，干跑时也照读，否则快照里的 ip 是假的
        out = subprocess.run("ifconfig %s 2>/dev/null" % dev, shell=True, timeout=10,
                             stdout=subprocess.PIPE).stdout.decode("utf-8", "replace")
    except (OSError, subprocess.TimeoutExpired):
        return None
    m = re.search(r"inet (?:addr:)?(\d+\.\d+\.\d+\.\d+)", out)
    return m.group(1) if m else None


def _cksum(data):
    if len(data) % 2:
        data += b"\x00"
    total = sum(struct.unpack("!%dH" % (len(data) // 2), data))
    total = (total & 0xFFFF) + (total >> 16)
    return ~((total & 0xFFFF) + (total >> 16)) & 0xFFFF


def probe(host):
    """返回 (通不通, 说明)。说明里带 errno，便于分开队列堵死和路由丢失。"""
    if not host:
        return False, "no-gateway"
    pid = os.getpid() & 0xFFFF
    body = b"heal"
    pkt = struct.pack("!BBHHH", 8, 0,
                      _cksum(struct.pack("!BBHHH", 8, 0, 0, pid, 1) + body), pid, 1) + body
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
        s.settimeout(TIMEOUT_S)
        t0 = time.time()
        s.sendto(pkt, (host, 0))
        while time.time() - t0 < TIMEOUT_S:
            data, _ = s.recvfrom(1024)
            if len(data) >= 28 and data[20] == 0 and data[24:26] == struct.pack("!H", pid):
                return True, "%.0fms" % ((time.time() - t0) * 1000)
        return False, "MISS"
    except OSError as e:
        return False, "ERR%d" % (e.errno or 0)
    finally:
        if s is not None:
            s.close()


def snapshot(dev):
    """一行现场:和 board_health 用同样的字段，便于两份日志对照。"""
    out = []
    base = None
    try:
        for n in sorted(os.listdir(WIFI_PROC)):
            c = os.path.join(WIFI_PROC, n)
            if os.path.isdir(c) and os.path.exists(os.path.join(c, "rx_signal")):
                base = c
                break
    except OSError:
        pass
    if base:
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
                    if k.strip() == "is_linked":
                        out.append("link=%s" % v.strip())
            if "Total False Alarm" in line:
                out.append("fa=%s" % line.rsplit("=", 1)[1].strip())
        tp = read(os.path.join(base, "sta_tp_info"))
        for line in tp.splitlines():
            if "rx_rate :" in line:
                out.append("rx_rate=%s" % line.split("rx_rate :", 1)[1].split(",")[0].strip())
                break
    out.append("carrier=%s" % read("/sys/class/net/%s/carrier" % dev, "?").strip())
    out.append("usb=%d" % (1 if os.path.exists(os.path.join(USB_DEV, "idVendor")) else 0))
    out.append("ip=%s" % (ipv4(dev) or "none"))
    return " ".join(out)


def emit(line):
    print(line, flush=True)
    try:
        if os.path.exists(LOG) and os.path.getsize(LOG) > MAXBYTES:
            os.replace(LOG, LOG + ".1")
        with open(LOG, "a") as f:
            f.write("%s %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), line))
    except OSError:
        pass


def steps(dev):
    """梯度。每一项是 (名字, 动作, 之后等多少秒再复验)。"""
    return [
        ("reassociate", lambda: sh("wpa_cli -i %s reassociate" % dev, 20), 20),
        ("wifi-connect", lambda: sh("sh %s" % WIFI_UP, 120), 25),
        ("usb-reauthorize", lambda: (
            sh("echo 0 > %s/authorized; sleep 3; echo 1 > %s/authorized" % (USB_DEV, USB_DEV), 30),
            time.sleep(0 if DRY else 8),
            sh("sh %s" % WIFI_UP, 120))[0], 35),
    ]


def main():
    dev = iface()
    gw = gateway()
    emit("netheal 起动 iface=%s gw=%s 周期=%.0fs 门限=%d次 dry_run=%s allow_reboot=%s"
         % (dev, gw, PERIOD_S, FAIL_N, DRY, ALLOW_REBOOT))
    t_start = time.time()
    fails = 0
    armed = FORCE_FAIL
    rung = 0
    down_since = None
    backoff_until = 0.0
    while True:
        time.sleep(PERIOD_S)
        gw = gateway() or gw
        dev = iface() or dev
        ok, why = (False, "forced") if FORCE_FAIL else probe(gw)
        if ok:
            if down_since is not None:
                emit("恢复 断了%.0fs 由第%d级(%s)救回 | %s"
                     % (time.time() - down_since, rung,
                        steps(dev)[rung - 1][0] if 0 < rung <= len(steps(dev)) else "自愈",
                        snapshot(dev)))
            armed, fails, rung, down_since = True, 0, 0, None
            continue
        fails += 1
        if not armed:
            if time.time() - t_start > GRACE_S and fails % 4 == 1:
                emit("还没通过一次，不布防(开机宽限 %.0fs) %s=%s | %s"
                     % (GRACE_S, gw, why, snapshot(dev)))
            continue
        if down_since is None:
            down_since = time.time()
            emit("探测失败 %s=%s (第%d次，%d 次才动手) | %s" % (gw, why, fails, FAIL_N, snapshot(dev)))
        if fails < FAIL_N or time.time() < backoff_until:
            continue
        lad = steps(dev)
        if rung >= len(lad):
            if ALLOW_REBOOT:
                emit("整条梯度都没救回来，按配置重启 | %s" % snapshot(dev))
                sh("sync; systemctl reboot", 30)
                return
            emit("整条梯度都没救回来，歇 %.0fs 再从头试(要自动重启就设 "
                 "NETHEAL_ALLOW_REBOOT=1) | %s" % (BACKOFF_S, snapshot(dev)))
            rung, backoff_until = 0, time.time() + BACKOFF_S
            continue
        name, act, wait = lad[rung]
        rung += 1
        emit("第%d级 %s 开始 | %s" % (rung, name, snapshot(dev)))
        rc, out = act()
        emit("第%d级 %s 执行完 rc=%s 等%.0fs 复验 | %s"
             % (rung, name, rc, wait, out.strip().replace("\n", " / ")[-160:]))
        time.sleep(0 if DRY else wait)
        ok2, why2 = (False, "forced") if FORCE_FAIL else probe(gateway() or gw)
        emit("第%d级 %s 复验 %s (%s) | %s"
             % (rung, name, "通了" if ok2 else "还是不通", why2, snapshot(dev)))
        if ok2:
            fails = 0
        else:
            fails = FAIL_N        # 保持已触发状态，下一轮直接上下一级


if __name__ == "__main__":
    sys.exit(main())
