#!/usr/bin/env python3
"""在一个短时间窗内轮测 DEEP-RD 的每个 AP，避免"不同时段结果不同"的时间混淆。

第一版是 shell，三个 AP 全"拿不到 IP"、零数据，还把驱动搞死一小时。三个 bug：
① `pkill` 后没 `ip link set up`（wpa_supplicant 退出会把接口带 DOWN，新的必然关联失败）
② `sleep 6` 就 DHCP、从不检查关联  ③ **板上没有 `ping`**，测量部分本来就是空的。
这版还把"反复重启 supplicant"换成只重启一次 + `wpa_cli` 换 BSSID —— 反复拉起驱动
正是它卡死的原因。

用法（板上，会短暂断链，必须 detach）：
  nohup setsid python3 /userdata/x5/wifi_channel_ab.py >/userdata/x5/logs/wifi_ab.log 2>&1 &
"""
import os
import socket
import struct
import subprocess
import sys
import time

SSID = os.environ.get("AB_SSID", "DEEP-RD")
PASS = os.environ.get("AB_PASS", "07310731")
PC = os.environ.get("AB_PC", "192.168.19.51")
DWELL = float(os.environ.get("AB_DWELL", "30"))
GAP = float(os.environ.get("AB_GAP", "0.2"))
MAX_AP = int(os.environ.get("AB_MAX_AP", "6"))
WATCHDOG_MIN = int(os.environ.get("AB_WATCHDOG_MIN", "15"))
WIFI_UP = "/etc/init.d/looper/wifi-connect.sh"
NETHEAL_LOG = "/userdata/x5/logs/board_netheal.log"
OUT = "/userdata/x5/logs/wifi_ab_result.txt"
DONE = "/userdata/x5/logs/wifi_ab_done"
CONF = "/tmp/wpa_ab.conf"
WATCHDOG_UNIT = "wifi-ab-watchdog"


def log(msg):
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def sh(argv, timeout=40):
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except Exception as e:
        return -1, str(e)


def iface():
    for n in sorted(os.listdir("/sys/class/net")):
        if n.startswith("wl"):
            return n
    return None


IFACE = os.environ.get("AB_IFACE") or iface()


def wpa_cli(*args, timeout=15):
    rc, out = sh(["wpa_cli", "-i", IFACE] + list(args), timeout)
    return out.strip()


def link_freq():
    rc, out = sh(["iw", "dev", IFACE, "link"], 10)
    for line in out.splitlines():
        if "freq:" in line:
            return line.split("freq:")[1].strip().split()[0]
    return None


def link_bssid():
    rc, out = sh(["iw", "dev", IFACE, "link"], 10)
    for line in out.splitlines():
        if line.startswith("Connected to"):
            return line.split()[2].lower()
    return None


def have_ip():
    """用 ioctl 而不是 `ip -4 addr` —— 板上没有独立的 `ip`，只有 `busybox ip`，
    裸 `ip` 会 command-not-found 然后被当成"没有 IP"。固件的 wifi-connect.sh
    正是栽在这里，每次都误报 ERROR: no IP assigned。"""
    import fcntl
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        raw = fcntl.ioctl(s.fileno(), 0x8915,  # SIOCGIFADDR
                          struct.pack("256s", IFACE[:15].encode()))
        return socket.inet_ntoa(raw[20:24])
    except OSError:
        return None
    finally:
        s.close()


def radio():
    """复用 board_health 的取法，别重写一遍解析。"""
    try:
        sys.path.insert(0, "/usr/local/sbin")
        import importlib.util
        spec = importlib.util.spec_from_file_location("bh", "/usr/local/sbin/board_health.py")
        bh = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bh)
        return bh.wifi()
    except Exception as e:
        return "radio=unreadable(%s)" % e


# ---------------------------------------------------------------- ICMP
def _cksum(b):
    if len(b) % 2:
        b += b"\0"
    s = sum(struct.unpack("!%dH" % (len(b) // 2), b))
    s = (s >> 16) + (s & 0xFFFF)
    s += s >> 16
    return (~s) & 0xFFFF


def icmp_probe(host, duration, gap):
    """板上没有 ping，用原始套接字自己发。返回 (发, 收, rtt 列表 ms)。"""
    sent = 0
    rtts = []
    ident = os.getpid() & 0xFFFF
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    except PermissionError:
        return 0, 0, []
    s.settimeout(1.0)
    deadline = time.time() + duration
    seq = 0
    while time.time() < deadline:
        seq = (seq + 1) & 0xFFFF
        payload = struct.pack("!d", time.time()) + b"x" * 24
        hdr = struct.pack("!BBHHH", 8, 0, 0, ident, seq)
        pkt = struct.pack("!BBHHH", 8, 0, _cksum(hdr + payload), ident, seq) + payload
        t0 = time.time()
        try:
            s.sendto(pkt, (host, 0))
            sent += 1
        except OSError:
            sent += 1
            time.sleep(gap)
            continue
        # 只等这一个 seq，收到别的就继续等（同一 socket 上可能有上一轮的迟到包）
        while True:
            try:
                data, _ = s.recvfrom(2048)
            except socket.timeout:
                break
            if len(data) < 28:
                continue
            t, _, _, rid, rseq = struct.unpack("!BBHHH", data[20:28])
            if t == 0 and rid == ident and rseq == seq:
                rtts.append((time.time() - t0) * 1000.0)
                break
        left = gap - (time.time() - t0)
        if left > 0:
            time.sleep(left)
    s.close()
    return sent, len(rtts), rtts


def pct(v, q):
    if not v:
        return float("nan")
    v = sorted(v)
    i = min(len(v) - 1, max(0, int(round((len(v) - 1) * q))))
    return v[i]


# ---------------------------------------------------------------- 兜底
def arm_watchdog():
    """独立于本脚本的重启定时器。上一版把恢复挂在自己的 trap 上，trap 执行了、
    但恢复本身失败了 —— 兜底必须既不依赖脚本存活也不依赖它成功。"""
    if os.path.exists(DONE):
        os.remove(DONE)
    rc, out = sh(["systemd-run", "--on-active=%dmin" % WATCHDOG_MIN,
                  "--unit=%s" % WATCHDOG_UNIT, "--collect",
                  "/bin/sh", "-c",
                  "[ -f %s ] && exit 0; sync; systemctl reboot" % DONE], 20)
    log("兜底定时器 %d 分钟后无条件重启（除非写出 %s）rc=%d %s"
        % (WATCHDOG_MIN, DONE, rc, out.strip()[:120]))
    return rc == 0


def disarm_watchdog(ok):
    """🔴 恢复失败时**必须把定时器留着**。无条件 stop 会变成"既不写 DONE 也不重启"，
    板子就再躺一次 —— 和上一版 trap 那个 bug 同一形状。"""
    if not ok:
        log("恢复没成功，兜底定时器保留 —— 它会到点重启板子")
        return
    open(DONE, "w").write(time.strftime("%F %T\n"))
    sh(["systemctl", "stop", "%s.timer" % WATCHDOG_UNIT], 20)
    sh(["systemctl", "stop", "%s.service" % WATCHDOG_UNIT], 20)
    log("恢复成功，兜底定时器已撤除")


# ---------------------------------------------------------------- 建链
def wait_assoc(want=None, secs=20):
    for _ in range(int(secs)):
        b = link_bssid()
        if b and (want is None or b == want.lower()):
            return b
        time.sleep(1)
    return None


def rebuild_supplicant():
    """按固件那份能用的顺序重建一次，唯一的加料是 ctrl_interface（为了后面能用 wpa_cli）。"""
    rc, psk = sh(["wpa_passphrase", SSID, PASS], 20)
    if rc != 0 or "psk=" not in psk:
        log("wpa_passphrase 失败：%s" % psk[:200])
        return None
    with open(CONF, "w") as f:
        f.write("ctrl_interface=/var/run/wpa_supplicant\n")
        f.write(psk)
    os.chmod(CONF, 0o600)
    sh(["pkill", "-f", "wpa_supplicant.*-i *%s" % IFACE], 20)
    sh(["pkill", "-f", "udhcpc.*-i *%s" % IFACE], 20)
    time.sleep(1)
    sh(["busybox", "ip", "link", "set", IFACE, "up"], 20)  # 上一版缺这行，且 `ip` 只在 busybox 里
    time.sleep(1)
    rc, out = sh(["wpa_supplicant", "-B", "-i", IFACE, "-c", CONF], 30)
    if rc != 0:
        log("wpa_supplicant 起不来 rc=%d %s" % (rc, out[:200]))
        return None
    b = wait_assoc(None, 20)
    if not b:
        log("20s 内没关联上")
        return None
    sh(["udhcpc", "-i", IFACE, "-t", "10", "-T", "3", "-n", "-q"], 60)
    ip = have_ip()
    log("重建完成 bssid=%s ip=%s" % (b, ip))
    return ip


def restore():
    """先用最轻的手段（清掉 bssid 锁定再重关联），不行才动固件脚本。"""
    wpa_cli("set_network", "0", "bssid", "00:00:00:00:00:00")
    wpa_cli("reassociate")
    if wait_assoc(None, 20):
        if not have_ip():
            sh(["udhcpc", "-i", IFACE, "-t", "10", "-T", "3", "-n", "-q"], 60)
        if have_ip() and reachable():
            log("恢复完成（wpa_cli 路径）ip=%s" % have_ip())
            return True
    log("wpa_cli 恢复失败，回落到 %s" % WIFI_UP)
    for attempt in (1, 2):
        rc, out = sh(["sh", WIFI_UP], 120)
        if have_ip() and reachable():
            log("恢复完成（固件脚本，第 %d 次）ip=%s" % (attempt, have_ip()))
            return True
        log("第 %d 次失败 rc=%d %s" % (attempt, rc, out.strip()[-160:]))
    return False


def reachable():
    sent, got, _ = icmp_probe(PC, 3.0, 0.5)
    return got > 0


# ---------------------------------------------------------------- AP 枚举
def scan_aps():
    seen = {}
    for i in range(3):
        wpa_cli("scan")
        time.sleep(4)
        out = wpa_cli("scan_results", timeout=20)
        for line in out.splitlines():
            p = line.split("\t")
            if len(p) >= 5 and p[4] == SSID:
                try:
                    seen.setdefault(p[0].lower(), (int(p[1]), int(p[2])))
                except ValueError:
                    pass
        log("第 %d 轮扫描后累计 %d 个 AP" % (i + 1, len(seen)))
    # iw/wpa 的扫描会漏 —— 上次我扫到 3 个，netheal 同时段实测漫游到 5 个。
    try:
        with open(NETHEAL_LOG, "rb") as f:
            blob = f.read()[-2000000:].decode("utf-8", "replace")
        import re
        for m in set(re.findall(r"bssid=([0-9a-f:]{17})", blob)):
            if m not in seen:
                seen[m] = (0, 0)
                log("从 netheal 历史日志补上漏扫的 %s" % m)
    except OSError:
        pass
    return seen


# ---------------------------------------------------------------- main
def preflight():
    """每个外部命令都必须真的能跑起来。板上 `ip` 不存在、`ping` 不存在,
    而 sh() 把 FileNotFoundError 吞成 rc=-1 —— 不检查就会静默走完全流程拿到空数据。"""
    need = [["busybox", "ip", "link", "show", IFACE], ["iw", "dev", IFACE, "link"],
            ["pkill", "--version"], ["wpa_cli", "-v"], ["wpa_supplicant", "-v"],
            ["wpa_passphrase"], ["udhcpc", "--help"], ["systemctl", "--version"],
            ["systemd-run", "--version"]]
    bad = []
    for argv in need:
        rc, out = sh(argv, 15)
        if rc == -1 and "No such file" in out:   # sh() 把 FileNotFoundError 变成这个
            bad.append(argv[0] if argv[0] != "busybox" else "busybox ip")
    if have_ip() is None:
        bad.append("have_ip()=None（测前就该有 IP，说明取法又错了）")
    return bad


def main():
    if IFACE is None:
        log("找不到 wl* 接口，退出")
        return 1
    missing = preflight()
    if missing:
        log("🔴 缺少命令 %s —— 拒绝开跑（上一版就是把不存在的命令当成执行失败了）"
            % ", ".join(missing))
        return 1
    log("接口=%s SSID=%s 目标=%s 每个 AP 测 %.0fs" % (IFACE, SSID, PC, DWELL))
    log("测前状态：bssid=%s ip=%s %s" % (link_bssid(), have_ip(), radio()))
    arm_watchdog()

    ok = False
    rows = []
    try:
        sh(["systemctl", "stop", "board-netheal"], 30)
        log("已暂停 board-netheal（否则它会在测试期间插手，污染结果）")

        if rebuild_supplicant() is None:
            log("重建失败，直接进恢复")
            return 1

        aps = scan_aps()
        order = sorted(aps.items(), key=lambda kv: -kv[1][1])[:MAX_AP]
        log("要测 %d 个：%s" % (len(order), ", ".join(b for b, _ in order)))

        for bssid, (freq, sig) in order:
            log("=== 切到 %s (扫描时 freq=%s sig=%s)" % (bssid, freq or "?", sig or "?"))
            wpa_cli("set_network", "0", "bssid", bssid)
            wpa_cli("reassociate")
            got = wait_assoc(bssid, 25)
            if got != bssid:
                log("  关联不上（现在在 %s），跳过" % got)
                rows.append((bssid, freq, "-", "-", "-", "-", "-", "关联不上"))
                continue
            ip = have_ip()
            if not ip:
                sh(["udhcpc", "-i", IFACE, "-t", "10", "-T", "3", "-n", "-q"], 60)
                ip = have_ip()
            if not ip:
                log("  关联上了但拿不到 IP，跳过")
                rows.append((bssid, freq, "-", "-", "-", "-", "-", "无 IP"))
                continue
            freq = link_freq() or freq      # 实测频点优先于扫描时的值
            r0 = radio()
            sent, got_n, rtts = icmp_probe(PC, DWELL, GAP)
            r1 = radio()
            loss = 100.0 * (sent - got_n) / sent if sent else float("nan")
            rows.append((bssid, freq,
                         "%.1f%%" % loss,
                         "%.0f" % pct(rtts, 0.5) if rtts else "-",
                         "%.0f" % pct(rtts, 0.95) if rtts else "-",
                         "%.0f" % max(rtts) if rtts else "-",
                         "%d/%d" % (got_n, sent), r1))
            log("  丢包 %.1f%%  p50 %s ms  p95 %s ms  max %s ms"
                % (loss,
                   "%.0f" % pct(rtts, 0.5) if rtts else "-",
                   "%.0f" % pct(rtts, 0.95) if rtts else "-",
                   "%.0f" % max(rtts) if rtts else "-"))
            log("  测前 %s" % r0)
            log("  测后 %s" % r1)
        ok = True
    finally:
        with open(OUT, "w") as f:
            f.write("# %s  每个 AP 测 %.0fs，%.1fs 一个包，目标 %s\n"
                    % (time.strftime("%F %T"), DWELL, GAP, PC))
            f.write("%-19s %-6s %-8s %-8s %-8s %-8s %-10s %s\n"
                    % ("BSSID", "freq", "丢包", "p50ms", "p95ms", "maxms", "收/发", "射频(测后)"))
            for r in rows:
                f.write("%-19s %-6s %-8s %-8s %-8s %-8s %-10s %s\n"
                        % (r[0], r[1] or "?", r[2], r[3], r[4], r[5], r[6], r[7]))
        log("=== 结果写入 %s ===" % OUT)
        print(open(OUT).read(), flush=True)
        restored = restore()
        sh(["systemctl", "start", "board-netheal"], 30)
        log("board-netheal 已恢复")
        disarm_watchdog(restored)
        if not restored:
            log("🔴 恢复失败 —— 兜底定时器会重启板子，别手动干预")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
