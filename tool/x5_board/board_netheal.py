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
FAIL_N = 6                 # 连续失败这么多次才动手 = 90 s。2026-08-24 见过一次 53 s 的掉线
                           # 自己好了，而第2级的拆重建反而把"关联着但过不了流量"打成
                           # "连都连不上"（association failed after 20s）—— 宁可多等
TIMEOUT_S = 2.5
GRACE_S = 90.0             # 开机宽限，别和 wifi-connect.sh 抢
BACKOFF_S = 120.0          # 整条梯度都没救回来之后歇多久再重来
NOGW_GRACE = 4             # 连续这么多轮没有默认路由（=60s）才补跑 dhcp，躲开开机时的枚举抖动
NOGW_COOLDOWN_S = 120.0    # 补跑之间的最小间隔
NOIP_COOLDOWN_S = 45.0     # "根本没有地址"这一类的补救间隔
PENDING_MAX = 4            # 网卡重建后最多等这么多轮 carrier，超了就走正常梯度，别卡死
RX_ALIVE_PKTS = 5          # 一个周期内收到这么多包就算"数据在流"（配合 ARP 判据一起用）
LOG = "/userdata/x5/logs/board_netheal.log"
MAXBYTES = 4 << 20
WIFI_UP = "/etc/init.d/looper/wifi-connect.sh"
USB_DEV = "/sys/bus/usb/devices/1-1"
WIFI_PROC = "/proc/net/rtl8710bu"
# 固定副地址：换到手机热点后 DHCP 每次给的地址都不同，人就得去热点的设备列表里找。
# 🔴 默认关闭。2026-09-21 实测这条路在本机走不通：网卡名 enxa4cb8fd549dc 正好 15 字符，
# 已顶满 IFNAMSIZ，`ifconfig <dev>:1` 的别名名字超长被截回主接口 —— 命令不是"加副地址"
# 而是"把主地址改掉"，板子当场从 192.168.19.102 掉到这个地址上失联。板上没有 `ip` 命令，
# 没有别的加副地址的办法。要重开必须先解决名字长度（比如把网卡改名成短名）。
PIN_IP = os.environ.get("NETHEAL_PIN_IP", "")
PIN_MASK = os.environ.get("NETHEAL_PIN_MASK", "255.255.255.0")
IFNAMSIZ_MAX = 15          # 内核 IFNAMSIZ=16 含结尾 NUL
# 续租时主动请求约定地址（DHCP 的 requested-ip，不是静态配置，也不是别名）。键是 /24 前缀。
# 🔴 **只对认这个选项的服务器有用，办公网不认**。2026-09-22 实测 DEEP-RD：请求 .102 时
#    服务器直接 select 回旧租约 .42；换 -C 当新客户端后仍从池子里发 .125。给它配一条的
#    代价是每次开机白跑一次续租、期间网卡 deconfigured 断 20~40 秒，纯亏。
#    实验室网靠 board_mdns.py 播报 looper.local 解决，不靠固定地址。
REQUEST_IPS = {}
for _a in os.environ.get("NETHEAL_REQUEST_IPS", "10.140.21.9").split(","):
    _a = _a.strip()
    if _a:
        REQUEST_IPS[_a.rsplit(".", 1)[0]] = _a
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


def sh(cmd, timeout=90, quiet=False):
    """跑一条命令，返回 (rc, 输出尾部)。绝不抛异常 —— 自愈失败也得继续活着。

    🔴 rc 非零必须自己叫出来。原来 rc 只进日志字符串、从不作为判据，于是 dhcp-renew
    连续 34 次 rc=-15（pkill 打死了自己那条 shell）淹在 INFO 里没人看见，那一级从来
    没真跑起来过。负 rc = 被信号打死，-15 是 SIGTERM，多半是自杀。"""
    if DRY:
        return 0, "(dry-run) " + cmd
    try:
        p = subprocess.run(cmd, shell=True, timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        rc, out = p.returncode, p.stdout.decode("utf-8", "replace")[-300:]
    except subprocess.TimeoutExpired:
        rc, out = -1, "timeout"
    except OSError as e:
        rc, out = -2, repr(e)
    if rc != 0 and not quiet:
        hint = ""
        if rc == -15:
            hint = "（SIGTERM：这条命令很可能被自己的 pkill 打死了）"
        elif rc < 0:
            hint = "（被信号 %d 打死）" % (-rc)
        emit("ERROR 命令失败 rc=%s%s cmd=%s | %s" % (rc, hint, cmd, out.strip()[-160:]))
    return rc, out


def iface():
    """🔴 不能只认 wl*：USB WiFi 网卡正是本项目要删掉的东西，现在的出口是 ESP32 网桥的 enx*。
    只找 wl* 会让这里恒为 None，main 每轮直接 continue —— 整个自愈服务空转、从不动手。
    2026-09-17 每次链路断死都只能人工断电重启，根因就是这一行。"""
    try:
        names = sorted(os.listdir("/sys/class/net"))
        for n in names:
            if n.startswith("enx"):
                return n
        for n in names:
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


def ensure_pinned(dev):
    """保证网卡上挂着 PIN_IP 这个固定副地址。板上没有 `ip` 命令，只能用 busybox 的别名接口。
    每轮都查是因为 USB 重新枚举、ifconfig down/up、dhcp-renew 都会把别名冲掉。"""
    if not PIN_IP or not dev:
        return
    alias = "%s:1" % dev
    # 🔴 前置断言，不是事后读回。内核接口名上限 IFNAMSIZ-1 = 15 字节，别名名超了会被
    # 截回主接口 —— 命令从"加副地址"变成"改主地址"，板子当场失联，而 ifconfig 返回 rc=0。
    # 读回来自检也没用：读的时候用的是同一个被截断的名字，自检必然和动作同谋。
    if len(alias) > IFNAMSIZ_MAX:
        emit("ERROR 固定副地址已停用：别名 %s 长 %d 字节 > %d，会被截回主接口把板子改失联"
             % (alias, len(alias), IFNAMSIZ_MAX))
        return
    if ipv4(alias) == PIN_IP:
        return
    rc, out = sh("ifconfig %s %s netmask %s up" % (alias, PIN_IP, PIN_MASK), 20)
    emit("挂固定副地址 %s=%s rc=%s %s" % (alias, PIN_IP, rc, out.strip()[-100:]))


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
    except socket.timeout:
        return False, "MISS"      # 见 board_health.py 里的注释:超时不是发送失败
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
        # 掉线时挂在哪个 AP 上 -- 板子在几个 AP 间漫游，2026-08-25 见过一次掉线正发生在
        # 漫游到 -63 dBm 的弱 AP 之后（正常那个是 -51）。攒几次就知道是不是规律。
        ap = read(os.path.join(base, "ap_info"))
        for line in ap.splitlines():
            if "macaddr" in line and ":" in line:
                out.append("bssid=%s" % line.split(":", 1)[1].strip())
            elif "cur_channel=" in line:
                out.append("ch=%s" % line.split("cur_channel=", 1)[1].split(",")[0].strip())

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


def usb_reauthorize():
    p = usb_dev_path() or USB_DEV
    return sh("echo 0 > %s/authorized; sleep 3; echo 1 > %s/authorized" % (p, p), 30)


def ifindex(dev):
    """网卡的内核序号。换 AP 时 USB 网卡会整个消失再出现，序号随之变化 —— 这是
    "网络被重新插过"的确定性信号，比等网关探测失败 6 次(90s)快得多且不会误判。"""
    if not dev:
        return None
    v = read("/sys/class/net/%s/ifindex" % dev).strip()
    return v or None


def usb_dev_path():
    """USB 设备目录。原来写死 /sys/bus/usb/devices/1-1，这块板子上它不存在 ——
    2026-09-21 日志里第 3 级 usb-reauthorize 报 rc=2 'Directory nonexistent'，
    那一级从来没真执行过。改成按网卡反查：net 设备的 device 链接指向 USB 接口目录
    (形如 1-1:1.0)，它的上一级就是 USB 设备目录。"""
    dev = iface()
    if not dev:
        return None
    try:
        intf = os.path.realpath("/sys/class/net/%s/device" % dev)
    except OSError:
        return None
    parent = os.path.dirname(intf)
    return parent if os.path.exists(os.path.join(parent, "authorized")) else None


def rx_packets(dev):
    v = read("/sys/class/net/%s/statistics/rx_packets" % dev).strip()
    return int(v) if v.isdigit() else None


def arp_complete(gw):
    """网关在 ARP 表里是不是"已解析"。/proc/net/arp 的 flags 位 0x2 = complete。
    ARP 由对方的协议栈回，和它回不回 ICMP 无关。"""
    for line in read("/proc/net/arp").splitlines()[1:]:
        f = line.split()
        if len(f) >= 4 and f[0] == gw:
            try:
                return bool(int(f[2], 16) & 0x2) and f[3] != "00:00:00:00:00:00"
            except ValueError:
                return False
    return False


def drop_stale_default(gw):
    """删掉指向链路本地地址的默认路由。

    🔎 2026-09-21 实测：usb0 这个 USB gadget 口已经不在 /sys/class/net 里了（设备没了），
    却留下一条 `default via 169.254.10.2 dev usb0 metric 0` 的僵尸路由，metric 比 DHCP
    装的那条小，于是真正的出口永远装不上 —— udhcpc 报 `RTNETLINK answers: File exists`
    然后一切照常返回 rc=0。netheal 原来只是躲开这种局面（gw 是 169.254 就跳过），
    躲开的结果是板子永远出不去。"""
    return sh("route del default gw %s" % gw, 20)


def subnet_of(ip):
    return ip.rsplit(".", 1)[0] if ip and ip.count(".") == 3 else ""


def want_ip(dev):
    """本网段的约定地址。当前地址优先，没地址就按网关猜 —— 换 AP 后地址还是旧网段的，
    那次会请求错网段的地址，服务器忽略、照常发一个随机的，下一轮由一次性纠正补上。"""
    return REQUEST_IPS.get(subnet_of(ipv4(dev))) or REQUEST_IPS.get(subnet_of(gateway()))


def dhcp_renew(dev, req=None):
    """🔴 两步必须分开跑。写成 `pkill ...; udhcpc -i <dev> ...` 一条命令时，执行它的这个
    shell 自己的命令行里就含着 `udhcpc -i <dev>`，pkill 当场把自己连同后半句一起打死 ——
    历史日志里 34 次 dhcp-renew 全是 rc=-15(SIGTERM)，udhcpc 一次都没真跑起来。
    `udhcp[c]` 的括号只保护得了模式串本身，保护不了同一行里的那个真实命令。"""
    # quiet：pkill 没匹配到进程就返回 1，那是常态不是故障，别让它每次都打 ERROR。
    sh("pkill -f 'udhcp[c].*%s'" % dev, 20, quiet=True)
    want = req if req is not None else want_ip(dev)
    return sh("udhcpc -i %s -n -q -t 8%s" % (dev, (" -r " + want) if want else ""), 40)


def steps(dev):
    """梯度。每一项是 (名字, 动作, 之后等多少秒再复验)。"""
    if dev.startswith("enx"):
        # ESP32 网桥：wifi-connect 那级是 USB WiFi 专用脚本，对它无意义甚至有害。
        # 重新枚举等效于人工断电重启，是实测唯一能救回断死的手段。
        return [
            ("dhcp-renew", lambda: dhcp_renew(dev), 15),
            ("link-bounce", lambda: sh("ifconfig %s down; sleep 2; ifconfig %s up" % (dev, dev), 30), 25),
            ("usb-reauthorize", lambda: usb_reauthorize(), 35),
        ]
    return [
        # 2026-08-25 定案:掉线现场是 `link=1 carrier=1 ip=none` —— 关联好着,丢的是 IP
        # (速率塌到 CCK_1M 后 DHCP 续租失败)。所以第一步只重新拿 IP,不要动关联:第2级的
        # 拆重建会把"关联着但没 IP"打成"连都连不上"(association failed after 20s),那次
        # 只能断电。自杀那个坑见 dhcp_renew 的注释。
        ("dhcp-renew", lambda: dhcp_renew(dev), 15),
        # 不用 wpa_cli reassociate:板上的 wpa_supplicant 是 wifi-connect.sh 手工起的、
        # 没带 -C 控制套接字，wpa_cli 直接 rc=255 连不上 —— 那一级是空操作(2026-08-24 实测)。
        ("link-bounce", lambda: sh("ifconfig %s down; sleep 2; ifconfig %s up" % (dev, dev), 30), 25),
        ("wifi-connect", lambda: sh("sh %s" % WIFI_UP, 120), 25),
        ("usb-reauthorize", lambda: (
            usb_reauthorize(),
            time.sleep(0 if DRY else 8),
            sh("sh %s" % WIFI_UP, 120))[0], 35),
    ]


def status():
    """一次性打印 netheal 此刻看到的**全部判据输入**，用来回答"它为什么这么判"。
    跑法：`python3 /usr/local/sbin/board_netheal.py --status`（只读，不动任何东西）。
    加这个是因为排查时最费时间的不是修，是搞清楚它当时看到了什么。"""
    dev = iface()
    gw = gateway()
    print("网卡        = %s  (ifindex=%s carrier=%s operstate=%s)"
          % (dev, ifindex(dev),
             read("/sys/class/net/%s/carrier" % dev, "?").strip() if dev else "-",
             read("/sys/class/net/%s/operstate" % dev, "?").strip() if dev else "-"))
    print("本机地址    = %s" % (ipv4(dev) if dev else "-"))
    print("默认网关    = %s%s" % (gw, "  ← 链路本地，会走无默认路由那条分支"
                                  if gw and gw.startswith("169.254.") else ""))
    print("网关 ARP    = %s" % ("已解析" if gw and arp_complete(gw) else "未解析/表里没有"))
    ok, why = probe(gw)
    print("ICMP 探测   = %s (%s)%s" % ("通" if ok else "不通", why,
          "" if ok else "   ⚠️ 手机热点的网关不回 ICMP，不通不代表断网"))
    r1 = rx_packets(dev)
    time.sleep(2.0)
    r2 = rx_packets(dev)
    d = (r2 - r1) if (r1 is not None and r2 is not None) else None
    print("2 秒收包    = %s 包%s" % (d, "  (>=%d 且 ARP 已解析就算链路在用)" % RX_ALIVE_PKTS
                                     if d is not None else ""))
    print("判活结论    = %s" % ("通" if ok else
          ("通（ping 无回应但链路在用）" if (d is not None and d >= RX_ALIVE_PKTS
                                            and gw and arp_complete(gw)) else "不通，会开始攒失败次数")))
    print()
    print("梯度        = %s" % ", ".join(n for n, _, _ in steps(dev or "enx0")))
    print("USB 设备目录 = %s  (写死的那个 %s 存在=%s)"
          % (usb_dev_path(), USB_DEV, os.path.exists(USB_DEV)))
    print("请求地址    = %s   固定副地址=%s"
          % (", ".join(sorted(REQUEST_IPS.values())) or "(关)",
             PIN_IP or "(关，网卡名顶满 IFNAMSIZ 时必须关)"))
    print("自动重启    = %s" % ("开" if ALLOW_REBOOT else "关"))
    print("周期/门限   = %.0fs / %d 次 (=%.0fs 才动手)" % (PERIOD_S, FAIL_N, PERIOD_S * FAIL_N))
    print()
    print("最近 8 条日志:")
    for ln in read(LOG).splitlines()[-8:]:
        print("  " + ln)


def main():
    dev = iface()
    gw = gateway()
    emit("netheal 起动 iface=%s gw=%s 周期=%.0fs 门限=%d次 dry_run=%s allow_reboot=%s"
         % (dev, gw, PERIOD_S, FAIL_N, DRY, ALLOW_REBOOT))
    ensure_pinned(dev)
    t_start = time.time()
    fails = 0
    nogw = 0
    last_nogw_fix = 0.0
    req_ip_forced = set()   # 已经纠正过的网段，换 AP 后新网段仍要纠正一次
    last_idx = ifindex(iface())
    last_noip_fix = 0.0
    last_rx = None
    pending_renew = 0
    armed = FORCE_FAIL
    rung = 0
    down_since = None
    last_rc = None
    backoff_until = 0.0
    while True:
        time.sleep(PERIOD_S)
        gw = gateway() or gw
        dev = iface() or dev
        ensure_pinned(dev)
        # 快速通道 1：网卡重新出现（换 AP 时 USB 网卡会整个消失再回来）。
        # 🔴 不能一看到序号变就立刻续租：网卡刚出现时内核还没把设备准备好，udhcpc 会
        # 报 `SIOCGIFINDEX: No such device` 失败，接着 fails 攒够就升级到第 2 级
        # link-bounce，把刚出现的网卡又弄没 —— 序号再变、再触发，自己把自己锁进循环。
        # 2026-09-22 早上的反复断连和重启就是这么来的。改成：记下待办，等下一轮、
        # 并且要求 carrier=1 再动手；这一类失败也不计入 fails。
        idx = ifindex(dev)
        if idx and last_idx and idx != last_idx:
            emit("网卡重新出现 ifindex %s->%s，等它就绪后续租" % (last_idx, idx))
            pending_renew = PENDING_MAX
            last_rx = None       # 网卡重建了，收包计数从头开始，不能跨着做差
        if idx:
            last_idx = idx
        if pending_renew and dev:
            if read("/sys/class/net/%s/carrier" % dev).strip() != "1":
                # 🔴 必须有次数上限：carrier 一直不为 1 就无限 continue 的话，梯度永远
                # 升不上去，netheal 整个瘫掉 —— 和「没默认路由就 continue」是同一类死角。
                pending_renew -= 1
                if pending_renew:
                    emit("网卡还没就绪(carrier!=1)，再等一轮(还剩%d次)" % pending_renew)
                    last_rx = rx_packets(dev)   # 跳过这一轮也要记，否则下轮增量横跨两周期
                    continue
                emit("网卡等了%d轮还没就绪，放弃等待，走正常梯度" % PENDING_MAX)
            else:
                pending_renew = 0
                rc, out = dhcp_renew(dev)
                emit("网卡就绪，续租 rc=%s 之后 ip=%s gw=%s" % (rc, ipv4(dev), gateway()))
                gw = gateway() or gw
                last_noip_fix = time.time()
                last_rx = rx_packets(dev)
                if rc == 0:
                    fails = 0          # 刚续上就别让上一轮攒的失败把梯度顶上去
                    continue           # 续上了就给地址一轮时间生效
                # 续租失败就不要 continue：那会把这一轮的探测和梯度一起跳过
        # 快速通道 2：压根没有地址。等 6 次网关探测毫无意义 —— 没地址就探不了。
        if dev and ipv4(dev) is None and time.time() - last_noip_fix > NOIP_COOLDOWN_S:
            last_noip_fix = time.time()
            rc, out = dhcp_renew(dev)
            emit("网卡没有地址，立即续租 rc=%s 之后 ip=%s gw=%s" % (rc, ipv4(dev), gateway()))
            gw = gateway() or gw
        # 拿不到网卡、或默认路由是 usb0 的链路本地地址，说明现在的状态本身不可信 ——
        # 这时候动手只会拿 None 去拼命令、或者去 ping 一个和 WiFi 无关的目标。
        if dev is None or gw is None or gw.startswith("169.254."):
            # 🔴 这里原来只是 continue，于是"跑到新网段却没拿到地址"这种局面**永远**没人管
            # —— 没有默认路由恰恰是最该重新要地址的时刻，不是"状态不可信"。
            # 但开机时 USB 重新枚举确实会短暂如此，所以给 NOGW_GRACE 轮宽限再动手，
            # 并且自己节流，免得在真没网的环境里每 15 秒打一次 udhcpc。
            nogw += 1
            if fails % 8 == 0:
                emit("状态不可信:iface=%s gw=%s 第%d轮（%d 轮后补一次 dhcp）" % (dev, gw, nogw, NOGW_GRACE))
            if dev and nogw >= NOGW_GRACE and time.time() - last_nogw_fix > NOGW_COOLDOWN_S:
                last_nogw_fix = time.time()
                if gw and gw.startswith("169.254."):
                    rc0, _ = drop_stale_default(gw)
                    emit("删掉指向 %s 的僵尸默认路由 rc=%s" % (gw, rc0))
                rc, out = dhcp_renew(dev)
                emit("无默认路由，补跑 dhcp-renew rc=%s 之后 gw=%s | %s"
                     % (rc, gateway(), out.strip().replace("\n", " / ")[-140:]))
            fails += 1
            # 🔴 跳过这一轮也要记收包数。不记的话，无网关期间的增量会全部攒到恢复后的
            # 第一轮上，那一轮的 d 必然很大 —— 判活的兜底("收够包+网关ARP在")就会把
            # 一次真故障误判成正常。和上面 carrier 那条 continue 是同一个道理。
            last_rx = rx_packets(dev) if dev else None
            continue
        nogw = 0
        # 固定地址只在续租时才请求，而开机那次 udhcpc 会随便拿一个地址、网络还是通的，
        # 于是永远不会续租 —— 地址就一直是随机的。这里补一刀：在已知网段却不是约定地址
        # 就主动换一次。**按网段各记一次**，否则换过 AP 之后第二个网段就没人管了。
        if dev and ipv4(dev):
            cur = ipv4(dev)
            want = REQUEST_IPS.get(subnet_of(cur))
            if want and want != cur and subnet_of(cur) not in req_ip_forced:
                req_ip_forced.add(subnet_of(cur))
                rc, out = dhcp_renew(dev, want)
                emit("地址 %s 不是约定的 %s，换一次 rc=%s 之后 ip=%s"
                     % (cur, want, rc, ipv4(dev)))
        ok, why = (False, "forced") if FORCE_FAIL else probe(gw)
        # 🔴 不能只信"对方回不回 ping"。2026-09-21 实测：手机热点的网关**根本不回 ICMP**
        # （ESP32 的 ICMP 对帐里几十个请求只换回 3 个回复），而数据一直在正常流动。
        # 只靠 ping 判活会让 netheal 永远以为断网，一遍遍跑梯度，第 2 级 ifconfig down/up
        # 会实打实把连接掐断 —— 用户看到的就是"一直断连"，而这是自愈自己造成的。
        # 兜底判据要两个条件同时成立，才不会反过来把真故障也判成正常：
        #   网关的 ARP 表项是完整的（ARP 由对方协议栈回，与 ICMP 策略无关）
        #   且这一轮确实收到了包（链路在被使用）
        rx = rx_packets(dev)
        if not ok and not FORCE_FAIL and rx is not None and last_rx is not None:
            d = rx - last_rx
            if d >= RX_ALIVE_PKTS and arp_complete(gw):
                ok, why = True, "ping无回应但链路在用(收%d包+网关ARP已解析)" % d
        last_rx = rx
        if ok:
            if down_since is not None:
                lad = steps(dev)
                if rung == 0:
                    how = "自己好的(还没动手)"
                elif last_rc == 0:
                    how = "第%d级 %s 之后" % (rung, lad[rung - 1][0])
                else:
                    # 那一级没执行成功就别记它的功。2026-08-24 第1级 rc=255 却被记成"救回"
                    how = "第%d级 %s 之后，但那一级 rc=%s，很可能是自己好的" % (
                        rung, lad[rung - 1][0], last_rc)
                emit("恢复 断了%.0fs %s | %s" % (time.time() - down_since, how, snapshot(dev)))
            armed, fails, rung, down_since, last_rc = True, 0, 0, None, None
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
            # 歇之前必须把网卡留在"通电并在尝试关联"的状态，不能留在 down。
            # 🔴 但只对 USB WiFi 有意义。wifi-connect.sh 里 IFACE 写死成早已不存在的
            # wlx80ea07cb5d4a，对 enx(ESP32 网桥) 跑它只会空等 30 秒 —— 而这是梯度全败、
            # 最需要及时重试的时刻，却让 netheal 聋 30 秒，日志还谎称"已重新拉起网卡"。
            if dev.startswith("enx"):
                emit("整条梯度都没救回来，歇 %.0fs 再从头试(ESP32 网桥没有可拉起的网卡；"
                     "要自动重启就设 NETHEAL_ALLOW_REBOOT=1) | %s" % (BACKOFF_S, snapshot(dev)))
            else:
                sh("sh %s" % WIFI_UP, 120)
                emit("整条梯度都没救回来，已重新拉起网卡，歇 %.0fs 再从头试(要自动重启就设 "
                     "NETHEAL_ALLOW_REBOOT=1) | %s" % (BACKOFF_S, snapshot(dev)))
            rung, backoff_until = 0, time.time() + BACKOFF_S
            continue
        name, act, wait = lad[rung]
        rung += 1
        emit("第%d级 %s 开始 | %s" % (rung, name, snapshot(dev)))
        rc, out = act()
        last_rc = rc
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
    if "--status" in sys.argv:
        status()
        sys.exit(0)
    sys.exit(main())
