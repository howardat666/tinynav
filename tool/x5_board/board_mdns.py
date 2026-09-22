#!/usr/bin/env python3
"""播报 looper.local，让人不用再找板子的 IP。

办公网的 DHCP **无视 requested-ip**（2026-09-22 实测：带 -C 换客户端身份后仍从池子里
发 .125），所以地址固定不了，只能反过来让名字固定。板上装不了 avahi（无 pip、busybox），
这里用 stdlib 实现一个只答 A 记录的最小 mDNS 应答器。

    python3 board_mdns.py            # 服务模式
    python3 board_mdns.py --selftest # 离线自检，不碰网络
"""
import errno
import os
import socket
import struct
import sys
import time

NAME = os.environ.get("MDNS_NAME", "looper")
GROUP = "224.0.0.251"
PORT = 5353
TTL = 120
QTYPE_A, QTYPE_ANY, CLASS_IN = 1, 255, 1
FLUSH = 0x8000          # cache-flush 位：让解析方立刻丢掉旧地址，而不是等 TTL 过期


def encode_name(name):
    return b"".join(bytes([len(p)]) + p.encode() for p in name.split(".")) + b"\0"


def read_name(buf, off):
    """返回 (名字, 新偏移)。压缩指针只跟一次 —— 查询包里本来就不该有环。"""
    parts, jumped, end = [], False, off
    for _ in range(128):
        if off >= len(buf):
            return "", len(buf)
        n = buf[off]
        if n & 0xC0 == 0xC0:
            if off + 1 >= len(buf):
                return "", len(buf)
            if not jumped:
                end, jumped = off + 2, True
            off = ((n & 0x3F) << 8) | buf[off + 1]
            continue
        off += 1
        if n == 0:
            return ".".join(parts), (end if jumped else off)
        parts.append(buf[off:off + n].decode("ascii", "replace"))
        off += n
    return "", len(buf)


def wants_us(buf, fqdn):
    """这个包里有没有在问我们的 A 记录。响应包(QR=1)一律不理，否则会互相应答成环。"""
    if len(buf) < 12 or (buf[2] & 0x80):
        return False
    qd = struct.unpack("!H", buf[4:6])[0]
    off = 12
    for _ in range(min(qd, 16)):
        name, off = read_name(buf, off)
        if off + 4 > len(buf):
            return False
        qtype, _qclass = struct.unpack("!HH", buf[off:off + 4])
        off += 4
        if name.lower() == fqdn and qtype in (QTYPE_A, QTYPE_ANY):
            return True
    return False


def build_answer(fqdn, ip):
    rr = encode_name(fqdn) + struct.pack("!HHIH", QTYPE_A, CLASS_IN | FLUSH, TTL, 4)
    return struct.pack("!HHHHHH", 0, 0x8400, 0, 1, 0, 0) + rr + socket.inet_aton(ip)


MREQ = socket.inet_aton(GROUP) + socket.inet_aton("0.0.0.0")


def route_dev(path="/proc/net/route"):
    """默认路由走哪块网卡。用它的 ifindex 判断"网卡是不是被重新插过"。"""
    try:
        with open(path) as f:
            for line in f.read().splitlines()[1:]:
                col = line.split()
                if len(col) > 1 and col[1] == "00000000":
                    return col[0]
    except OSError:
        pass
    return ""


def dev_ifindex(dev, root="/sys/class/net"):
    try:
        with open(os.path.join(root, dev, "ifindex")) as f:
            return f.read().strip()
    except OSError:
        return ""


def rejoin(s):
    """重挂组播成员，返回是否成功。

    🔴 USB 网卡消失再出现时 ifindex 会变，旧的成员资格跟着设备一起没了，而 socket 不会
    自己补 —— 不补就再也收不到查询，只剩 60 秒一次的定时播报在硬撑（TTL 120s 时勉强够，
    一旦播报也丢一次，名字就解析不到了）。这块板子的 ifindex 实测变过 4->5->6。"""
    try:
        s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, MREQ)
    except OSError:
        pass
    try:
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, MREQ)
        return True
    except OSError as e:
        return e.errno == errno.EADDRINUSE


_ip_cache = ["", 0.0]


def my_ip(max_age=2.0):
    """连一下网关方向就能拿到出口地址，不用解析 ifconfig 的输出，也不真发包。
    缓存 2 秒：每条查询都要现取（否则地址刚变时会答出旧的），但不能每个包都开一次 socket。"""
    if time.time() - _ip_cache[1] < max_age:
        return _ip_cache[0]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((GROUP, PORT))
        ip = s.getsockname()[0]
        _ip_cache[:] = ["" if ip.startswith("127.") else ip, time.time()]
    except OSError:
        _ip_cache[:] = ["", time.time()]
    finally:
        s.close()
    return _ip_cache[0]


def selftest():
    fqdn = "looper.local"
    q = struct.pack("!HHHHHH", 1, 0, 1, 0, 0, 0) + encode_name(fqdn) + struct.pack("!HH", QTYPE_A, CLASS_IN)
    assert wants_us(q, fqdn), "标准 A 查询该被认出来"
    assert wants_us(q[:12] + encode_name(fqdn) + struct.pack("!HH", QTYPE_ANY, CLASS_IN), fqdn), "ANY 也要答"
    assert not wants_us(q[:12] + encode_name("other.local") + struct.pack("!HH", QTYPE_A, CLASS_IN), fqdn), "别人的名字不能答"
    assert not wants_us(q[:12] + encode_name(fqdn) + struct.pack("!HH", 28, CLASS_IN), fqdn), "AAAA 我们没有，不能答"
    resp = bytearray(q); resp[2] |= 0x80
    assert not wants_us(bytes(resp), fqdn), "响应包必须忽略，否则会成环"
    assert not wants_us(b"\x00" * 8, fqdn) and not wants_us(b"", fqdn), "截断包不能崩"
    # 压缩指针指向自己 -> 必须能退出而不是死循环
    assert not wants_us(struct.pack("!HHHHHH", 1, 0, 1, 0, 0, 0) + b"\xc0\x0c" + struct.pack("!HH", QTYPE_A, CLASS_IN), fqdn)
    a = build_answer(fqdn, "192.168.19.42")
    assert a[2:4] == b"\x84\x00" and a[6:8] == b"\x00\x01", "要是权威响应且带 1 条答案"
    assert a.endswith(socket.inet_aton("192.168.19.42"))
    assert struct.unpack("!H", a[-12:-10])[0] & FLUSH, "必须带 cache-flush，否则改了地址要等 TTL"
    import tempfile
    d = tempfile.mkdtemp()
    # /proc/net/route：第一列是网卡，第二列 00000000 才是默认路由
    rt = os.path.join(d, "route")
    open(rt, "w").write(
        "Iface\tDestination\tGateway\tFlags\n"
        "enx0\t0013A8C0\t00000000\t0001\n"          # 直连网段，不是默认路由
        "enx0\t00000000\tFE13A8C0\t0003\n")
    assert route_dev(rt) == "enx0", route_dev(rt)
    open(rt, "w").write("Iface\tDestination\tGateway\n" "enx0\t0013A8C0\t00000000\n")
    assert route_dev(rt) == "", "没有默认路由时不能瞎认一块网卡"
    assert route_dev(os.path.join(d, "没这个文件")) == ""
    os.makedirs(os.path.join(d, "enx0"))
    open(os.path.join(d, "enx0", "ifindex"), "w").write("7\n")
    assert dev_ifindex("enx0", d) == "7" and dev_ifindex("不存在", d) == ""
    # rejoin 只有 ADD 这一步的错误码决定成败：已经在组里算成功，设备没了要如实报失败。
    class _Sock(object):
        def __init__(self, err):
            self.err = err
        def setsockopt(self, level, opt, val):
            if opt == socket.IP_ADD_MEMBERSHIP and self.err:
                raise OSError(self.err, "fake")
    assert rejoin(_Sock(0)), "正常加入该算成功"
    assert rejoin(_Sock(errno.EADDRINUSE)), "已经在组里该算成功，否则每轮都误报重挂失败"
    assert not rejoin(_Sock(errno.ENODEV)), "设备没了必须如实报失败，不能吞掉"
    print("board_mdns 自检全部通过")


def main():
    fqdn = NAME.lower() + ".local"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    s.bind(("", PORT))
    rejoin(s)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
    s.settimeout(5)
    print("mdns 起动 name=%s" % fqdn, flush=True)

    last_ip, last_announce, answered = "", 0.0, 0
    last_idx = dev_ifindex(route_dev())
    while True:
        ip = my_ip()
        now = time.time()
        idx = dev_ifindex(route_dev())
        if idx and idx != last_idx:
            print("出口网卡 ifindex %s -> %s，重挂组播成员 ok=%s"
                  % (last_idx or "(无)", idx, rejoin(s)), flush=True)
            last_idx, last_announce = idx, 0.0   # 顺便强制立刻重播一次
        # 地址变了立刻重播，否则别人要等 TTL 到期才知道 —— 而"地址会变"正是这东西存在的理由。
        if ip and (ip != last_ip or now - last_announce > 60):
            if ip != last_ip:
                print("地址 %s -> %s，重新播报" % (last_ip or "(无)", ip), flush=True)
            try:
                s.sendto(build_answer(fqdn, ip), (GROUP, PORT))
            except OSError as e:
                print("播报失败 %r" % e, flush=True)
            last_ip, last_announce = ip, now
        try:
            buf, addr = s.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            time.sleep(1)
            continue
        cur = my_ip()
        if cur and wants_us(buf, fqdn):
            try:
                s.sendto(build_answer(fqdn, cur), (GROUP, PORT))
                answered += 1
                if answered in (1, 10, 100) or answered % 1000 == 0:
                    print("已应答 %d 次（最近来自 %s）ip=%s" % (answered, addr[0], cur), flush=True)
            except OSError as e:
                print("应答失败 %r" % e, flush=True)


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        main()
