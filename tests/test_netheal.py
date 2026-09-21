#!/usr/bin/env python3
"""board_netheal 的离线测试。PC 上直接 `python3 tests/test_netheal.py`，不需要 ROS/板子。

每一组都对应一个真实发生过的故障，组名即故障。**每组末尾都有反向验证** —— 证明这条
判据真能把旧写法判红，否则测试全绿也说明不了任何事（2026-09-21 栽过：给别名功能写了
6 项测试全过，而 bug 在"这条命令在真机上的语义"，测试只验了"发出了哪条命令"）。
"""
import re
import sys

sys.argv = ["test_netheal"]
sys.path.insert(0, "/home/dm/looper/tinynav-x5/tool/x5_board")
import board_netheal as N  # noqa: E402

DEV = "enxa4cb8fd549dc"          # 15 字符，正好顶满 IFNAMSIZ 上限
FAILS = []


def check(name, fn):
    try:
        fn()
        print("  ok   %s" % name)
    except AssertionError as e:
        FAILS.append(name)
        print("  FAIL %s -> %s" % (name, e))


# ---------------------------------------------------------------- pkill 自杀
def t_pkill_not_suicidal():
    """dhcp-renew 历史上 34 次全是 rc=-15：pkill 和 udhcpc 写在同一条 shell 命令里，
    执行它的 shell 自身命令行含着后半句，pkill 把自己打死。"""
    calls = []
    real = N.sh
    N.sh = lambda c, t=90, quiet=False: (calls.append((c, quiet)), (0, ""))[-1]
    N.dhcp_renew(DEV)
    N.sh = real
    assert len(calls) == 2, calls
    kill, renew = calls[0][0], calls[1][0]
    assert kill.startswith("pkill") and renew.startswith("udhcpc"), calls
    pat = re.search(r"pkill -f '(.*)'", kill).group(1)
    assert re.search(pat, "/bin/sh -c " + kill) is None, "pkill 会打死自己: %r" % pat
    # pkill 没匹配到进程返回 1 是常态，不能每轮都打 ERROR
    assert calls[0][1] is True and calls[1][1] is False, calls
    # 反向验证：旧的合并写法必须被同一条判据抓出来
    old = "/bin/sh -c pkill -f 'udhcp[c].*%s' 2>/dev/null; udhcpc -i %s -n -q -t 8" % (DEV, DEV)
    assert re.search(pat, old) is not None, "判据没有鉴别力：旧写法也没被抓到"


# ------------------------------------------------------------------- rc 告警
def t_rc_is_loud():
    """rc 非零必须自己叫出来。原来它只进日志字符串，34 次淹在 INFO 里没人看见。"""
    logs = []
    real = N.emit
    N.emit = logs.append
    try:
        assert N.sh("true")[0] == 0 and not logs, logs
        logs.clear()
        assert N.sh("false")[0] == 1 and logs and logs[0].startswith("ERROR"), logs
        logs.clear()
        assert N.sh("kill -TERM $$")[0] == -15 and "自己的 pkill" in logs[0], logs
        logs.clear()
        assert N.sh("false", quiet=True)[0] == 1 and not logs, logs
    finally:
        N.emit = real


# --------------------------------------------------------------- 别名截断
def t_pinned_alias_disabled():
    """`ifconfig <dev>:1` 的别名名超过 IFNAMSIZ(15) 会被截回主接口，命令从"加副地址"
    变成"改主地址"，板子当场失联，而 ifconfig 返回 rc=0。板上没有 `ip` 命令，
    没有别的加副地址的办法，所以这个功能默认必须是关的。"""
    assert N.PIN_IP == "", "PIN_IP 默认必须为空：%r" % N.PIN_IP
    calls = []
    real_sh, real_emit, real_ipv4 = N.sh, N.emit, N.ipv4
    N.sh = lambda c, t=90, quiet=False: (calls.append(c), (0, ""))[-1]
    N.emit = lambda s: None
    N.ipv4 = lambda d: None
    try:
        N.ensure_pinned(DEV)
        assert calls == [], "默认关闭时不该动手: %s" % calls
        # 就算有人把它打开，别名名字也必须塞得进 IFNAMSIZ，否则会落到主接口上
        N.PIN_IP = "10.140.21.9"
        N.ensure_pinned(DEV)
        alias = "%s:1" % DEV
        assert len(alias) > 15, "这个断言本身失效了，DEV 该是 15 字符"
        assert all(alias not in c for c in calls), \
            "别名 %r 长 %d 字节，超过 IFNAMSIZ 上限会被截回主接口" % (alias, len(alias))
    finally:
        N.PIN_IP = ""
        N.sh, N.emit, N.ipv4 = real_sh, real_emit, real_ipv4


# --------------------------------------------------------------- 固定地址请求
def t_requests_fixed_ip():
    """热点每次给的地址都不同，而小米热点页不显示 IP，人根本找不到板子。
    用 DHCP 自带的 requested-ip 让它每次都要同一个地址。板上实测：网段对不上时
    服务器给别的地址、udhcpc 照常拿到租约，所以两个网络通用。"""
    calls = []
    real = N.sh
    N.sh = lambda c, t=90, quiet=False: (calls.append(c), (0, ""))[-1]
    try:
        assert N.REQUEST_IP, "默认该带一个请求地址"
        N.dhcp_renew(DEV)
        assert "-r %s" % N.REQUEST_IP in calls[1], calls[1]
        # 反向验证：关掉它就不该再带 -r
        calls.clear()
        N.REQUEST_IP = ""
        N.dhcp_renew(DEV)
        assert "-r" not in calls[1], calls[1]
    finally:
        N.REQUEST_IP = "10.140.21.9"
        N.sh = real


# ------------------------------------------------------- 没有默认路由不能空转
def t_no_gateway_is_acted_on():
    """原来 gw 为空就 continue，于是"跑到新网段却没拿到地址"永远没人管 —— 而那恰恰
    是最该重新要地址的时刻。开机时 USB 重新枚举会短暂无路由，所以要有宽限和节流。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    head = src.split("def main(")[1]
    blk = head.split("nogw += 1")[1].split("nogw = 0")[0]
    assert "dhcp_renew" in blk, "无默认路由的分支里没有补救动作，还是在空转"
    assert "NOGW_GRACE" in blk and "NOGW_COOLDOWN_S" in blk, "缺宽限或节流"
    assert "drop_stale_default" in blk, "没先删掉指向链路本地的僵尸默认路由，dhcp 装不上路由"
    assert N.NOGW_GRACE >= 3, "宽限太短，会和开机时的 USB 枚举抖动打架"
    assert N.NOGW_COOLDOWN_S >= 60, "节流太松，真没网时会每轮都打 udhcpc"
    # 反向验证：这条判据对旧写法（只有 continue）必须判红
    old = "\n            if fails % 8 == 0:\n                emit(...)\n            fails += 1\n            continue\n"
    assert "dhcp_renew" not in old, "判据没有鉴别力"


# ------------------------------------------------------------- 事件驱动快速通道
def t_fast_paths():
    """换 AP 时 USB 网卡整个消失再出现（实测 usb=0 + SIOCGIFINDEX: No such device）。
    等"网关探测连续失败 6 次"要 90 秒，而网卡序号变化是确定性信号，可以立刻动手；
    "根本没有地址"同理 —— 没地址就探不了网关，等 6 次毫无意义。
    这样既快又不用降低 FAIL_N，不会因为网络抖一下就误触发。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    body = src.split("def main(")[1]
    fast = body.split("ensure_pinned(dev)")[2].split("ok, why =")[0]
    assert "ifindex" in fast and "dhcp_renew" in fast, "缺少网卡重新出现这条快速通道"
    assert "ipv4(dev) is None" in fast, "缺少没有地址这条快速通道"
    assert "NOIP_COOLDOWN_S" in fast, "没地址那条缺节流，真断网时会每轮打 udhcpc"
    assert "last_idx and idx != last_idx" in fast, "开机第一次观测就会误触发"
    # 反向验证：这条判据对旧写法（直接进 probe）必须判红
    assert "ifindex" not in "        ok, why = probe(gw)", "判据没有鉴别力"
    assert N.NOIP_COOLDOWN_S >= 30, "节流太松"


def t_usb_path_is_discovered():
    """第 3 级 usb-reauthorize 写死 /sys/bus/usb/devices/1-1，这块板子上不存在 ——
    2026-09-21 日志实证 rc=2 Directory nonexistent，那一级从来没真执行过。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    fn = src.split("def usb_dev_path(")[1].split("\ndef ")[0]
    assert "realpath" in fn and "authorized" in fn, "没有按网卡反查，还是写死路径"
    ladder = src.split("def steps(")[1].split("\ndef ")[0]
    assert "USB_DEV" not in ladder, "梯度里还直接引用写死的 USB_DEV"
    assert callable(N.usb_dev_path)


# ------------------------------------------------- 判活不能只靠对方回不回 ping
def t_liveness_not_only_icmp():
    """2026-09-21 实测：手机热点的网关根本不回 ICMP（ESP32 的 ICMP 对帐里几十个请求
    只换回 3 个回复），而数据一直在正常流动。只靠 ping 判活会让 netheal 永远以为断网，
    第 2 级 ifconfig down/up 把连接实打实掐断 —— "一直断连"是自愈自己造成的。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    blk = src.split("ok, why = (False, \"forced\") if FORCE_FAIL else probe(gw)")[1] \
             .split("if ok:")[0]
    assert "rx_packets" in blk and "arp_complete" in blk, "还是只靠 ICMP 判活"
    assert "not ok and" in blk, "兜底判据必须只在 ping 失败时才用"
    assert "and arp_complete" in blk, "两个条件必须同时成立，否则会把真故障也判成正常"
    # 反向验证：旧写法只有一行 probe，这条判据必须判红
    assert "rx_packets" not in "        ok, why = probe(gw)", "判据没有鉴别力"
    # ARP 解析本身
    sample = ("IP address  HW type  Flags  HW address         Mask  Device\n"
              "10.0.0.1    0x1      0x2    c6:cf:3f:4a:3e:1a  *     enx0\n"
              "10.0.0.2    0x1      0x0    00:00:00:00:00:00  *     enx0\n")
    real = N.read
    N.read = lambda p, d="": sample if "arp" in p else d
    try:
        assert N.arp_complete("10.0.0.1") is True
        assert N.arp_complete("10.0.0.2") is False, "flags=0x0 不算已解析"
        assert N.arp_complete("10.0.0.9") is False, "表里没有不算已解析"
    finally:
        N.read = real


# ------------------------------------------------------------------ 梯度形状
def t_ladder_shape():
    """enx*(ESP32 网桥) 这条路不该出现 wifi-connect 那一级 —— 那是 USB WiFi 网卡
    专用脚本，网卡已经物理拆掉了，对网桥无意义甚至有害。"""
    names = [s[0] for s in N.steps(DEV)]
    assert names[0] == "dhcp-renew", names
    assert "wifi-connect" not in names, names
    assert "wifi-connect" in [s[0] for s in N.steps("wlan0")], "反向验证：wl* 那条路该有它"


if __name__ == "__main__":
    print("board_netheal 离线测试")
    check("pkill 不会打死自己", t_pkill_not_suicidal)
    check("rc 非零会报 ERROR", t_rc_is_loud)
    check("别名功能默认关且名字不超长", t_pinned_alias_disabled)
    check("续租会请求固定地址", t_requests_fixed_ip)
    check("没有默认路由时会补跑 dhcp", t_no_gateway_is_acted_on)
    check("事件驱动的两条快速通道", t_fast_paths)
    check("USB 路径按网卡反查而非写死", t_usb_path_is_discovered)
    check("判活不只看 ICMP", t_liveness_not_only_icmp)
    check("enx 梯度不含 wifi-connect", t_ladder_shape)
    print("\n%s" % ("全部通过" if not FAILS else "失败: %s" % FAILS))
    sys.exit(1 if FAILS else 0)
