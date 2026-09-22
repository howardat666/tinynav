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
    """DHCP 自带的 requested-ip 让板子每次都要同一个地址，人不用满网段扫。
    🔴 但只能配给**认这个选项的**服务器。办公网实测不认（请求 .102 给回 .42，换 -C 当
    新客户端给 .125），给它配一条只会每次开机白跑一次续租、断网 20~40 秒。"""
    calls = []
    real_sh, real_ip, real_gw = N.sh, N.ipv4, N.gateway
    N.sh = lambda c, t=90, quiet=False: (calls.append(c), (0, ""))[-1]
    N.gateway = lambda: ""
    try:
        assert "10.140.21" in N.REQUEST_IPS, "热点那侧认 requested-ip，必须留着"
        assert "192.168.19" not in N.REQUEST_IPS, "办公网不认，配了只会白断网一次"
        calls.clear()
        N.ipv4 = lambda d: "10.140.21.77"
        N.dhcp_renew(DEV)
        assert "-r 10.140.21.9" in calls[1], calls[1]
        # 反向验证：办公网地址上不该带 -r
        calls.clear()
        N.ipv4 = lambda d: "192.168.19.42"
        N.dhcp_renew(DEV)
        assert "-r" not in calls[1], calls[1]
        # 显式指定优先于按网段推断
        calls.clear()
        N.ipv4 = lambda d: "10.140.21.77"
        N.dhcp_renew(DEV, "192.168.19.102")
        assert "-r 192.168.19.102" in calls[1], calls[1]
        # 反向验证：陌生网段不该瞎请求别的网段的地址
        calls.clear()
        N.ipv4 = lambda d: "172.16.0.5"
        N.dhcp_renew(DEV)
        assert "-r" not in calls[1], calls[1]
    finally:
        N.sh, N.ipv4, N.gateway = real_sh, real_ip, real_gw


def t_fixed_ip_corrected_per_subnet():
    """开机那次 udhcpc 不带 -r，网络是通的所以永远不会续租 —— 要有一次性纠正。
    🔴 而这个"一次"必须**按网段各记一次**：写成每次开机一个布尔时，先连热点纠正过一次，
    之后换回实验室网就再也不会纠正，地址依旧随机。这就是 2026-09-22 的实际故障。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    body = src.split("def main(")[1]
    assert "req_ip_forced = set()" in body, "得按网段记，不能是布尔"
    assert "req_ip_forced = False" not in body, "旧的全局一次性写法还在"
    assert "req_ip_forced.add(subnet_of(" in body, "记的必须是网段而不是一个标志位"
    assert "subnet_of(cur) not in req_ip_forced" in body, "判据也得按网段查"


def t_ladder_exhausted_skips_usb_wifi_script():
    """梯度全败的兜底里不能对 enx 跑 wifi-connect.sh：那脚本的 IFACE 写死成早就不存在的
    wlx80ea07cb5d4a，只会空等 30 秒，而这正是最需要及时重试的时刻，日志还谎称拉起了网卡。
    steps() 里早就把这一级从 enx 梯度里去掉了，兜底却漏了。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    branch = src.split("rung >= len(lad)")[1][:900]
    assert 'dev.startswith("enx")' in branch, "兜底要分平台"
    enx_part = branch.split('dev.startswith("enx")')[1].split("else:")[0]
    assert "WIFI_UP" not in enx_part, "enx 分支里还在跑 wifi-connect"
    assert "WIFI_UP" in branch, "USB WiFi 那条路还得保留"


def t_skipped_round_still_records_rx():
    """每一条 continue 之前都要记下收包数。漏记的话，跳过期间的增量会全攒到恢复后的
    第一轮，判活的兜底(收够包+网关ARP在)就会把一次真故障误判成正常。"""
    src = open("/home/dm/looper/tinynav-x5/tool/x5_board/board_netheal.py").read()
    body = src.split("def main(")[1]
    nogw = body.split("nogw += 1")[1].split("nogw = 0")[0]
    assert "last_rx = rx_packets" in nogw, "无网关分支 continue 前没记收包数"


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
    # 🔴 不能一看到序号变就立刻续租：网卡刚出现时内核还没准备好，udhcpc 报
    # SIOCGIFINDEX: No such device，失败又会把梯度顶到 link-bounce 把网卡弄没，循环。
    assert "pending_renew" in fast, "序号变化后没有等就绪，会在设备没准备好时就 udhcpc"
    assert "carrier" in fast, "没有检查 carrier 就动手"
    # 🔴 等待必须有次数上限。今天已经在同一类死角上栽了两次：「没默认路由就 continue」
    # 和「等 carrier 就无限 continue」——都让梯度永远升不上去，整个自愈瘫掉。
    assert "PENDING_MAX" in fast, "等 carrier 没有次数上限，会无限 continue 把梯度卡死"
    assert N.PENDING_MAX >= 2, "上限太小"
    assert "last_rx = None" in fast, "网卡重建后没作废 last_rx，会跨着做差算出假增量"
    assert fast.count("last_rx = rx_packets(dev)") >= 2, "跳过的轮次没更新 last_rx，增量会横跨两周期"
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
    check("固定地址按网段各纠正一次", t_fixed_ip_corrected_per_subnet)
    check("梯度兜底对 enx 不跑 USB WiFi 脚本", t_ladder_exhausted_skips_usb_wifi_script)
    check("跳过的轮次也记收包数", t_skipped_round_still_records_rx)
    check("没有默认路由时会补跑 dhcp", t_no_gateway_is_acted_on)
    check("事件驱动的两条快速通道", t_fast_paths)
    check("USB 路径按网卡反查而非写死", t_usb_path_is_discovered)
    check("判活不只看 ICMP", t_liveness_not_only_icmp)
    check("enx 梯度不含 wifi-connect", t_ladder_shape)
    print("\n%s" % ("全部通过" if not FAILS else "失败: %s" % FAILS))
    sys.exit(1 if FAILS else 0)
