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
    check("enx 梯度不含 wifi-connect", t_ladder_shape)
    print("\n%s" % ("全部通过" if not FAILS else "失败: %s" % FAILS))
    sys.exit(1 if FAILS else 0)
