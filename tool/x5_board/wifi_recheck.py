import re, sys, statistics as st
L = sys.argv[1]
rows = []
for ln in open(L, errors='replace'):
    m = re.match(r'(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) up=(\d+) load=([\d.]+).*?rssi=(-?\d+) qual=(\d+) link=(\d) igi=0x([0-9a-f]+) fa=(\d+)', ln)
    if m:
        rows.append(dict(t=m.group(1), load=float(m.group(3)), rssi=int(m.group(4)),
                         qual=int(m.group(5)), link=int(m.group(6)),
                         igi=int(m.group(7), 16), fa=int(m.group(8))))
print("样本 %d 条\n" % len(rows))

# igi 和 fa 的关系：如果是"外来干扰"，fa 高时 igi 应该被 DIG 调【高】(降灵敏度)；
# 如果是"信号弱导致 AGC 开大"，则 igi 【低】的时候 fa 高。
ok = [r for r in rows if r['link'] == 1]
lo_igi = [r['fa'] for r in ok if r['igi'] <= 0x24]
hi_igi = [r['fa'] for r in ok if r['igi'] >= 0x30]
def med(v): return sorted(v)[len(v)//2] if v else float('nan')
print("igi 低（灵敏度开大, <=0x24）时 fa 中位 %6.0f   n=%d" % (med(lo_igi), len(lo_igi)))
print("igi 高（灵敏度收小, >=0x30）时 fa 中位 %6.0f   n=%d" % (med(hi_igi), len(hi_igi)))
print("→ 若前者远大于后者，说明 fa 高是【AGC 开大】的结果，不是外来强干扰\n")

# rssi 和 igi 的关系
lo_rssi = [r['igi'] for r in ok if r['rssi'] <= -65]
hi_rssi = [r['igi'] for r in ok if r['rssi'] >= -50]
print("rssi 弱(<=-65) 时 igi 中位 0x%02x   n=%d" % (int(med(lo_rssi)), len(lo_rssi)))
print("rssi 强(>=-50) 时 igi 中位 0x%02x   n=%d" % (int(med(hi_rssi)), len(hi_rssi)))
print("→ 若信号弱时 igi 更低，则确认 DIG 是跟着 rssi 走的\n")

# 掉线前 60 s 的 rssi 波动 vs 全体
edges = [i for i in range(1, len(rows)) if rows[i-1]['link'] == 1 and rows[i]['link'] == 0]
sd_before, trend = [], []
for i in edges:
    w = [r['rssi'] for r in rows[max(0, i-6):i] if r['link'] == 1]
    if len(w) >= 4:
        sd_before.append(st.pstdev(w))
        trend.append(w[-1] - w[0])
allsd = []
for i in range(6, len(rows), 6):
    w = [r['rssi'] for r in rows[i-6:i] if r['link'] == 1]
    if len(w) >= 4:
        allsd.append(st.pstdev(w))
print("掉线前 60s 的 rssi 标准差: 中位 %.1f dB   n=%d" % (med(sd_before), len(sd_before)))
print("全体 60s 窗口的 rssi 标准差: 中位 %.1f dB   n=%d" % (med(allsd), len(allsd)))
print("掉线前 60s 的 rssi 净变化: 中位 %+.0f dB（负=在变弱）" % med(trend))
