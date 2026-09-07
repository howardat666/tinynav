import re, sys, collections
L, C = sys.argv[1], sys.argv[2]
t_pause = None
for ln in open(C, errors="replace"):
    if "nav paused" in ln:
        m = re.search(r"\[(\d+\.\d+)\]", ln)
        if m:
            t_pause = float(m.group(1)); break
print("nav 暂停于时间戳", t_pause)
rows = []
for ln in open(L, errors="replace"):
    if "decision:" not in ln:
        continue
    tm = re.search(r"\[(\d+\.\d+)\]", ln)
    def g(p, s=ln):
        m = re.search(p, s)
        return float(m.group(1)) if m else None
    rows.append(dict(t=float(tm.group(1)),
                     vx=g(r"chose vx=([-+0-9.]+)"), om=g(r"omega=([-+0-9.]+)"),
                     fc=g(r"front_clearance=>?([0-9.]+)m"), esdf=g(r"esdf_at_robot=([0-9.]+)m"),
                     cells=g(r"obstacle_cells=([0-9]+)"), fwd=g(r"fwd_ok=([0-9]+)/"),
                     blocked=g(r"blocked=([0-9]+)/"), lag=g(r"stamp_lag=([0-9.]+)"),
                     cyc=g(r"cycle=([0-9.]+)")))
drive = [r for r in rows if t_pause is None or r["t"] < t_pause]
print("决策总数 %d，真正在跑的 %d（时长 %.0f s）" % (
    len(rows), len(drive), drive[-1]["t"] - drive[0]["t"]))

def pct(v, q):
    v = sorted(x for x in v if x is not None)
    return v[min(int(len(v)*q), len(v)-1)] if v else float("nan")

print("\n真正在跑那段的统计:")
for name, key in (("esdf_at_robot", "esdf"), ("front_clearance", "fc"),
                  ("obstacle_cells", "cells"), ("fwd_ok", "fwd"),
                  ("stamp_lag", "lag"), ("cycle", "cyc")):
    v = [r[key] for r in drive]
    print("  %-16s p10=%6.2f p50=%6.2f p90=%6.2f max=%6.2f" % (
        name, pct(v, .1), pct(v, .5), pct(v, .9), pct(v, 1.0)))
print("  选出的 vx:", dict(sorted(collections.Counter(r["vx"] for r in drive).items())))

bad = [i for i, r in enumerate(drive) if r["esdf"] is not None and r["esdf"] < 0.172]
print("\n🔴 esdf_at_robot < hard(0.172)，车体已压进障碍: %d 次 / %d" % (len(bad), len(drive)))
if bad:
    print("   前 12 次，带上一拍状态（看障碍是不是突然出现的）:")
    for i in bad[:12]:
        r = drive[i]; p = drive[i-1] if i > 0 else r
        print("   t+%6.1fs  esdf %.2f->%.2f  fc %.2f->%.2f  上一拍vx %+.3f  cells %.0f->%.0f  lag %.2f" % (
            r["t"]-drive[0]["t"], p["esdf"], r["esdf"], p["fc"], r["fc"],
            p["vx"], p["cells"], r["cells"], r["lag"]))
# front_clearance 的突降
drops = [(drive[i]["t"]-drive[0]["t"], drive[i-1]["fc"], drive[i]["fc"], drive[i-1]["vx"])
         for i in range(1, len(drive))
         if drive[i-1]["fc"] and drive[i]["fc"] and drive[i-1]["fc"]-drive[i]["fc"] > 0.5]
print("\nfront_clearance 一拍内掉超过 0.5 m 的次数: %d" % len(drops))
for d in drops[:8]:
    print("   t+%6.1fs  %.2f -> %.2f m   上一拍vx %+.3f" % d)
