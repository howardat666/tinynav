#!/usr/bin/env python3
"""从 map_node 日志回放 map->odom 拟合，用来离线 A/B 拟合侧的改动。

为什么能这么做：日志里 `reloc pose` + `reloc odom` 成对给出「这一个解观测到的
map->odom」，而拟合就是把最近 100 条这样的观测按权重平均。所以整条拟合链路不依赖
图像、地图和 ROS，纯文本就能重放。

⚠️ 只重建 SE(2)+z：日志给的旋转只有 yaw。对这里要测的东西够用 —— 失效模式是
yaw 在两个盆地之间翻，而 tilt 有单独的门。绝对数值不能和板上的 SE(3) 解逐位对齐。
"""
import argparse, math, re, statistics as st, sys

RE_POSE = re.compile(r'reloc pose: t=(\d+) cam_map=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] '
                     r'yaw_map=([-+\d.]+)deg .*ratio=([\d.]+)')
RE_ODOM = re.compile(r'reloc odom: t=(\d+) cam_odom=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] '
                     r'yaw_odom=([-+\d.]+)deg')


def parse(path):
    """→ [(ts_ns, tx, ty, tyaw_deg, ratio)]，即每个解观测到的 map->odom。"""
    obs, pend = [], None
    for ln in open(path, errors='replace'):
        m = RE_POSE.search(ln)
        if m:
            pend = dict(ts=int(m.group(1)), mx=float(m.group(2)), my=float(m.group(3)),
                        myaw=float(m.group(5)), ratio=float(m.group(6)))
            continue
        m = RE_ODOM.search(ln)
        if m and pend and int(m.group(1)) == pend['ts']:
            d = math.radians(float(m.group(5)) - pend['myaw'])
            c, s = math.cos(d), math.sin(d)
            obs.append((pend['ts'],
                        float(m.group(2)) - (c * pend['mx'] - s * pend['my']),
                        float(m.group(3)) - (s * pend['mx'] + c * pend['my']),
                        (float(m.group(5)) - pend['myaw'] + 180) % 360 - 180,
                        pend['ratio'],
                        # 原始量：地图里测到的相机位置 / 里程计里的相机位置 / 里程计偏航
                        pend['mx'], pend['my'], float(m.group(2)), float(m.group(3)),
                        float(m.group(5))))
            pend = None
    return obs


def solve(cons):
    """闭式加权平均：平移直接加权，yaw 走单位向量平均（不能算术平均，会在 ±180 翻车）。"""
    w = [c[4] for c in cons]
    s = sum(w) or 1e-9
    tx = sum(c[1] * c[4] for c in cons) / s
    ty = sum(c[2] * c[4] for c in cons) / s
    sy = sum(math.sin(math.radians(c[3])) * c[4] for c in cons) / s
    cy = sum(math.cos(math.radians(c[3])) * c[4] for c in cons) / s
    return tx, ty, math.degrees(math.atan2(sy, cy))


def resid(fit, cons):
    tx, ty, yaw = fit
    return [(math.hypot(c[1] - tx, c[2] - ty),
             abs((c[3] - yaw + 180) % 360 - 180)) for c in cons]


def replay(obs, half_life_s, stale_resid_m, stale_keep, window=100, half_life_deg=0.0):
    """逐条喂进去，复现 map_node 每来一个解就重拟合一次的行为。"""
    live, out, streak, purges = [], [], 0, 0
    prev = None
    for o in obs:
        # 🔑 用【并入这一条之前】的变换去预测它 —— 纯预测量，这就是机器人此刻
        # 相信自己在地图里的位置，和重定位直接测出来的位置，差了多远。
        pred_err = float('nan')
        if prev is not None:
            a = math.radians(prev[2])
            ca, sa = math.cos(a), math.sin(a)
            px = ca * o[5] - sa * o[6] + prev[0]      # T @ cam_map
            py = sa * o[5] + ca * o[6] + prev[1]
            pred_err = math.hypot(px - o[7], py - o[8])
        live.append(o)
        newest = o[0]
        # 权重 = 内点率 × 时间半衰，和 map_node 里逐字一致
        # 转角衰减：一条约束过不过时，取决于「从那时起车转了多少度」而不是「过了多少秒」。
        # 车停着时旧约束依然有效（不该衰减）；原地转 90° 只要 3 秒，旧约束已经偏了好几度。
        cum = None
        if half_life_deg > 0:
            cum, acc = {}, 0.0
            for j in range(len(live) - 1, 0, -1):
                acc += abs((live[j][9] - live[j-1][9] + 180) % 360 - 180)
                cum[live[j-1][0]] = acc
            cum[live[-1][0]] = 0.0
        cons = []
        for c in live[-window:]:
            wgt = c[4]
            if half_life_deg > 0:
                wgt *= 0.5 ** (cum.get(c[0], 0.0) / half_life_deg)
            elif half_life_s > 0:
                wgt *= 0.5 ** (((newest - c[0]) / 1e9) / half_life_s)
            cons.append((c[0], c[1], c[2], c[3], wgt))
        fit = solve(cons)
        r = resid(fit, cons)
        k = min(5, len(r))
        recent_t = st.median([x[0] for x in r[-k:]])
        d_t = math.hypot(fit[0] - prev[0], fit[1] - prev[1]) if prev else 0.0
        out.append(dict(ts=o[0], fit=fit, n=len(cons), d_t=d_t, pred_err=pred_err,
                        recent_t=recent_t, all_t=st.median([x[0] for x in r])))
        prev = fit
        if stale_resid_m > 0:
            if recent_t > stale_resid_m:
                streak += 1
                if streak >= 2:
                    keep = set(c[0] for c in sorted(live)[-stale_keep:])
                    dropped = len(live) - len(keep)
                    if dropped > 0:
                        live = [c for c in live if c[0] in keep]
                        purges += 1
                    streak = 0
            else:
                streak = 0
    return out, purges


def lag_events(out, thresh):
    """连续 recent 残差超限的段 = 「解跟不上现在这一片」的持续时间。"""
    ev, start = [], None
    for r in out:
        if r['recent_t'] > thresh:
            if start is None:
                start = r['ts']
        elif start is not None:
            ev.append((r['ts'] - start) / 1e9)
            start = None
    if start is not None:
        ev.append((out[-1]['ts'] - start) / 1e9)
    return ev


RE_FIT = re.compile(r'map->odom: t=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw=([-+\d.]+)deg')


def parse_logged_fits(path):
    """日志里真实拟合出的 map->odom，用来校验回放忠不忠实。"""
    out = []
    for ln in open(path, errors='replace'):
        m = RE_FIT.search(ln)
        if m:
            out.append((float(m.group(1)), float(m.group(2)), float(m.group(4))))
    return out


def validate(path, out):
    """回放的解 vs 板上真实解，逐次对齐。差太大就说明这个回放不能用来下结论。"""
    real = parse_logged_fits(path)
    n = min(len(real), len(out))
    if n < 10:
        return None
    dy = [abs((out[i]['fit'][2] - real[i][2] + 180) % 360 - 180) for i in range(n)]
    dt = [math.hypot(out[i]['fit'][0] - real[i][0], out[i]['fit'][1] - real[i][1])
          for i in range(n)]
    return n, q(dy, .5), q(dy, .9), q(dt, .5), q(dt, .9)


def q(v, f):
    v = sorted(v)
    return v[min(int(len(v) * f), len(v) - 1)] if v else float('nan')


def main():
    p = argparse.ArgumentParser()
    p.add_argument("log", nargs="+")
    p.add_argument("--half-life", type=float, default=20.0)
    p.add_argument("--lag-threshold", type=float, default=1.0,
                   help="recent 残差超过它就算「跟不上」，默认 1.0m")
    p.add_argument("--stale-keep", type=int, default=10)
    p.add_argument("--windows", default="", help="逗号分隔的窗口大小；给了就扫窗口而不是扫门限")
    p.add_argument("--half-life-deg", default="", help="逗号分隔的转角半衰期(度)；给了就扫它")
    p.add_argument("--combo", default="",
                   help="逗号分隔的 窗口:转角半衰期 组合，如 100:15,5:0,5:15")
    p.add_argument("--arms", default="0,1.0",
                   help="逗号分隔的 stale_resid 门限；0=关（现状）")
    a = p.parse_args()

    for path in a.log:
        obs = parse(path)
        if len(obs) < 20:
            print(f"{path}: 只配到 {len(obs)} 个解，跳过")
            continue
        span = (obs[-1][0] - obs[0][0]) / 1e9
        print(f"\n===== {path.split('/')[-1]} =====")
        print(f"解 {len(obs)} 个 / {span:.0f}s")
        base, _ = replay(obs, a.half_life, 0.0, a.stale_keep)
        v = validate(path, base)
        if v:
            print(f"  回放忠实度（vs 日志里的真实拟合，{v[0]} 次对齐）: "
                  f"yaw 差 p50={v[1]:.2f}° p90={v[2]:.2f}°  平移差 p50={v[3]:.3f}m p90={v[4]:.3f}m")
        else:
            print("  ⚠️ 日志里的拟合太少，无法校验忠实度 —— 下面的数字只能当参考")
        print(f"{'门限':>8} {'拟合':>5} {'清空':>4} {'位置误差p50':>11} {'p90':>7} {'max':>7} {'>0.5m':>6} {'>2m':>5} "
              f"{'recent残差p50':>13} {'p90':>7} "
              f"{'d_t>0.3占比':>11} {'跟不上段数':>10} {'最长段':>8} {'累计':>8}")
        if a.combo:
            # 窗口和转角半衰期叠加：两者单独都有效，但叠起来可能过猛（窗口只剩 5 条时
            # 再按转角衰减，等于进一步砍掉证据）。要上板的组合必须这样直接测。
            combos = [(0.0, int(w), float(d)) for w, d in
                      (x.split(":") for x in a.combo.split(","))]
        elif a.half_life_deg:
            combos = [(0.0, 100, float(d)) for d in a.half_life_deg.split(",")]
        elif a.windows:
            combos = [(0.0, int(w), 0.0) for w in a.windows.split(",")]
        else:
            combos = [(float(x), 100, 0.0) for x in a.arms.split(",")]
        for thr, win, hld in combos:
            out, purges = replay(obs, a.half_life, thr, a.stale_keep,
                                 window=win, half_life_deg=hld)
            rt = [r['recent_t'] for r in out]
            dt = [r['d_t'] for r in out]
            ev = lag_events(out, a.lag_threshold)
            label = (f"{win}+{hld:.0f}°" if a.combo else
                     f"转角{hld:.0f}°" if a.half_life_deg else
                     f"窗口{win}" if a.windows else
                     ("关(现状)" if thr == 0 else f"{thr:.1f}m"))
            pe = [r['pred_err'] for r in out if r['pred_err'] == r['pred_err']]
            print(f"{label:>8} {len(out):5d} {purges:4d} {q(pe,.5):10.3f}m {q(pe,.9):6.2f}m "
                  f"{max(pe) if pe else 0:6.2f}m "
                  f"{100*sum(1 for x in pe if x>0.5)/max(len(pe),1):5.1f}% "
                  f"{100*sum(1 for x in pe if x>2.0)/max(len(pe),1):4.1f}% "
                  f"{q(rt,.5):12.3f}m {q(rt,.9):6.2f}m "
                  f"{100*sum(1 for x in dt if x>0.3)/len(dt):10.1f}% "
                  f"{len(ev):10d} {max(ev) if ev else 0:7.1f}s {sum(ev):7.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
