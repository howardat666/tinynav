#!/usr/bin/env python3
"""闭环标定差速车的前馈(kff/截距)。在板上跑。

  python3 diffcar_kff_closedloop.py [最高速度] [--apply]

不用固件的开环 `o` 扫描：那个给两轮同一个 PWM，而实测左轮同 PWM 比右轮快 4.3%，
开环没人纠正，**每米偏航 7.7 度**，再叠上反向时脚轮甩头，落地跑会撞墙(2026-08-21 撞过)。
闭环下 PID 把两轮抹平(悬空实测 6257 计数差 1 个)，而且稳态 PWM 正好就是 kff 该拟合的量。

每档正反各跑一次，净位移约为零，最大单向行程 = 速度 x 保持时间(默认 0.30 m/s x 1.6 s = 0.48 m)。
"""
import re
import sys
import time

sys.path.insert(0, '/root/car')
import carlib  # noqa: E402

HOLD_S = 1.6          # 保持时间。悬空实测 c 0.30 约 1.5 s 收敛，取样只用最后 0.6 s
SAMPLE_S = 0.6
V_ABORT = 9.9         # 低压保护 9.60，留余量


def parse_e(lines):
    """从 e 的回复里取 (左,右) 的 (实测, PWM)，以及电压。需要固件报了 PWM。"""
    out = []
    volt = None
    for line in lines:
        m = re.search(r'实测=(-?[\d.]+)m/s 目标=(-?[\d.]+) PWM=(-?\d+)', line)
        if m:
            out.append((float(m.group(1)), float(m.group(2)), int(m.group(3))))
        v = re.search(r'当前=([\d.]+)V', line)
        if v:
            volt = float(v.group(1))
    return out, volt


def fit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    a = sxy / sxx
    b = my - a * mx
    pred = [a * x + b for x in xs]
    ss = 1 - sum((y - p) ** 2 for y, p in zip(ys, pred)) / sum((y - my) ** 2 for y in ys)
    return a, b, ss, max(abs(y - p) for y, p in zip(ys, pred))


def hold(car, v):
    """闭环保持 v，返回取样窗口内每轮的 (实测均值, PWM均值)。"""
    car.drain()
    t0 = last = time.time()
    samples = []
    while time.time() - t0 < HOLD_S:
        now = time.time()
        if now - last > 0.05:
            car.send('c %.4f' % v)
            last = now
        if now - t0 > HOLD_S - SAMPLE_S:
            got, volt = parse_e(car.ask('e', 0.18))
            if len(got) >= 2:
                samples.append((got[0], got[1], volt))
        else:
            time.sleep(0.01)
    if not samples:
        return None
    n = len(samples)
    lv = sum(s[0][0] for s in samples) / n
    lp = sum(s[0][2] for s in samples) / n
    rv = sum(s[1][0] for s in samples) / n
    rp = sum(s[1][2] for s in samples) / n
    vmin = min(s[2] for s in samples if s[2] is not None)
    return (lv, lp), (rv, rp), vmin, n


def main():
    vmax = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 0.30
    apply_it = '--apply' in sys.argv
    car = carlib.Car()
    time.sleep(0.6)
    car.drain()

    got, v0 = parse_e(car.ask('e', 0.8))
    if not got or len(got[0]) != 3:
        sys.exit('固件没在 e 里报 PWM —— 先烧带 PWM 报告的固件(diffcar_esp32/ota.sh)')
    if v0 is None or v0 < V_ABORT:
        sys.exit('电压 %s 低于 %.2fV 的中止线' % (v0, V_ABORT))
    print('电压 %.2fV，最高速度 %.2f m/s，单向行程 %.2f m' % (v0, vmax, vmax * HOLD_S))

    speeds = [round(vmax * f, 4) for f in (0.15, 0.25, 0.4, 0.55, 0.7, 0.85, 1.0)]
    rows = []
    print('\n  目标   左实测  左PWM   右实测  右PWM   电压   样本')
    for v in speeds:
        acc = []
        for sign in (+1, -1):                    # 正反各一次，净位移约零，顺带抵消正反不对称
            out = hold(car, sign * v)
            car.send('c 0')
            time.sleep(0.35)
            if out:
                (lv, lp), (rv, rp), vmin, n = out
                acc.append((abs(lv), abs(lp), abs(rv), abs(rp), vmin, n))
        car.send('s')
        time.sleep(0.4)
        if len(acc) < 2:
            print('  %.3f  取样失败，跳过' % v)
            continue
        m = [sum(a[i] for a in acc) / len(acc) for i in range(4)]
        vmin = min(a[4] for a in acc)
        rows.append((v, m[0], m[1], m[2], m[3]))
        print('  %.3f  %6.3f  %6.1f  %6.3f  %6.1f  %5.2f  %d'
              % (v, m[0], m[1], m[2], m[3], vmin, sum(a[5] for a in acc)))
        if vmin < V_ABORT:
            print('  电压掉到 %.2fV，停止' % vmin)
            break

    use = [r for r in rows if r[1] > 0.03 and r[3] > 0.03]
    if len(use) < 4:
        car.kill()
        sys.exit('可用点不足 %d 个' % len(use))
    al, bl, sl, el = fit([r[1] for r in use], [r[2] for r in use])
    ar, br, sr, er = fit([r[3] for r in use], [r[4] for r in use])
    print('\n左轮: kff=%.1f 截距=%.2f  R2=%.5f 最大残差 %.2f PWM' % (al, bl, sl, el))
    print('右轮: kff=%.1f 截距=%.2f  R2=%.5f 最大残差 %.2f PWM' % (ar, br, sr, er))
    print('\n  x %.1f %.1f\n  i %.0f %.0f\n  w' % (al, ar, round(bl), round(br)))

    if apply_it:
        for cmd in ('x %.1f %.1f' % (al, ar), 'i %.0f %.0f' % (round(bl), round(br)), 'w'):
            print('>> ' + cmd)
            for line in car.ask(cmd, 0.8):
                print('   ' + line)
    car.kill()


if __name__ == '__main__':
    main()
