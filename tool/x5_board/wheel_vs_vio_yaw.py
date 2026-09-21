#!/usr/bin/env python3
"""直接比轮速里程计和 VIO 的转角，判定偏航尺度误差。

为什么这么做：之前从 map->odom 的漂移反推尺度误差，量出 4.7~10%，但那条路上
「里程计错」和「转弯时重定位变差」混在一起分不开。bag 里两种位姿都录着，
**直接比就完全绕开重定位**。

⚠️ 位姿是相机光学约定，不能取某个欧拉角当 yaw。这里用相对旋转的【旋转向量】：
模长 = 转过的总角度（与约定无关），沿主轴的分量给符号。平面运动下主轴就是重力轴。
"""
import argparse, math, sqlite3, sys
import numpy as np


def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def rotvec(R):
    """旋转矩阵 -> 旋转向量（轴×角）。"""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if abs(math.pi - th) < 1e-6:            # 接近 180°，用对称式避免除零
        A = (R + np.eye(3)) / 2.0
        ax = np.sqrt(np.maximum(np.diag(A), 0.0))
        return ax / (np.linalg.norm(ax) or 1.0) * th
    v = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return v / (2.0 * math.sin(th)) * th


def read_poses(db, topic):
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    c = sqlite3.connect(db)
    tid = c.execute("select id from topics where name=?", (topic,)).fetchone()
    if not tid:
        return []
    out = []
    for ts, data in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid[0],)):
        m = deserialize_message(bytes(data), PoseStamped)
        q, t = m.pose.orientation, m.pose.position
        out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag")
    p.add_argument("--vio-topic", default="/camera/camera/vio_image")
    p.add_argument("--wheel-topic", default="/wheel/camera_pose")
    p.add_argument("--dt", type=float, default=2.0, help="比较的时间步长(秒)")
    a = p.parse_args()

    import glob
    db = sorted(glob.glob(a.bag + "/*.db3"))[0]
    vio = read_poses(db, a.vio_topic)
    whl = read_poses(db, a.wheel_topic)
    print(f"[info] VIO {len(vio)} 条, 轮速 {len(whl)} 条")
    if len(vio) < 50 or len(whl) < 50:
        return 2

    # 用 VIO 的时间轴，最近邻取轮速（两者都由同一块板打戳）
    wt = np.array([w[0] for w in whl])
    def wheel_at(t):
        i = int(np.argmin(np.abs(wt - t)))
        return (whl[i][1], abs(wt[i] - t))

    def wheel_p(t):
        return whl[int(np.argmin(np.abs(wt - t)))][2]

    # VIO 的累计路程（20Hz 采样，逐段求和）。用 VIO 而不是编码器路程：两者的距离
    # 尺度只差约 1%，而这里它只进二阶项。
    step = np.linalg.norm(np.diff(np.stack([v[2] for v in vio]), axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(step)])

    pairs, dropped = [], []
    t0 = vio[0][0]
    i = 0
    while i < len(vio):
        j = i
        while j < len(vio) and vio[j][0] - vio[i][0] < a.dt:
            j += 1
        if j >= len(vio):
            break
        Rv1, Rv2 = vio[i][1], vio[j][1]
        Rw1, e1 = wheel_at(vio[i][0]); Rw2, e2 = wheel_at(vio[j][0])
        if max(e1, e2) > 0.15:          # 时间对不齐就跳过
            i = j; continue
        rv = rotvec(Rv1.T @ Rv2)
        rw = rotvec(Rw1.T @ Rw2)
        # 🔴 ESP32 掉压复位会把它的位姿积分清零，表现为轮速这一侧凭空跳一大截。
        # 不剔掉的话尺度拟合会被一两个窗口彻底带偏，而且不报错。
        if abs(np.linalg.norm(rw) - np.linalg.norm(rv)) > math.radians(45.0):
            dropped.append(vio[i][0] - t0)
            i = j; continue
        pairs.append((vio[i][0] - t0, rv, rw, float(cum[j] - cum[i]),
                      float(np.linalg.norm(vio[j][2] - vio[i][2])),
                      float(np.linalg.norm(wheel_p(vio[j][0]) - wheel_p(vio[i][0])))))
        i = j

    if len(pairs) < 20:
        print(f"配对只有 {len(pairs)} 组，不足")
        return 2
    # 主轴 = 所有 VIO 旋转向量的主成分（平面运动下就是重力轴）
    M = np.stack([p[1] for p in pairs])
    # 🔴 不能去均值。我们要的是「所有旋转共同的轴」= 最大化 Σ(M·a)² 的方向，
    # 那是【不去均值】的第一主方向。去均值求的是「方差最大的方向」：若这段 bag 偏向
    # 一侧转（闭环路线常见），M ≈ μ·g + 噪声，减掉均值正好把信号抹掉、剩噪声主方向，
    # 而且不会报错，只会让后面所有投影悄悄变错。
    axis = np.linalg.svd(M, full_matrices=False)[2][0]
    if np.dot(axis, M.sum(0)) < 0:
        axis = -axis
    dv = np.array([float(np.dot(p[1], axis)) for p in pairs]) * 180 / math.pi
    dw = np.array([float(np.dot(p[2], axis)) for p in pairs]) * 180 / math.pi

    big = np.abs(dv) > 2.0
    print(f"[info] {len(pairs)} 组 {a.dt}s 窗口，其中转角>2° 的 {big.sum()} 组")
    if dropped:
        print(f"  ⚠️ 剔掉 {len(dropped)} 组两侧差异>45°的窗口（多半是 ESP32 掉压复位清零了位姿积分）："
              + ", ".join(f"{d:.0f}s" for d in dropped[:8]) + (" ..." if len(dropped) > 8 else ""))
    # 主轴解释了多少能量：低于 0.95 说明运动不是平面的，投影出来的 dv/dw 不可信
    frac = float(np.sum((M @ axis) ** 2) / max(np.sum(M ** 2), 1e-12))
    print(f"[info] 主轴 = [{axis[0]:+.3f},{axis[1]:+.3f},{axis[2]:+.3f}]  解释能量 {frac:.4f}"
          + ("" if frac > 0.95 else "  ⚠️ 低于0.95，投影不可信"))
    for lbl, m in (("全部", np.ones(len(dv), bool)), ("转角>2°", big),
                   ("左转(>2°)", big & (dv > 0)), ("右转(>2°)", big & (dv < 0))):
        if m.sum() < 8:
            continue
        x, y = dv[m], dw[m]
        k = float(np.sum(x*y) / np.sum(x*x))        # 过原点最小二乘：轮速 = k × VIO
        r = float(np.corrcoef(x, y)[0, 1])
        res = y - k*x
        # 🔑 必须报标准误：窗口残差约 4°（主要来自 0.15s 的时间对齐容差乘上转速），
        # 而窗口转角才 20 来度，单条 bag 根本分不开几个百分点的差别。我据此误判过。
        se = float(res.std(ddof=1) / math.sqrt(np.sum(x*x)))
        print(f"  {lbl:12s} n={m.sum():4d}  轮速/VIO = {k:.4f} ± {se:.4f}  "
              f"(尺度误差 {100*(k-1):+.2f}% ± {100*se:.2f})  r={r:+.4f}  残差 std={res.std():.2f}°")
    print(f"\n  累计转角: VIO {np.abs(dv).sum():.0f}°  轮速 {np.abs(dw).sum():.0f}°  "
          f"比值 {np.abs(dw).sum()/max(np.abs(dv).sum(),1e-9):.4f}")

    # 🔑 把「轮距错」和「左右轮有效周长不等」分开。后者每走一米就凭空加一个固定偏航，
    # 与转不转无关，所以 s 恒为正而 dv 有正负 —— 两项可辨识。改轮距修不掉第二项。
    sp = np.array([p[3] for p in pairs])
    B_FW = 0.2083
    m = np.abs(dv) > 2.0
    if m.sum() >= 20:
        X = np.stack([dv[m], sp[m]], axis=1)
        (ka, kc), *_ = np.linalg.lstsq(X, dw[m], rcond=None)
        eps = -kc * B_FW * math.pi / 180.0
        res = dw[m] - X @ np.array([ka, kc])
        print(f"\n  两参数拟合 (n={m.sum()}, dv 与路程的相关 {np.corrcoef(np.abs(dv[m]), sp[m])[0,1]:+.2f}):")
        print(f"    轮距项   轮速/VIO = {ka:.4f} ({100*(ka-1):+.2f}%)  =>  WHEEL_BASE = {B_FW*ka:.4f} m")
        print(f"    路程项   {kc:+.3f} °/m  =>  左右轮有效周长差 {100*eps:+.3f}%  （改轮距修不掉）")
        print(f"    残差 std={res.std():.2f}°   单参数拟合残差 std="
              f"{(dw[m]-float(np.sum(dv[m]*dw[m])/np.sum(dv[m]**2))*dv[m]).std():.2f}°")

    # 🔑 常数尺度误差(轮距参数错) vs 打滑(转得快滑得多)：前者各档一样，后者随转速上升。
    # 修法完全不同 —— 常数能一次改对，打滑改参数只会在某一个转速上对。
    rate = np.abs(dv) / a.dt        # °/s
    print(f"\n  {'转速档 (°/s)':>16} {'样本':>5} {'尺度误差':>9} {'残差std':>8}")
    for lo, hi in ((1, 5), (5, 10), (10, 20), (20, 40), (40, 999)):
        m = (rate >= lo) & (rate < hi) & (np.abs(dv) > 2.0)
        if m.sum() < 5:
            continue
        x, y = dv[m], dw[m]
        k = float(np.sum(x*y) / np.sum(x*x))
        res = y - k*x
        print(f"  {lo:6.0f}~{hi if hi<999 else 999:5.0f} {m.sum():9d} {100*(k-1):+8.2f}% {res.std():7.2f}°")
    # 🔑 原地转 vs 走弧线：转弯半径 = 路程/转角。两者接地打滑机理不同（原地两轮反转、
    # 弧线两轮同向），有效轮距可能不是同一个值，而导航里两种都有。
    # ⚠️ 必须两边各分一次档：半径是用转角算的，而转角带噪声。只按 VIO 侧(dv)分档会
    # 挑中"dv 恰好偏小"的窗口，系统性低估自变量、把斜率抬高；按轮速侧(dw)分档这个
    # 偏差反向。两列趋势相反 = 趋势是分档造成的假象，不是物理。
    print(f"\n  {'转弯半径 (m)':>16} {'样本':>5} {'按VIO分档':>13} {'按轮速分档':>14}")
    for lo, hi in ((0.0, 0.15), (0.15, 0.4), (0.4, 1.0), (1.0, 1e9)):
        cells = []
        for ref in (dv, dw):
            rr = sp / np.maximum(np.abs(ref) * math.pi / 180.0, 1e-6)
            mm = (rr >= lo) & (rr < hi) & (np.abs(dv) > 2.0)
            if mm.sum() < 5:
                cells.append(('', 0)); continue
            x, y = dv[mm], dw[mm]
            kk = float(np.sum(x*y) / np.sum(x*x))
            ss = float((y - kk*x).std(ddof=1) / math.sqrt(np.sum(x*x)))
            cells.append(('%+6.2f%% ±%4.2f' % (100*(kk-1), 100*ss), mm.sum()))
        if not any(c[0] for c in cells):
            continue
        print(f"  {lo:6.2f}~{hi if hi < 1e8 else 99:5.2f} "
              f"{cells[0][1]:4d}/{cells[1][1]:<4d} {cells[0][0]:>13s} {cells[1][0]:>14s}")

    for lo, hi in ():          # 占位，保留下面共用的打印格式
        m = np.zeros(len(dv), bool)
        if m.sum() < 5:
            continue
        x, y = dv[m], dw[m]
        k = float(np.sum(x*y) / np.sum(x*x))
        res = y - k*x
        se = float(res.std(ddof=1) / math.sqrt(np.sum(x*x)))
        print(f"  {lo:6.2f}~{hi if hi < 1e8 else 99:5.2f} {m.sum():9d} "
              f"{100*(k-1):+7.2f}% ±{100*se:4.2f} {res.std():7.2f}°")

    # 平移量分档：原地转 vs 行进转，打滑机理不同
    print(f"\n  路程: 总 {sp.sum():.1f} m，窗口中位 {np.median(sp):.3f} m")

    # 🔑 距离尺度（定 WHEEL_CIRC 用）。只取几乎不转的窗口：相机相对车体有偏置，原地转会
    # 让相机自己划一段弧，而两侧用的偏置未必一致 —— 转起来这个比值就不可信了。
    # ⚠️ VIO 少报平移（量级几个百分点），所以这个比值是【上界偏松】的参考，不是真值。
    dvio = np.array([p[4] for p in pairs])
    dwhl = np.array([p[5] for p in pairs])
    st = (np.abs(dv) < 2.0) & (dvio > 0.05)
    if st.sum() >= 5:
        x, y = dvio[st], dwhl[st]
        k = float(np.sum(x*y) / np.sum(x*x))
        se = float((y - k*x).std(ddof=1) / math.sqrt(np.sum(x*x)))
        print(f"  距离尺度（{st.sum()} 个近直线窗口）: 轮速/VIO = {k:.4f} ± {se:.4f} "
              f"({100*(k-1):+.2f}%)  位移合计 VIO {x.sum():.1f}m / 轮速 {y.sum():.1f}m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
