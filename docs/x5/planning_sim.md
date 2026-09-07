# 规划仿真（planning lab）

改一版规划就推板子跑一趟、再读日志，一轮 20 分钟；同样的东西在仿真里是几十秒。
这个仿真器来自上游（PR #222 / #224 / #229 / #230，已合入 `upstream/main`），
搬到 `x5/wheel-nav` 上做了四处适配，见下面「适配了什么」。

## 它是什么

**闭环，跑的是真代码。** `tool/simulator/ros_planning_web.py` 是一个 8 Hz 的循环：

```
积分 cmd_vel → 更新车位姿 → 渲染合成深度图
  → 发 /slam/depth + PoseStamped + /control/target_pose + camera_info
  → 真的 planning_node 和 cmd_vel_control 子进程（不是仿的）
  → 收 /cmd_vel、/planning/trajectory_path、obstacle_mask、occupancy_grid
  → 回到第一步
```

场景是长方体拼的，也能直接加载 `tinynav_db/maps` 里的真地图（`map_volume.py`）。

## 怎么跑

```bash
scripts/run_planning_sim_docker.sh &          # 起仿真（宿主机没有 uv，走 docker）
# 浏览器开 http://localhost:8766 —— 已经预载好场景在跑，左边 Scene 下拉框换场景
python3 tool/simulator/scenarios.py           # 无头跑全部场景，打印判据
python3 tool/simulator/scenarios.py wall_ahead   # 只跑一个
```

起来就已经是**板上那一套**，不用手工配：

- 规划参数取自 `app_start.sh`（`x5_presets.BOARD_ENV`）：`span>=0.2`、`dilation=0`、
  `low_obs=on (0.05~0.25 m, <1.5 m, >=2 pts)`、`cam_h=0.18`、`TINYNAV_ALLOW_REVERSE=0`。
  启动日志里 `obstacle: raycast ...` 和 `reverse: ...` 两行可以核对。
- 机器人默认 `diffcar`，几何和板上一致。
- 相机默认 `match` 档，见下。
- 预载场景默认 `wall_ahead`，用 `TINYNAV_SIM_SCENE=chair_legs` 换。

浏览器里能看到俯视图、深度图、障碍栅格和选中的轨迹，可以拖着改场景 —— 调参时比看数字快。
换场景选下拉框再点 **Load scene**，它会把 planning 和 control 都换新的并自动跑起来。

## 相机：对齐的是角分辨率，不是像素数

实机深度 544×640、fx=fy=309.5，板上 `TINYNAV_RAYCAST_STEP=5` —— 也就是每
`5/309.5 = 0.01615 rad` 取一条射线。**决定障碍图有多少洞的是角分辨率**（见
`dilation-is-patching-raycast-holes`），所以缩放渲染时必须让 step 跟着变。

| `TINYNAV_SIM_CAMERA` | 分辨率 | step | 角分辨率 | 渲染 | 用途 |
|---|---|---|---|---|---|
| `match`（默认） | 218×256 fx=123.8 | 2 | 0.9256 °/射线 | 25 ms/帧 | 和板上逐位一致，日常用这个 |
| `full` | 544×640 fx=309.5 | 5 | 0.9256 °/射线 | 263 ms/帧 | 逐像素一致，仿真掉到约 3.8 Hz，复核用 |
| `fast` | 160×100 fx=80/50 | 5 | 3.5810 °/射线 | 6 ms/帧 | 上游默认，只适合看流程通不通 |

板上是 `0.9256 °/射线`，`match` 就是照这个数配出来的。

## 场景与判据

| 场景 | 内容 | 判据 |
|---|---|---|
| `open_run` | 空地直行 3 m | 必须到达。任何改动都不许把它弄坏 |
| `corridor` | 净宽 0.8 m 走廊 | 必须到达 |
| `narrow_gap` | 0.9 m 的缝 | 必须到达 |
| `wall_ahead` | 前方 0.2 m 一堵宽墙、两侧开阔、目标在右后方 | 到达**且变向 ≤ 2 次** |
| `dead_end` | 三面围住 | 不许倒车、变向 ≤ 4 次（倒车默认关，正确行为是停住报无解） |
| `chair_legs` | 五条 5 cm 的椅子腿，离地 0~0.35 m | 必须到达。专门压 `low_obs` 那条按离地高度判的通路 |
| `doorway_turn` | 0.9 m 门洞，出去之后目标在右侧 | 必须到达。贪心的终点距离在门里就想往右切 |

场景定义在 `tool/simulator/x5_presets.py`，**web 下拉框和无头套件共用这一份** ——
两份定义迟早分叉。每个场景自带 `max_flips`（变向上限）。

`wall_ahead` 就是 2026-08-27 板上摆头那一幕的复刻，`dead_end` 是楔住那一幕。
变向次数是摆头的判据 —— 摆头的特征就是 `omega` 反复换号。

## 适配了什么（相对上游）

1. **模块名**：上游叫 `tinynav.core.robot_specs`，x5 叫 `robot_config`；默认平台从 `go2` 换成 `diffcar`。
2. **位姿类型**：x5 的 planning/control 全程用相机光学约定的 `PoseStamped`，不吃 `Odometry`。
   仿真新增 `/slam/pose_sim`，**和 depth 用同一个 stamp** —— planning 那边是精确时间戳同步。
3. **target 的 QoS**：x5 用 `TRANSIENT_LOCAL` 订阅 `/control/target_pose`。VOLATILE 的发布者
   和 TRANSIENT_LOCAL 的订阅者在 DDS 里**不兼容、根本不建连**，表现是 planning 一直打
   `No target pose` 而一个错都不报。仿真改成 latched 发布。
4. **子进程解释器**：上游写死 `uv run`；改成用跑仿真那个解释器（`TINYNAV_SIM_PYTHON` 可覆盖），
   否则会在挂载进来的仓库里现建一个几 GB 的 venv。
   子进程还要显式带 `-p robot_type:= -p pose_topic:= -p pose_sync:=exact`，x5 不读 `ROBOT_TYPE` 环境变量。

## 局限（别拿仿真的结论当实车结论）

- **没有 map_node**，所以没有重定位、没有 POI、没有 `/mapping/global_plan`。target 是直接发的。
  想评估上游 PR #226（跟全局路线打分）就得先让仿真发一条合成的 global plan。
- **深度是理想的** —— 没有空洞、没有噪声、没有 544×640 的射线取样密度差异。
  「探测距离/召回率」类结论必须在实车 bag 上复核（见 `dilation-is-patching-raycast-holes` 的教训）。
- **没有板子的 CPU 排队**。周期、时间戳滞后、关键帧饿死这些都测不出来。
- 仿真里默认相机高 0.18 m（和 Looper 实装一致），上游默认 0.45 m。

## 一个测试台自己的坑（踩过两次）

**成套跑必须把 planning 和 control 都换新的。** `planning_node` 会带着上一个场景的
障碍图，`cmd_vel_control` 会带着上一个场景累积的时间参数化路径参考（实测 `idx` 涨到
500 多）。少了这个隔离，下一个场景表现成「一直不动」或「130 次变向」，**看着完全像
规划器的 bug**。`/api/load-scene` 已经包含这个隔离。

**预热期间要冻住车。** 规划器重启要付一次 numba 编译，那几秒里车已经开走一米多，
场景之间的用时和路程就不可比了。`load-scene` 带 `freeze: true`，等它发出第一条非退化
轨迹之后再 `POST /api/freeze {"frozen": false}` 放行。加上冻结之后 `doorway_turn`
从「通过」变成「141 次变向、到不了」—— 之前是预热期间蒙过去的。

---

# 2026-09-02 更新：两个自己的坑 + 判别力的上限

## 1. 🔴 三个可视化订阅是 RELIABLE，和发布端不兼容

`planning_node` 的可视化话题在 2026-09-02 全改成 `_viz_qos` = **BEST_EFFORT**
（为了不让跟不上的 web 后端把规划环阻塞住）。而 `ros_planning_web.py` 里

```python
self.create_subscription(PointCloud,   "/planning/footprint",      ..., 10)   # 默认 RELIABLE
self.create_subscription(OccupancyGrid,"/planning/obstacle_mask",  ..., 10)
self.create_subscription(OccupancyGrid,"/planning/occupancy_grid", ..., 10)
```

**BEST_EFFORT 发布 + RELIABLE 订阅 = QoS 不兼容、零投递、完全静默。**
判据：`ros2 topic info -v <话题>` 直接打印两端的 Reliability。已全部改成 BEST_EFFORT。

⚠️ **收回一个过度归因**：我一开始说"这把碰撞判据弄瞎了"（footprint 空 →
`clearance_to_objects` 返回 inf → 最小净空打成 `nan`、压进障碍恒 0）。那次探测是在
`freeze: True` **没解冻**的仿真上做的，冻着本来就没有新深度、planning 也不发 footprint。
修之前那套场景其实是给出了有限净空的。**QoS 该改，但别拿它解释别的现象。**

🔑 顺带：改 QoS 的影响面**不止 UI**。`tool/` 和 `tests/` 里订阅同样话题的离线工具会静默
失效，而它们不像界面那样"一看就知道黑了"。同一天在 `tool/x5_board/obstacle_band_probe.py`
也查出一个显式写死 `ReliabilityPolicy.RELIABLE` 的（收不到任何东西），已修。
**改 QoS 之后要 `grep -rn create_subscription` 整个仓库。**

## 2. 判据写死了「倒车关」

```python
# 旧：倒车默认关，所以任何真正的倒车指令都是问题
ok = ok and len(revs) == 0
```

`TINYNAV_ALLOW_REVERSE=1` 之后，任何用到倒车的场景都会**按判据自己的定义**失败 ——
`doorway_turn` 就是这么"挂"的（到达 69.9 s、净空 1.038 m、压进 0，唯一的失败原因是
发过倒车指令）。已改成按环境变量分流：

```python
if os.environ.get("TINYNAV_ALLOW_REVERSE", "0") != "1":
    ok = ok and len(revs) == 0        # 关着时任何倒车都是问题
# 开着时倒车是脱困手段，判据只看有没有压进障碍
```

汇总行也多打一列**倒车次数** —— 原来 `reverse_cmds` 算了但没打，所以「这个场景到底用了
几次倒车」看不见。

## 3. 🔴 判别力的上限：`dead_end` 在抛硬币

同一个场景、同一份代码，五次运行：

| 配置 | 变向 | 倒车 | 结果 |
|---|---|---|---|
| FIFO + 倒车关 | 19 | 0 | FAIL |
| FIFO + 倒车开 | **4** | **1** | OK |
| 取最新 + 倒车开 第1遍 | 6 | 0 | FAIL |
| 取最新 + 倒车开 第2遍 | 18 | 0 | FAIL |
| 取最新 + 倒车开 单跑 | 19 | 1 | FAIL |

原因：`_should_retreat` 要 `front_blocked AND (escape_clear < 门限 OR escape_age > 门限)`，
而 `dead_end` 里车离前墙 0.55 m、两侧墙 ±0.7 m，**还转得开**，第二个条件成不成立全看
障碍图那一刻的状态。

🔑 **所以在这个场景上仿真的判别力就是 ±1 —— 拿它判任何改动都会得到噪声。**
真车楔住是 `fwd_ok=0/90` 连续 16 秒（几何完全不同、条件硬得多），
所以这个抛硬币**不能**推断真车行为；真车那次已单独验过倒车会触发。

## 4. 由此得到的使用纪律

- **单次运行不能定罪。** 有一次「取最新」跑出 `open_run 42.2 s`（基线 10.7 s）、
  `corridor 只走 0.21 m`，第二遍和单跑全部正常 —— **不可复现**。
- **要 A/B 就跑两遍，而且比 8 个稳定场景，不比 `dead_end`。**
  「取最新」两遍下来那 8 个和基线逐位相同（10.8/10.8/10.8/9.5/9.6/未到/12.5/8.9 s）。
- **不要同时开两个仿真服务** —— 会撞 8766 端口，两边结果都废。
- 起服务后**等到日志出现 `traj published` 再跑场景**，否则第一个场景在预热期里跑。

---

## 2026-09-03：新增三个「全局路线自己穿过障碍」的场景

旧的 9 个场景的 `route` **全部是正确的**（已经绕开障碍），所以它们一个都覆盖不到板上
占 46% 的那一类失效。新增的三个把路线故意画穿障碍，起点也刻意给偏 + 给转角
（正对着能一条直线开过去的起点测不出转向期间的判据）：

| 场景 | 起点 / 偏航 | 考什么 |
|---|---|---|
| `route_thru_wall` | (−0.4,−0.5) / +40° | 路中间后来放了一堵墙，路线直穿它 |
| `route_grazes_leg` | (−0.5,+0.7) / −50° | 路线擦着椅腿（`route_clear`≈0.10），让 5~10 cm 就能过 —— 板上占比最大的一档 |
| `route_thru_blocked_door` | (−0.3,−0.2) / +25° | ⚠️ 预期难，是解钉的**上限探针**：进展项仍指着被堵死的那扇门 |

顺带修了 `scene_catalog()` 对没写 `title` 的场景会 `KeyError`（`corridor` 就没写，
web 下拉框一直是坏的）。

⚠️ **仿真和板上不是同一套速度**：仿真 `vx<=0.25 yaw<=0.6`，板上 `vx<=0.40`。
轨迹长度差 1.6 倍，直接影响代价项之间的相对权重，**跨平台比数字之前先核这一行启动日志**。

### main 上的仿真更新（`#240`，2026-09-03 合入）

main 走的是另一条路线：`tool/simulator/offline_planning_web/`（浏览器里选场景）+
重写过的 `ros_planning_web.py`，并且**删掉了 `scenarios.py` 和 `x5_presets.py`**
（那两个是 x5 独有的无头跑法）。两套架构不同，**没有直接可拉的东西** ——
要么保留无头这套（回归判据能进 CI），要么整体换成 web 那套。暂不动。

## 🔴 仿真的深度没有噪声 —— 有一类参数它结构上调不了（2026-09-04）

`tool/simulator/planning_scene.py` / `map_volume.py` 里搜 `noise|random|jitter|sigma`
**零命中**：渲染出来的深度是几何精确的。

于是**任何"取决于噪声落在量化边界哪一侧"的参数，在仿真里都没有区分力，而且会给出相反的
结论**。已经踩到的一次：

| `TINYNAV_GRID_OFFSET_Z` | 启动日志的门限 | 仿真 `chair_leg_low_head_on`（65 mm 腿） | 真实数据 `scene_chair.npz` 5~10 cm 格 |
|---|---|---|---|
| 0.15 | 50 mm | 绕开（净空 +35 mm） | 8 格 @ 0.68 m |
| 0.1625 | 63 mm | **压上去**（−20 mm） | **27 格 @ 0.53 m** |

两边**方向完全相反**。真实数据那组才算数 —— 板上地面单帧在 5×5 cm 格内的高度跨度实测
max 89 mm，而仿真是 0。

**判据：凡是和"地面噪声 vs 体素层边界"有关的参数（z 波段下界、跨度门限、grid_offset_z），
一律用 `tool/x5_board/replay_obstacle_map.py` + `data/obstacle_scenes/*.npz` 扫，不要用仿真。**
仿真管的是规划行为（绕不绕、摆不摆头、到不到）。
