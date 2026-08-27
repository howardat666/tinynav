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
