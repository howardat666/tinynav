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
python3 tool/simulator/scenarios.py           # 无头跑全部场景，打印判据
python3 tool/simulator/scenarios.py wall_ahead   # 只跑一个
```

浏览器开 `http://localhost:8766` 能看到俯视图、深度图、障碍栅格和选中的轨迹，
可以拖着改场景 —— 调参时比看数字快。

## 场景与判据

| 场景 | 内容 | 判据 |
|---|---|---|
| `open_run` | 空地直行 3 m | 必须到达。任何改动都不许把它弄坏 |
| `corridor` | 净宽 0.8 m 走廊 | 必须到达 |
| `narrow_gap` | 0.9 m 的缝 | 必须到达 |
| `wall_ahead` | 前方 0.2 m 一堵宽墙、两侧开阔、目标在右后方 | 到达**且变向 ≤ 2 次** |
| `dead_end` | 三面围住 | 不许倒车、变向 ≤ 4 次（倒车默认关，正确行为是停住报无解） |

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
