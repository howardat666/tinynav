# TinyNav RTK MVP

目标：在 TinyNav 原有视觉/IMU 定位基础上接入 RTK，用于室外采集、建图记录和导航定位修正。

当前实现分三层：

- 采数据：录 bag 时把 RTK topic 一起录进去。
- 建图：默认仍使用 TinyNav 视觉关键帧和 VIO；RTK 模式可用 `/slam/odometry_fused` 作为 keyframe odom，并额外保存 `/rtk/odom` 轨迹。
- 导航：用 `rtk_fusion_node.py` 融合 `/slam/odometry` 和 `/rtk/odom`，输出 `/slam/odometry_fused`，控制节点可切到 fused odom。

## 0. RTK 输入

默认串口：

```text
/dev/ttyCH341USB0
```

默认波特率：

```text
115200
```

运行前传入 NTRIP 账号和密码：

```bash
export TINYNAV_NTRIP_USER="your-user"
export TINYNAV_NTRIP_PASSWORD="your-password"
```

caster host、端口、mountpoint、initial GGA 有默认值；现场需要更换时再覆盖：

```bash
export TINYNAV_NTRIP_HOST="your-caster-host"
export TINYNAV_NTRIP_PORT="2101"
export TINYNAV_NTRIP_MOUNTPOINT="your-mountpoint"
export TINYNAV_NTRIP_INITIAL_GGA="your-gga-sentence"
```

单独启动 RTK bridge：

```bash
bash /tinynav/scripts/run_rtk.sh
```

不用 NTRIP，只解析串口 NMEA：

```bash
uv run python /tinynav/rtk/rtk_bridge_node.py --ros-args -p ntrip_enabled:=false
```

主要输出：

- `/fix`
- `/heading`
- `/vel`
- `/time_reference`
- `/rtk/odom`
- `/rtk/path`
- `/rtk/status`

## 1. 采数据

先启动 RTK bridge：

```bash
export TINYNAV_NTRIP_USER="your-user"
export TINYNAV_NTRIP_PASSWORD="your-password"
bash /tinynav/scripts/run_rtk.sh
```

再启动相机和录包流程。`scripts/run_rosbag_record.sh` 已经额外记录必要 RTK topic：

- `/fix`
- `/rtk/odom`
- `/rtk/status`
- `/rtk/fusion_status`

录包命令保持原来的用法：

```bash
bash /tinynav/scripts/run_map_record.sh
```

或者只调用录包脚本：

```bash
bash /tinynav/scripts/run_rosbag_record.sh
```

检查 RTK 是否正在被录：

```bash
ros2 topic echo /rtk/status
ros2 topic echo /rtk/odom
```

## 2. 建图

普通建图仍然使用 TinyNav 原来的视觉关键帧、深度和 VIO：

```bash
bash /tinynav/scripts/run_rosbag_build_map.sh
```

RTK 融合建图会额外启动 `rtk_fusion_node.py`，并让 `build_map_node.py` 用最近的 `/slam/odometry_fused` 替代 raw keyframe odom：

```bash
USE_RTK_FUSED_MAPPING=1 bash /tinynav/scripts/run_rosbag_build_map.sh
```

RTK 融合建图的作用：

- 特征少或 VIO 漂移明显时，用 RTK 位置修正 keyframe pose。
- 仍然保留视觉 loop closure 和 pose graph。
- 不是经纬度地图；地图坐标仍然是 TinyNav 局部米制坐标。

无论是否启用融合建图，`build_map_node.py` 都会订阅 `/rtk/odom`，并在地图目录里保存：

```text
rtk_continuous_odom.npy
```

融合建图还会保存使用统计：

```text
rtk_mapping_stats.npy
```

建图完成后检查：

```bash
ls /tinynav/output/map_go2_looper/rtk_continuous_odom.npy
ls /tinynav/output/map_go2_looper/rtk_mapping_stats.npy
```

## 3. 导航

RTK 导航启动前先设置账号密码：

```bash
export TINYNAV_NTRIP_USER="your-user"
export TINYNAV_NTRIP_PASSWORD="your-password"
```

启动 RTK 导航 MVP：

```bash
bash /tinynav/scripts/run_navigation_rtk.sh /tinynav/output/map_go2_looper
```

默认打开 6 个 tmux pane：

- pane 0：RTK bridge
- pane 1：RTK/VIO fusion
- pane 2：perception node
- pane 3：planning node
- pane 4：map node
- pane 5：cmd_vel_control，使用 `/slam/odometry_fused`

Jetson 上默认不启动 RViz 和 POI publisher，减少资源占用。需要时手动打开：

```bash
RUN_RVIZ=1 RUN_POIS=1 bash /tinynav/scripts/run_navigation_rtk.sh /tinynav/output/map_go2_looper
```

融合节点：

- 输入 `/slam/odometry`
- 输入 `/rtk/odom`
- 输出 `/slam/odometry_fused`
- 输出 `/rtk/fusion_status`

控制节点使用：

```bash
uv run python /tinynav/tinynav/platforms/cmd_vel_control.py --ros-args \
  -p odom_topic:=/slam/odometry_fused
```

默认控制节点仍使用 `/slam/odometry`，所以不带 RTK 的旧流程不受影响。

## 4. 当前融合策略

RTK bridge 会解析位置、heading、速度和 GNSS 时间。

`rtk_fusion_node.py` 当前只使用 RTK 位置做融合：

- 使用 `/rtk/odom` 的 position xyz。
- 使用 `/rtk/odom` 的 position covariance 做质量门控。
- 使用 `/fix.status.status` 做状态码门控。
- 不直接使用 RTK heading/yaw。
- yaw 只通过 SLAM 轨迹和 RTK 轨迹的平面位移估计一个坐标系对齐角。
- fused odom 的姿态主要来自 `/slam/odometry`。

RTK 淡入条件：

- 已收到 `/slam/odometry` 和 `/rtk/odom`。
- 已收到 `/fix`。
- `/fix.status.status >= min_navsat_status`，默认要求 `STATUS_GBAS_FIX`。
- RTK 时间戳不超过 `max_rtk_age_s`，默认 `1.5s`。
- `/fix` 时间戳不超过 `max_fix_age_s`，默认 `1.5s`。
- SLAM 时间戳不超过 `max_slam_age_s`，默认 `0.5s`。
- RTK position std 不超过 `max_rtk_position_std_m`，默认 `1.0m`。
- RTK 相邻可用点跳变不超过 `max_rtk_jump_m`，默认 `3.0m`。
- 第一次可用 RTK 会初始化 offset，之后用低通慢慢修正。

RTK 淡出条件：

- RTK 断流、fix 状态不够、超时、协方差过大或跳变过大时，本帧 RTK 被拒绝。
- 如果已经有 offset，`/slam/odometry_fused` 会继续跟随 SLAM 加上最后一次有效 offset。
- 如果还没有任何有效 RTK，`/slam/odometry_fused` 基本等同 `/slam/odometry`。

## 5. 验证

RTK 输入：

```bash
ros2 topic echo /rtk/status
ros2 topic echo /rtk/odom
ros2 topic echo /fix
cat /tmp/rtk_nmea
```

融合输出：

```bash
ros2 topic echo /rtk/fusion_status
ros2 topic echo /slam/odometry_fused
```

建图记录：

```bash
ls /tinynav/output/map_go2_looper/rtk_continuous_odom.npy
```

判断标准：

- `/rtk/status` 里 `accepted=true`。
- 推车移动时 `/rtk/odom` 以米为单位变化。
- `/rtk/fusion_status` 里 `offset_ready=true`。
- 行走一段距离后 `alignment_ready=true`。
- `/slam/odometry_fused` 连续、平滑，没有瞬间米级跳变。
