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

当前推荐的数据流：

```text
NTRIP caster -> RTCM -> /dev/ttyTHS1 -> RTK board
RTK board -> NMEA -> /dev/ttyCH341USB0 -> rtk_bridge_node.py
```

如果使用同一个串口收发，也可以通过 ROS 参数覆盖 `serial_port`、`rtcm_serial_port`、`baud`、`rtcm_baud`。

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

主要输出 topic：

| Topic | 类型 | 作用 | 发布频率/触发条件 |
| --- | --- | --- | --- |
| `/rtk/io_status` | `std_msgs/String` JSON | 串口/NTRIP/RTCM 调试状态，重点看 `last_nmea_age_s`、`last_rtcm_age_s`、`rtcm_written_bytes`、`rtcm_dropped_bytes`、`ntrip_gga_source` | 固定 1 Hz |
| `/rtk/status` | `std_msgs/String` JSON | 定位状态摘要，重点看 `accepted`、`gga_quality`、卫星数、HDOP、经纬度、ENU、最近 GGA | 固定 1 Hz |
| `/rtk/nmea_sentence` | `std_msgs/String` | 过滤后的 NMEA 句子镜像，默认只转发 GGA，便于观察定位输入 | 每收到匹配 `raw_sentence_types` 的 NMEA 就发布；默认约等于 GGA 频率 |
| `/fix` | `sensor_msgs/NavSatFix` | 从 GGA 解析出的经纬度、高程和 GNSS 状态 | 每收到一条带经纬度的 GGA 就发布；室内 GGA 为空时不发布 |
| `/time_reference` | `sensor_msgs/TimeReference` | GGA UTC 时间参考 | 跟 `/fix` 一样，由有效 GGA 触发 |
| `/vel` | `geometry_msgs/TwistStamped` | 从 RMC 解析的地速和航向速度分量 | 每收到一条有效 RMC 就发布 |
| `/heading` | `geometry_msgs/QuaternionStamped` | 从 HDT/THS 解析的 heading | 每收到一条 HDT 或 THS 就发布 |
| `/rtk/odom` | `nav_msgs/Odometry` | 经纬度转换到本地 ENU 后的 RTK 里程计 | 每收到一条达到 `min_navsat_status` 的 GGA 就发布 |
| `/rtk/path` | `nav_msgs/Path` | `/rtk/odom` 的轨迹累计 | 跟 `/rtk/odom` 一样 |

频率说明：

- `/rtk/status` 和 `/rtk/io_status` 是定时 1 Hz，室内无定位也会发布。
- `/rtk/nmea_sentence` 由 `raw_sentence_types` 过滤后发布，默认 `GGA`。要转发全部 NMEA，可传 `-p raw_sentence_types:=ALL`；要同时看 GGA/RMC/THS，可传 `-p raw_sentence_types:=GGA,RMC,THS`。
- `/tmp/rtk_nmea` 始终镜像全部收到的 NMEA，适合本机快速看原始串口输出。
- `/fix`、`/time_reference` 依赖 GGA 里有经纬度；室内常见 `$GNGGA,xxxx,,,,,0,...` 不会发布 `/fix`。
- `/rtk/odom`、`/rtk/path` 还需要 GGA 状态达到 `min_navsat_status`。默认 `min_navsat_status=STATUS_FIX`，即普通 fix/dgps/float/fixed 都可产生 odom；如果要只接受 RTK fixed/float，可提高该参数。
- NTRIP GGA 回传周期由 `ntrip_gga_period_s` 决定，默认 1 Hz。`ntrip_gga_source=live` 表示正在用实时 GGA 给 caster；`initial` 表示当前实时 GGA 没有经纬度，只能用初始 GGA。

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
ros2 topic echo /rtk/io_status --field data
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
ros2 topic echo /rtk/io_status --field data
ros2 topic echo /rtk/status
ros2 topic echo /rtk/nmea_sentence --field data
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

- `/rtk/io_status` 里 `last_nmea_age_s < 2`、`last_rtcm_age_s < 2`、`rtcm_dropped_bytes=0`。
- `/rtk/io_status` 里 `ntrip_gga_source=live`，说明 caster 收到的是实时位置 GGA。
- `/rtk/status` 里 `accepted=true`。
- 推车移动时 `/rtk/odom` 以米为单位变化。
- `/rtk/fusion_status` 里 `offset_ready=true`。
- 行走一段距离后 `alignment_ready=true`。
- `/slam/odometry_fused` 连续、平滑，没有瞬间米级跳变。

户外如果怀疑 NMEA 更新不及时，按这个顺序查：

1. 看 `/rtk/io_status`：`last_nmea_age_s` 和 `last_gga_age_s` 是否持续小于 2 秒。
2. 看计数：`nmea_sentence_count`、`nmea_gga_count` 是否持续增长。如果总数增长但 GGA 不增长，说明板子没有持续输出 GGA。
3. 看 `/rtk/nmea_sentence --field data`：默认只显示 GGA，适合确认 GGA 是否带经纬度、quality 是否从 0 变成 1/2/4/5。
4. 看 `cat /tmp/rtk_nmea`：如果需要确认 THS/RMC 等其它句子是否把串口刷得太满，这里能看到全部 NMEA。
5. 看 `/rtk/io_status`：`rtcm_written_bytes` 是否持续增长，`rtcm_dropped_bytes` 是否为 0，`ntrip_gga_source` 是否为 `live`。

需要复盘现场问题时，录 bag 至少保留 `/rtk/io_status`、`/rtk/status`、`/fix`、`/rtk/odom`。`/rtk/nmea_sentence` 默认只含 GGA，数据量不大；如果要完整原始 NMEA，单独保存 `cat /tmp/rtk_nmea` 输出或用 `raw_sentence_types:=ALL` 临时打开。
