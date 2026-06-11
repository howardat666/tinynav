# TinyNav RTK MVP TODO

当前目标：把 RTK 融合进 TinyNav 的定位和导航链路，先做能在 Jetson Docker 上跑通的 MVP。调试和安全状态保留，地图地理坐标、经纬度 POI、完整日志分析等先放到后续。

## MVP 目标

- `rtk_bridge_node.py` 负责 RTK 输入：NTRIP、RTCM 写串口、NMEA 解析、发布 `/rtk/odom`。
- 新增 `rtk_fusion_node.py`：融合 `/slam/odometry` 和 `/rtk/odom`，输出 `/slam/odometry_fused`。
- TinyNav 导航节点默认行为不变；RTK 模式下通过参数或脚本让控制/导航消费 `/slam/odometry_fused`。
- RTK 状态不好时自动退回 VIO，不让定位跳变直接影响底盘。

## 1. RTK Bridge 必须稳定

准备写/改：

- `rtk/rtk_bridge_node.py`
  - 不硬编码 NTRIP 账号和密码，改用 ROS 参数或 `TINYNAV_NTRIP_USER` / `TINYNAV_NTRIP_PASSWORD` 环境变量。
  - caster、端口、mountpoint、initial GGA 可以保留现场默认值，但必须支持 ROS 参数或环境变量覆盖。
  - 保留 `/fix`、`/heading`、`/vel`、`/time_reference`、`/rtk/odom`、`/rtk/path`、`/rtk/status`。
  - `/rtk/status` 至少能看出：fix 是否 accepted、heading/velocity 是否 ready、NTRIP/serial 是否启用、ENU 坐标。
  - 可选补充：最近 NMEA/RTCM 时间、GGA quality、卫星数、HDOP。

验收标准：

- Jetson Docker 内单独运行 RTK bridge 可以看到 `/rtk/odom`。
- `/rtk/status` 能判断当前 RTK 是否可用于融合。
- 不需要再单独运行 `str2str` 或 `nmea_navsat_driver`。

## 2. RTK/VIO 融合定位

新增：

- `rtk/rtk_fusion_node.py`
  - 输入：`/slam/odometry`、`/rtk/odom`。
  - 输出：`/slam/odometry_fused`、`/rtk/fusion_status`。

第一版融合策略：

- VIO/IMU 负责高频连续运动。
- RTK 负责低频全局位置修正。
- 维护一个慢变化 offset，把 VIO 世界坐标逐步拉向 RTK 世界坐标。
- RTK 状态不好时不融合。
- RTK 跳变过大、速度不合理、时间戳太旧时拒绝本帧。
- heading 未实测确认前，只融合位置，不融合 yaw。

验收标准：

- `/slam/odometry_fused` 连续、平滑，不出现瞬间米级跳变。
- RTK 可用时长期漂移小于纯 VIO。
- RTK 不可用时系统继续输出接近 `/slam/odometry` 的结果。

## 3. 接入 TinyNav 导航

准备写/改：

- `tinynav/platforms/cmd_vel_control.py`
  - 把 odom topic 参数化。
  - 默认仍是 `/slam/odometry`。
  - RTK 模式传入 `/slam/odometry_fused`。
- 可选：`tinynav/core/map_node.py`
  - continuous odom topic 参数化。
  - 默认仍是 `/slam/odometry`。
- 暂时不改 `planning_node.py`，它和深度图同步更敏感。

验收标准：

- 原始 TinyNav 不带 RTK 仍能按默认 topic 跑。
- RTK 模式下控制节点使用 `/slam/odometry_fused`。
- `/nav/paused` 和急停逻辑仍然优先。

## 4. MVP 启动脚本

新增：

- `scripts/run_rtk.sh`
  - 只启动 `rtk_bridge_node.py`，方便单独调试 RTK。
- `scripts/run_navigation_rtk.sh`
  - 启动 RTK bridge。
  - 启动 `rtk_fusion_node.py`。
  - 启动 TinyNav perception/planning/map/control。
  - 控制节点订阅 `/slam/odometry_fused`。

验收标准：

- Jetson Docker 内一条脚本能启动 RTK 导航 MVP。
- RTK 断流或降级时，fusion 节点不会输出危险跳变。

## 暂缓内容

这些后面有空再做：

- RTK logger 和完整离线分析工具。
- RTK 与 TinyNav 坐标系的独立 alignment 节点。
- 建图阶段保存 `rtk_origin_lla.npy`、`rtk_poses.npy`、`map_rtk_alignment.npy`。
- 经纬度 POI 和 app 前端显示。
- 完整 RViz/PlotJuggler 调试面板。

## 实车注意事项

- 不要让多个进程同时打开 `/dev/ttyCH341USB0`。
- RTK 不要直接替代 VIO；遮挡、多路径、断网时 RTK 可能跳变。
- TinyNav 内部继续使用米制局部坐标，规划和控制不要直接处理经纬度。
- 融合必须做门控：fix 状态、协方差、跳变距离、速度合理性、时间戳 age。
- heading 必须实测确认方向后再进入融合。
