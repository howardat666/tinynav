# VIO 与轮速里程计的三组对比：跑通链路的操作手册

目标：在 LeKiwi + Looper（全部算力在相机内的 X5 上）跑通三组配置，确认导航链路
可用，并把 VIO 与轮速里程计的差异隔离出来。

| | 建图位姿 | 导航位姿 | 作用 |
| --- | --- | --- | --- |
| **a** | VIO | VIO | 基线。链路本身是否可用 |
| **b** | VIO | 轮速 | 目标配置。只换里程计，其余不变 |
| **c** | 轮速 | 轮速 | 全轮速。地图本身的质量差异 |

三组只差环境变量，代码路径完全相同。

## 板上环境的既有事实（2026-08-05 实测）

X5 是上位机，建图和导航都在板上跑，电脑只当浏览器。板上现状：

| | 状态 |
| --- | --- |
| tmux / screen | **都没有** —— `scripts/run_*.sh` 在板上根本跑不起来 |
| ROS 2 Humble | 有，但必须 `. /userdata/x5/env.sh`，否则 `rosbag2_py` 因缺 `libtinyxml2.so.9` 直接坏掉（`ros2 bag record` 报 MISSING） |
| Python 依赖 | numpy 1.26.1 / scipy 1.15.3 / numba 0.61.2 / cv2 4.11 / einops / codetiming / tqdm / pyserial / rclpy / rosbag2_py / pydbow3 / onnxruntime 1.18.0 / **av 17.1.0** —— 全齐 |
| decord | 无 aarch64 wheel，用 `pylibs/decord.py` 那个真实现顶（PyAV 后端） |
| 外网 | **没有** —— pip 装不了东西，wheel 得从 PC 拷 |
| `app/` | **板上没有**，fastapi/uvicorn/psutil 也没装 → web app 目前跑不起来 |
| 代码同步 | 手工拷贝，会静默变旧。用 `tool/x5_board/sync_to_board.sh`，它会回读 md5 对比 |
| `/userdata` | 53 G，剩 37 G |
| 内存 | total 1307 MB，**无 swap**。`insight_full` 占 ~212 MB |

所以 **web app 是目标，但不是今天的路径**：录制和建图用
`tool/x5_board/map_record.sh` 直接跑（它用 `setsid` + nohup + 日志代替 tmux 的分屏）。

> **词典是生死问题。** `MemTotal` 只有 1307 MB，默认 `ORBvoc` 要 1735 MB，必然 OOM。
> 一律用 `/userdata/x5/voc/voc_office_k10L5.dbow3`（峰值 407 MB）。

## 为什么只开一趟车

a 和 c 若各录一次 bag，两张地图的差异里就混进了「两趟车开得不一样」，归因不干净。

`build_map_node` 的位姿是从**话题**取的，所以只要一个 bag 里同时有
`/camera/camera/vio_image` 和 `/wheel/camera_pose`，换个参数就能建出第二张图。
录制时 `wheel_odometry_node` 本来就必须在跑（它同时是驱动轮子的执行器），所以
轮速位姿是免费搭进 bag 的。

**建图那趟必须驱动，不能手推。** 手推全向底盘的滚子打滑约 11%，标定时实测到过
物理上不可能的轮半径。方案 c 的地图要有意义，每一米都得是轮子滚出来的。

## 环境变量

| 变量 | 取值 | 含义 |
| --- | --- | --- |
| `TINYNAV_ROBOT_TYPE` | `go2` \| `b2` \| `lekiwi` | 机器人几何，见 `tinynav/core/robot_config.py` |
| `TINYNAV_ACTUATOR` | `unitree` \| `wheel` \| `none` | 谁消费 `/cmd_vel` |
| `TINYNAV_ODOM_SOURCE` | `vio` \| `wheel` | **导航**用哪个位姿 |
| `TINYNAV_MAP_ODOM_SOURCE` | `vio` \| `wheel` | **离线建图**回放哪个位姿 |
| `TINYNAV_WHEEL_PORT` | 默认 `/dev/ttyS3` | Feetech 总线 |
| `TINYNAV_WHEEL_RADIUS` | 默认 `0.050385` | 实测值 |
| `TINYNAV_BASE_RADIUS` | 默认 `0.127083` | 实测值 |
| `TINYNAV_WHEEL_CAMERA_OFFSET` | 默认 `0.06,0.05,0.18` | `base_link -> camera`，`[前,左,上]` 米 |

`scripts/run_lekiwi_app.sh {vio-vio|vio-odom|odom-odom}` 把这些一次配好。

## 操作步骤

### 0. 一次性检查

```bash
# 板上：舵机总线在不在
python3 tool/x5_board/servo_scan.py

# 相机的话题在不在（分辨率应为 544x640，vio 话题必须存在）
ros2 topic hz /camera/camera/vio_image
ros2 topic hz /camera/camera/depth/image_rect_raw
```

### 1. 起 app，确认接线

```bash
bash scripts/run_lekiwi_app.sh vio-vio
curl -s localhost:8000/device/platform
```

`/device/platform` 是只读的，回读实际生效的配置。**每次换方案都先看这个** ——
三组配置只差环境变量，而每一个配错都是静默失败：`robot_type` 配错只是跟踪变差，
`pose_topic` 配错只是永远重定位不上，都不会报错。

预期看到：

```json
{"robotType":"lekiwi","actuator":"wheel","odomSource":"vio","mapOdomSource":"vio",
 "keyframePoseTopic":"/camera/camera/vio_image",
 "controlPoseTopic":"/camera/camera/vio_100hz",
 "cmdVelNodeEnabled":true,"wheelOdometryRunning":true}
```

### 2. 录建图 bag（只录这一次）

实测码率（71 秒静止预演，`--sensor looper`）：**14.3 MB/s = 857 MB/min**，eMMC
完全跟得上，**零丢帧** —— infra1 20.02 Hz、depth 5.01 Hz、vio_image 20.01 Hz、
vio_100hz 99.0 Hz、color/compressed 30.0 Hz、imu 400 Hz。5 分钟约 4.3 GB。

在 app 里开始录制（或 `POST /bag/start`），然后**驱动**着开一圈。遥控二选一：

- app 的遥控（发 `/cmd_vel`，`wheel_odometry_node` 消费）
- ssh 里 `python3 tool/x5_board/wheel_teleop.py`

速度建议 **0.15 m/s 左右**。关键帧阈值是 3 cm，而 bridge 的严格时间戳同步把
关键帧率顶到 **4.83 Hz**，所以超过约 0.15 m/s 之后关键帧就由运动而不是阈值决定，
间距不再均匀。慢一点同时也少给里程计记账打滑。

开完停止录制。app 会校验 bag（非空、可读）后归档。

**录完确认两个位姿都在**：

```bash
ros2 bag info <bag> | grep -E "vio_image|wheel/camera_pose"
```

`/wheel/camera_pose` 缺了就说明录制时 `wheel_odometry_node` 没跑，方案 c 做不了。

### 3. 建 VIO 地图（方案 a 和 b 共用）

⚠️ **板上建图必须带三个参数，否则会被 OOM 杀掉。** 2026-08-05 实测：不带参数跑一个
71 秒的 bag，**59 秒后 rc=137 被 OOM 杀死，只建到第 1 个关键帧**，峰值 RSS 645 MiB
（板子 1307 MiB、无 swap），MemAvailable 掉到 103 MiB。

机理不是「模型太大」，是**生产快于消费**：

- `BagPlayer` 原本**完全不节流**，`play_next()` 有多快读多快发。同进程内
  `publish 一条 → spin_once` 看似自节流，但消费者 `build_map_node` 在
  **另一个进程**、隔着 DDS，节流传不过去。
- `build_map_node` 的 `ApproximateTimeSynchronizer` 队列是 **200**，每个 slot 装
  关键帧图 + 深度 + RGB 三张全分辨率图 ≈ 1.4 MB → **280 MB 的敞口**。那行代码的
  注释写着「Keep sync queue bounded to reduce OOM risk on Jetson」，值却是 200。

另外第一个关键帧里 **12.6 秒是纯 rviz 可视化**（`publish_local_pointcloud` 8538 ms
+ `pose_graph_trajectory_publish` 4055 ms），而真正干活的特征提取 + 存图 + embedding
合计只有 719 ms。板上无头运行，这些一分钱都不该花。

所以板上一律这样跑：

```bash
. /userdata/x5/env.sh
cd /userdata/x5/tinynav

# 1) bridge（决定用哪个位姿源建图）
python3 tool/looper_bridge_node.py --pose-topic /camera/camera/vio_image &

# 2) 建图
python3 tinynav/core/build_map_node.py \
    --bag_file /userdata/x5/bags/<NAME>/<NAME>_0.db3 \
    --map_save_path /userdata/x5/maps/<NAME>_vio \
    --play-rate 1.0 --sync-queue-size 20 --no-visualization \
    --loop-closure-mode bow --loop-closure-use-bow \
    --dbow3-vocabulary-path /userdata/x5/voc/voc_office_k10L5.dbow3
```

方案 c 只改两处：`--pose-topic /wheel/camera_pose` 和 `--map_save_path ..._odom`。

`TINYNAV_MAP_ODOM_SOURCE=vio` 时，在 app 里触发建图。建图跑在隔离的
`ROS_DOMAIN_ID=231`，不会和相机的实时话题打架。

日志里会打出用的是哪个位姿源：

```
map build pose source: vio (/camera/camera/vio_image); navigation will run on vio
```

### 4. 方案 a：VIO 建图 + VIO 导航

地图建好后在 app 里启动导航节点，下发 POI。这是基线 —— 先确认链路本身通。

看什么：重定位是否成功、跟踪是否稳定、`/device/platform` 是否是 `vio`/`vio`。

### 5. 方案 b：VIO 建图 + 轮速导航

**地图不用重建**，直接换配置重启 app：

```bash
bash scripts/run_lekiwi_app.sh vio-odom
curl -s localhost:8000/device/platform    # 三个 pose topic 都应变成 /wheel/camera_pose
```

注意 `keyframePoseTopic` 也必须变成 `/wheel/camera_pose` —— 只换 planning 和
control 是错的，见 `wheel_odometry.md` 的 Frames 一节。

看什么：和 a 用**同一张地图**，所以差异就是里程计源的差异。重点看重定位间隔内的
漂移，以及标定时发现的 **1.6% 横向不可观测滑移**在闭环里表现如何。

### 6. 方案 c：全轮速

```bash
bash scripts/run_lekiwi_app.sh odom-odom
```

用**第 2 步那个 bag** 重新建图（这次回放 `/wheel/camera_pose`），然后导航。

注意这会覆盖 app 的 `map_path`。方案 a 的地图要留着的话，建图前先把目录挪走
（`map_path` 是符号链接时 app 会正确处理，见 `_start_rosbag_build_map` 的注释）。

## 三个已经踩过的坑

**① bridge 的时间戳同步。** `looper_bridge_node` 原来用
`message_filters.TimeSynchronizer`，精确匹配 `(sec, nanosec)`。这对相机自己的
VIO 成立，因为固件是用**图像的**时间戳给 `vio_image` 打戳（所以才叫这个名字），
实测 12 秒内 58/58 帧全部匹配。但轮速位姿用的是板子自己的时钟减去总线读取延迟，
纳秒永远不会相同 —— 精确匹配的结果是**一帧都同步不出来，而且完全静默**。

现在 `--pose-sync auto`（默认）对 `/camera/camera/vio*` 用精确匹配，其他一律
近似匹配，并且把选了哪条分支打进日志。20 ms 的 slop 在 0.15 m/s 下是 1.5 mm 和
0.23°，远小于 3 cm 的关键帧阈值。

**② 机器人几何原来是 GO2 的，而且两处互相矛盾。** `planning_node` 用
`GO2_CONFIG`（`camera_x=0.2`），`cmd_vel_control` 里硬编码 `[0, 0, 0.35]`。LeKiwi
的真值是 **0.06 m**。按 0.35 算，控制器认为机器人中心在相机后 35 cm，比整台机器人
还长，跟踪误差是系统性的、和里程计源无关 —— 不修的话三组对比全是噪声。现在两边
共用 `tinynav/core/robot_config.py`，结构上无法再漂移。

同一处还有个真 bug：`cam_offset_3d` 把 body 的**左**偏置放进了相机光学的**右**轴，
符号是反的。GO2/B2 的 `camera_y` 都是 0 所以从没暴露；我们是 50 mm，会引入 100 mm
横向偏置。

**③ 串口只能一个进程占。** Feetech 总线半双工，`wheel_odometry_node`
（`enable_wheel_command:=true`）同时读编码器和写 `Goal_Velocity`，是 LeKiwi 上
`unitree_control` 的对应物。`lekiwi_control.py` 必须完全不跑：它开环，既不读闭环
位姿也不发 `/control/target_pose`（planning 需要它）。app 现在按
`TINYNAV_ACTUATOR` 决定起哪个执行器。

`wheel_odometry_node` 是**长生命周期**的，不随传感器节点或导航节点启停：重启它会
把里程计原点归零，等于把所有消费者瞬移一次；录制中途归零还会直接污染写进 bag 的
`/wheel/camera_pose`。

## 还没做的

- **IMU 融合。** 实测偏航：陀螺 −0.1%，轮速 −0.3%（2 圈原地自转）。轮速的 0.3% 在
  5 秒重定位间隔内是 0.31°，已经达标；而陀螺赢只是因为做了零偏校正，不校正是
  −0.54%，比轮速更差。真正的短板是直线行驶时 1.6% 的横向滑移，陀螺修不了。所以
  优先级排在链路跑通之后，见对话记录里的分析。
- **`base_link -> camera` 的姿态标定。** 现在只有卷尺量的平移，姿态假设正装水平。
  标定时实测 IMU 重力方向偏离竖直 4.6°。
- **板子时钟。** 板上是 2025-08-27，比实际慢约 11 个月。板内自洽（`is_time_sync=1`），
  所以板上节点都正常，只有 PC 端 rviz 会错。**绝对不要在节点运行时同步时钟** ——
  11 个月的跳变会让 `max_keyframe_age_s` 丢掉每一个关键帧。
