# 把 tinynav 的重定位跑在 Looper 相机内的 X5 上

**状态：跑通了。** ORB + DBoW3 的纯 CPU 重定位在 X5 上端到端运行，**p50 374 ms / p90 420 ms**，峰值 RSS **407 MB**，5 秒预算有约 **13 倍余量**。

这篇记录怎么从零把板子准备到这个状态，以及过程中撞到的每一个坑 —— 其中有四个是文档里没有、只能实测撞出来的。

- 硬件：Looper 相机内的 D-Robotics X5（8× Cortex-A55 @1.5 GHz，1× Bayes-e BPU，MemTotal **1307 MB**，无 swap）
- 相机固件：2.1.2，`insight_full`，已应用 [`depth_frame_skip.md`](depth_frame_skip.md) 的 5 Hz 降频和 [`fix_64gb_mipi.md`](fix_64gb_mipi.md) 的 1lane 修复
- 板上目录约定：一切都在 **`/userdata/x5/`** 下，不动固件的 `/userdata/install`

---

## 0. 快速上手

```bash
# PC 侧：每次板子重启后都要跑（没有 RTC，见 §2）
tool/x5_board/sync_board_time.sh

# 板上：每个 shell 都要 source
. /userdata/x5/env.sh

# 跑一次重定位评测
python3 /userdata/x5/reloc_offline_eval.py \
    --map /userdata/x5/maps/map_gt \
    --query-map /userdata/x5/maps/map_day \
    --vocab /userdata/x5/voc/voc_office_k10L5.dbow3 \
    --transform-json /userdata/x5/results/transform_gt_day.json \
    --db-path /userdata/x5/scratch/reloc_db \
    --out-prefix /userdata/x5/results/x5_gt_day
```

板上布局：

| 路径 | 内容 |
|---|---|
| `/userdata/x5/env.sh` | 环境变量（§1） |
| `/userdata/x5/tinynav/` | tinynav 代码；`tinynav_cpp_bind.so` 必须放在包**内部**（§4） |
| `/userdata/x5/pydbow3/` | 交叉编译的 `pydbow3` + 4 个依赖库，3.2 MB |
| `/userdata/x5/cpp_bind/` | 交叉编译的 `tinynav_cpp_bind` + Ceres/SuiteSparse 等 12 个库，5.8 MB |
| `/userdata/x5/pylibs/` | 不走 pip 的纯 Python 包：`sensor_msgs_py`、`decord`（§5） |
| `/userdata/x5/wheels/` | 离线 wheel + `install_on_board.sh`，119 MB |
| `/userdata/x5/maps/` | `map_gt`、`map_day`，各 1.8 GB |
| `/userdata/x5/voc/` | `ORBvoc.dbow3`、`voc_office_k10L4/L5.dbow3` |
| `/userdata/x5/cache/numba/` | numba 磁盘缓存，决定启动是 66 s 还是 34 s（§6） |

---

## 1. 板上 ROS 2 的 Python 环境是坏的，但只是环境变量的问题

`/opt/ros/humble` 和 `/opt/tros/humble` 都是 `/userdata/hobot/opt/ros/humble` 的符号链接。固件自带的
`/etc/init.d/looper/setting/ros2_env.conf` 足够跑 C++ 的 `insight_full`，但跑任何 Python 节点都不行，**有四处缺失**：

| # | 问题 | 后果 |
|---|---|---|
| 1 | `PYTHONPATH` 只有 `humble/lib/python3.10/site-packages` | 那里只有 82 个 `ament_*` 构建工具。**`rclpy`、`cv_bridge`、`message_filters`、`tf2_py` 和所有消息模块都在 `humble/local/lib/python3.10/dist-packages`**（86 个包），完全没被引用 |
| 2 | `AMENT_PREFIX_PATH` 根本没设 | `ros2` CLI 报 `PackageNotFoundError: ros2cli` |
| 3 | `ROS_DISTRO` 是 `/opt/ros/humble/humble` | 是个路径，而且是错的；应该是发行版名 `humble` |
| 4 | Python 的 `bin/` 不在 `PATH` | `pip` 找不到，只能用 `python3 -m pip` |

> ⚠️ **我自己在这里栽过一次，值得写下来当反面教材。** 第一次排查时我用 `find ... -path "*site-packages*"` 找包，这个条件恰好把
> `local/lib/python3.10/dist-packages/` 排除掉了，于是我得出「板上 ROS 2 是纯 C++ 安装、Python 节点根本跑不了」的结论 ——
> 那会是个周级别的工程。实际上只是环境变量少了一行。**用 `find` 判断"某个东西不存在"时，先确认你的过滤条件没把它排除掉。**

修法见 [`tool/x5_board/env.sh`](../../tool/x5_board/env.sh)。**不改 `ros2_env.conf`** ——
它属于固件，OTA 会覆盖。修完 `ros2 node list` / `ros2 topic list` / `ros2 topic hz` 全部可用（这是板子历史上第一次）。

---

## 2. 时钟：两个独立的问题，必须一起修

### 2.1 X5 没有 RTC 电池

`hwclock -r` 读出 1970-01-01，也没有任何 NTP daemon 在跑（`ntpdate` 二进制存在但没人调用）。
实测板子开机后是 **2025-08-26**，而 PC 是 **2026-08-04** —— 差了近 11 个月。

`tool/x5_board/sync_board_time.sh` 用 NTP 的办法修（把远端读数夹在两次本地读数中间，补偿半个往返），
一轮就收敛到 **-2 ms**。**每次板子重启后都要重跑**，因为没电池。

### 2.2 `insight_full` 默认用 CLOCK_MONOTONIC 打时间戳

这个比时钟偏差隐蔽得多，也严重得多。实测：

```
uptime          = 850.76
rclpy clock now = 1785809011.750       ← REALTIME
depth  header.stamp = 838.571          ← 约等于 uptime
infra1 header.stamp = 838.721
imu    header.stamp = 838.757
```

相机给消息打的是**单调时钟**（≈开机秒数），而 tinynav 节点用
`self.get_clock().now()`（REALTIME）给自己的输出打戳 —— 两者差 **17.8 亿秒**。后果：

- 用图像时间戳查 TF 必然抛异常（在 t=1.78e9 填充的 buffer 里查 t=838）
- `message_filters` 的时间同步永远匹配不上，而且是**静默丢弃**，不报错

`map_node.py` 和 `planning_node.py` 里有 12 处 `get_clock().now()`，所以这不是理论问题。

**好消息是不用改代码。** 固件本来就支持墙上时钟，只是判定条件没满足
（`insight_full_node.cpp:321`）：

```cpp
std::ifstream f(kTimeSyncFlagPath);          // /etc/init.d/looper/setting/is_time_sync
if (f >> flag && flag == 1) {
  int64_t offset = computeRealtimeOffset();  // REALTIME - MONOTONIC
  threads_.time_offset_ns.store(offset, ...);
} else {
  RCLCPP_WARN(..., "NTP sync not ready (%s). Timestamps will use CLOCK_MONOTONIC.");
}
```

这个 flag 文件在我们的板子上**根本不存在**。而且 `insight_full_node_vio.cpp:500` 有个
`timeSyncMonitorThread()` 每秒重读一次，所以 **写入 `1` 立即生效，不用重启**。

`sync_board_time.sh` 在同步完时钟后会自动设这个 flag —— 两件事必须一起做：光同步时钟不设 flag，
戳还是单调的；光设 flag 不同步时钟，戳会是错误的墙上时间。设完之后：

```
depth  header.stamp = 1785810317.561   wall - stamp = +0.2 s   ← depth 是 5 Hz，首帧年龄正常
infra1 header.stamp = 1785810317.711   wall - stamp = +0.0 s
imu    header.stamp = 1785810317.733   wall - stamp = +0.0 s
vio_100hz                              wall - stamp = +0.054 s
```

原始 flag 状态备份在 `/userdata/fixbak/`（本机是「文件不存在」，所以留的是
`is_time_sync.absent_marker`）。

### 2.3 🔴 flag 必须**跳变** 0→1，光是「值为 1」没用

这个坑是 2026-08-04 板子重启后实际踩到的，很隐蔽：

`computeRealtimeOffset()` **每次跳变只算一次**，结果缓存在 `threads_.time_offset_ns` 里；
`timeSyncMonitorThread()` 只在看到 flag 从「未设」变成「已设」时才重算。而
**flag 文件在 `/etc/init.d/looper/setting/` 下，重启后仍然存在，但时钟不会存活**。

于是重启后的时序是：flag 已经是 1 → 启动时就用**错误的时钟**（2025-08-26）算好偏移 →
我事后把时钟改对，但监控线程看到 flag 一直是 1，`was_synced` 保持 true，**永远不重算**。
现象是 PC 侧看到相机时间戳差 **29600733 秒（343 天）**，尽管板子 `date` 已经完全正确。

所以 `sync_board_time.sh` 的做法是：对完时钟后**先写 0、等 3 秒让监控线程观察到、再写 1**。

⚠️ **自测时不要用「板上两个话题互相配对」来验证** —— 它们共享同一个时钟，对错都能配上，
这个测试对该 bug 完全不敏感。要拿一个**板上话题**和一个**本机自己打戳的话题**去配。

### 2.4 USB 链路本身有约 240 ms 延迟

时钟对齐后（板内 `date` 与 PC 差 2 ms），PC 侧订阅板上话题实测：

| 话题 | PC 收到时刻 − header.stamp |
|---|---|
| `imu` | +0.215 s |
| `infra1/image_rect_raw` | +0.240 s |
| `depth/image_rect_raw` | +0.383 s |

这**不是时钟误差，是真实的传输 + 序列化延迟**（544×640 mono8 @20 Hz ≈ 7 MB/s 过 USB gadget + DDS）。
后果：跨机 `message_filters` 在 50 ms 容差下配对 **0 次**。要么把容差放到 0.5 s 以上，
要么（更好）**把消费者放在板上跑**，不要让同步跨越 USB 链路。

---

## 3. 相机话题的 QoS 和消息类型不统一

订阅时用错了收不到消息，而且**只有 `RELIABILITY` 不兼容时才会打警告**，类型用错则完全没有提示。

| 话题 | 类型 | Reliability |
|---|---|---|
| `depth/image_rect_raw` | `sensor_msgs/Image`（mono16, mm） | RELIABLE |
| `infra1,2/image_rect_raw` | `sensor_msgs/Image`（mono8, 544×640） | RELIABLE |
| `color/image_rect_raw/compressed` | `sensor_msgs/CompressedImage` | RELIABLE |
| `imu` | `sensor_msgs/Imu` | **BEST_EFFORT** ⚠️ 用默认 QoS 订阅收不到 |
| `vio_100hz` | **`geometry_msgs/PoseStamped`** ⚠️ 不是 `Odometry` | RELIABLE |

`vio_100hz` 是 `PoseStamped` 而 `map_node.py:461` 订阅的是 `Odometry, '/slam/odometry'`，
所以中间必须有转换（`x5_work/vio_relay.py` 干的就是这件事）。

---

## 4. 依赖：全部有官方 aarch64 wheel，没有一个需要板上编译

板上**已有**：`rclpy`、`cv_bridge`、`message_filters`、`tf2_ros`、`tf2_py`、全部 msg、
`rosbag2_py`、`rosidl_runtime_py`、`yaml`、`onnxruntime 1.18.0`、`pyserial`。

装上去的（`/userdata/x5/wheels/`，119 MB，2m12s 装完）：

| 包 | 版本 | 备注 |
|---|---|---|
| numpy | 1.26.1 | 从板上原有的 1.21.5 升级 |
| scipy | 1.15.3 | |
| llvmlite / numba | 0.44.0 / 0.61.2 | ⚠️ 见下方 manylinux 坑 |
| opencv-python-**headless** | 4.11.0.86 | ⚠️ 见下方 GTK 坑 |
| codetiming / einops / fufpy / tqdm | | 纯 Python |
| async-lru | 2.3.0 | ⚠️ 连带要升 `typing_extensions` |
| typing_extensions | 4.16.0 | 板上原有 3.10.0.2 太老，没有 `Self` |
| av | 17.1.0 | decord 替代品要用（§5） |

### 四个坑

**① numba/llvmlite 的 wheel 是 `manylinux_2_28`，不是 `manylinux2014`。**
只给 `--platform manylinux2014_aarch64` 时 pip 报的是

```
ERROR: Could not find a version that satisfies the requirement numba==0.61.2 (from versions: ... 0.60.0)
```

看着像 0.61.2 这个版本不存在，实际只是 0.61 系列把 glibc 门槛从 2.17 提到了 2.28。
板上 glibc 2.35，完全没问题。下载时要同时给
`--platform manylinux_2_28_aarch64 --platform manylinux_2_27_aarch64`。

**② 必须用 `opencv-python-headless`，因为板上完全没有 GTK。**
`ls /usr/lib/aarch64-linux-gnu/libgtk*` 什么都没有。板上系统自带的那个 cv2 是坏的 ——
注意它的报错会随 `LD_LIBRARY_PATH` 变化：修好库路径之前报 `libopencv_hdf.so.4.5d`，
修好之后才暴露真正的拦路虎 `libgtk-3.so.0`。**别被第一层报错带偏。**
`headless` wheel 的 `readelf -d` 里对 libgtk / libX11 / libGL 的引用数为 0，
而且仓库里没有任何 `cv2.imshow` / `namedWindow` / `waitKey`，所以无功能损失。
装的时候把坏的 `cv2.cpython-310-aarch64-linux-gnu.so` 显式改名备份，
**不要依赖「同目录下包优先于扩展模块」这种隐式 import 顺序**。

**③ `tinynav_cpp_bind.so` 必须放在 `tinynav` 包内部。**
`map_node.py:21` 写的是 `from tinynav.tinynav_cpp_bind import pose_graph_solve`，
所以光把它放到 `sys.path` 上不够，会报 `ModuleNotFoundError: No module named 'tinynav.tinynav_cpp_bind'`。
要 `cp` 进 `/userdata/x5/tinynav/tinynav/`，并把 `/userdata/x5/cpp_bind`（库是平铺的，
没有 `lib/` 子目录）加进 `LD_LIBRARY_PATH`。

**④ 升 numpy 会不会搞坏那些按 numpy 1.21 头文件编的二进制模块？实测不会。**
这是整个第 1 步里我最担心的一项，因为失败特征（`ndarray size changed`、
`_ARRAY_API not found`）只在**真正跑起来**时才出现，光 import 看不出来。
所以 `abi_check.py` 里每一项都做真实计算，8/8 通过：

| 检查 | 结果 |
|---|---|
| `cv_bridge` mono16 + mono8 往返 | 逐字节一致 |
| `rclpy` 建节点 + 构造 `PointCloud2` | OK |
| `onnxruntime 1.18.0` numpy 透传 | OK |
| `tf2_ros` + `tf2_py` | OK |
| `pydbow3` uint8 建库 / float64 被拒 | OK（dtype dispatch 生效） |
| `tinynav_cpp_bind` 三个符号 | OK |

> 🔴 **绝对不要用「升到 numpy 2」来解决问题** —— numpy 1.x 内部保证「旧头文件编、新运行时跑」
> 这个方向兼容，跨大版本不保证，会真的搞坏 `cv_bridge` 和 `onnxruntime`。

---

## 5. `decord` 在 aarch64 上不存在，用 PyAV 顶掉

`decord` 和 `eva-decord` **都没有 aarch64 wheel，也没有 sdist**，从源码编要把整套
ffmpeg + CMake 搬上板。而 `tool/video_db.py` 在模块顶层 import 它，
`TinyNavDB.__init__`（`build_map_node.py:453`）又**无条件**构造 `VideoDB` ——
所以**只要打开任何一张地图就会炸**（`TypeError: 'NoneType' object is not callable`，
因为 `build_map_node.py:35-37` 的 try/except 已经把 `VideoDB` 置成 `None` 了）。

而 tinynav 用到的 `decord` API 只有两个操作：`len(reader)` 和
`reader[i].asnumpy()` 返回 RGB 帧。PyAV 有 aarch64 wheel，能干这两件事。
所以 [`tool/x5_board/decord_shim/decord.py`](../../tool/x5_board/decord_shim/decord.py)
是个**真能用的替代实现，不是抛异常的假桩**：关键帧 seek + 前向解码 + 单帧缓存。

验证方式是在 PC 上跟**真 decord 0.6.0 逐帧比对**（顺序读、前跳、回跳、重复读、首帧、末帧
共 12 个位置），`max|diff| = 0` 全部逐字节一致。板上实测 1123 帧地图视频可正常随机访问。

> 写这个 shim 时踩了一个自己造的坑，记一下：`_seek_to()` 为了知道 seek 落到哪一帧，
> 必须先解一帧读它的 pts，这帧被消费掉并放进缓存了；但 `_decode_at()` seek 完之后
> 没检查缓存就继续往后找，于是找 index 0 时从 index 1 开始扫，扫完 1123 帧报
> `ran out of frames`。**seek 消费掉的那一帧可能就是你要的那一帧。**

只在板上把它放到 PYTHONPATH 里。装了真 decord 的机器不要用 —— 真 decord 快得多。

---

## 6. 实测结果

地图 `map_gt`（1123 关键帧），词典 `voc_office_k10L5`，每 37 帧取 1 个查询，28 个计时查询
（另有 2 个 warmup 被丢弃）。判定：`relocalize` 返回 True **且** XY 误差 ≤ 0.5 m **且**
旋转测地误差 ≤ 10°。

| | gt→gt 自一致 | **gt→day 跨时段** | PC 上的 gt→day（参考） |
|---|---|---|---|
| 成功率 | 27/28 = **96.4 %** | 27/28 = **96.4 %** | 97.3 %（1100+ 查询） |
| wall p50 | 457 ms | **374 ms** | 41 ms |
| wall p90 | 516 ms | **420 ms** | — |
| wall max | 1001 ms | 449 ms | — |
| 峰值 RSS | 429 MB | **407 MB** | 1735 MB（大词典）/ 663 MB（小词典） |
| 启动 `node_build_s` | 66.1 s（numba 冷） | **34.2 s（numba 热）** | 2.7 s |

**X5 比 PC 慢约 9 倍**，和 PC 上按核数缩放外推的 0.2–1.1 s 吻合。
**5 秒预算有约 13 倍余量 —— 时间明确不是瓶颈。**

分段耗时（gt→day，ms，mean/p50）：

| 阶段 | mean | p50 | 说明 |
|---|---:|---:|---|
| `match` | 132.3 | 137.6 | 最大头，3× FLANN-LSH |
| `depth3d` | 59.1 | 63.1 | 纯 Python for 循环，有优化空间 |
| `feature_extract` | 63.4 | 63.9 | ORB |
| `db_load` | 46.5 | 47.7 | eMMC 读 |
| `candidate_search` | 25.0 | 24.8 | DBoW3 检索 |
| `pnp` | 8.3 | 7.6 | |
| `publish` | 2.9 | 2.7 | |

### 内存和温度

`MemTotal` 只有 1307 MB，所以**词典选择是生死问题**：默认双份 `ORBvoc` 要 1735 MB，
在这块板上必然 OOM。换 `voc_office_k10L5` 后峰值 407 MB，运行期最低可用内存 517 MB，
峰值 CPU 温度 80 °C（降频阈值 95 °C 被动 / 110 °C 临界，有余量）。

### numba

10 个 `@njit` **全部在导航路径搜索里，一个都不在重定位路径上**。代价发生在
`MapNode.__init__ → _warmup_nav_path_search()`。`NUMBA_CACHE_DIR` 生效后
**启动从 66.1 s 降到 34.2 s（省 32 s）**，缓存 25 个文件 / 576 KB。

> 🔴 `.nbc` 是**目标 CPU 的机器码**，不能在 x86 上预编译再拷到 aarch64，
> 必须在板上真跑一次来生成。

---

## 7. 还没解决的

按对「白天办公室导航」这个当前目标的阻塞程度排序：

| | 问题 | 影响 |
|---|---|---|
| 🔴 | **PnP 之后没有任何几何校验**（只检查 `landmarks>40` 且 `inliers≥20`）。板上独立复现：gt→day 有 1/28 个查询返回 `success=True` 但 XY 误差 **1.9e14 m**、旋转 **112°**，还带着 weight>0.9 进 Ceres 污染 map→odom | 会污染位姿图，白天也有 1.8–2.2 % 发生率。**这是上车前必须修的** |
| 🔴 | **夜间成功率只有 14.2 %**，且根因在**特征层不在检索层**：夜间每帧只有 326 个 ORB 点（白天 908–926），甚至有整帧 0 个。换词典只能到 16.7 % | 换词典/调检索是死路。要走补光 / 换特征 / 夜间图只夜间用。**「晚上没灯」这个前提下，纯可见光无主动照明的方案不成立** |
| 🟡 | 启动 34 s：`LoopClosure.__init__`(bow) 用 `get_depth_embedding_features_images` 取描述子，白读 **1564 MB depth**（只要描述子的 132 MB），改用 `db.features[ts]` 一行搞定 | 冷启动时间 |
| 🟡 | `__init__` 建两个 `LoopClosure` 各载一份完整词典，而 `nav_loop_closure` 服务的 `keyframe_mapping` 在 `map_node.py:622` 已被禁用 | 大词典下白费 596 MB |
| 🟡 | ORB 二值描述子被存成 float32/255，`features.dat` 145 MB（本可 36 MB），每次匹配还要转回 uint8 | 磁盘 + `db_load` 耗时 |
| ⚪ | `depth3d` 是纯 Python for 循环，59 ms | 有优化空间但当前不是瓶颈 |
| ⚪ | LeKiwi 轮速里程计代码已写完自测通过，但**没在真硬件上标定过**；还有和 `lekiwi_control.py` 的串口独占冲突、`base_link → camera` 静态变换缺失 | 见 [`wheel_odometry.md`](wheel_odometry.md) |

不影响当前目标但记一下：`embeddings` 是 Dummy 零向量（所以 `--descriptor embedding` 报错是正常的，
不是 bug）；三张地图的描述子宽度都是 32，是真 ORB 不是 SuperPoint。

---

## 8. 相关文件

| 文件 | 作用 |
|---|---|
| [`tool/x5_board/env.sh`](../../tool/x5_board/env.sh) | 板上环境变量，修 §1 的四处缺失 |
| [`tool/x5_board/sync_board_time.sh`](../../tool/x5_board/sync_board_time.sh) | 时钟同步 + 设 `is_time_sync` flag（§2） |
| [`tool/x5_board/decord_shim/decord.py`](../../tool/x5_board/decord_shim/decord.py) | PyAV 实现的 decord 替代（§5） |
| `x5_work/reloc_offline_eval.py` | 离线重定位评测脚本（不在仓库里） |
| `x5_work/reloc_pc_baseline.md` | PC 侧基线，含更大样本量的成功率 |
| [`depth_frame_skip.md`](depth_frame_skip.md) | depth 降到 5 Hz |
| [`fix_64gb_mipi.md`](fix_64gb_mipi.md) | 64GB 相机 MIPI 修复 |
