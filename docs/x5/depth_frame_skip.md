# 把 Looper 的 depth 降到 5 Hz —— 改动、部署与实测

> 完成日期：**2026-08-03** · 目标机：64GB Looper（`soc_uid 0x328152560a164e920256c08a00120040`）
> 相机 MIPI 故障的修复见 [`fix_64gb_mipi.md`](fix_64gb_mipi.md)
> ✅ **已部署并验证：depth = 5.003 Hz，VIO 保持开启，零崩溃**

---

## 1. 为什么降 depth 频率

板上重定位周期预算是 **5 秒**，根本不需要 12.8 Hz 的深度图。而 depth 是 X5 上最贵的一项：

| 资源 | depth 的开销（12.8 Hz vs 完全关闭，实测） |
|---|---|
| CPU | **0.86 核** |
| BPU | **89% → 0%** |
| 内存 | RSS 213 MB → 176 MB（**−37 MB**） |
| 温度 | 85–87 °C → 75–77 °C（**−10 °C**） |

🔴 **温度是最要紧的一条**：CPU/DDR 的 passive 触发点是 **95 °C**，而 12.8 Hz 时静止场景就到 **85–87 °C，余量只剩 8 °C**。后面还要压上 ORB / SuperPoint 的负载，必须先腾出热余量。

---

## 2. LooperHub 的代码改动

仓库 `/home/dm/looper/LooperHub`（`deepmirrorinc/LooperHub`），分支 **`x5/depth-frame-skip`**，6 文件 `+24/−4`。

降频机制**本来就在代码里**，只是没暴露成参数：

```cpp
// src/perception/depth_engine.cpp:209
// frame rate control: process only one frame every frame_skip_ frames, reduce DDR bandwidth pressure
if (++compute_frame_count_ % frame_skip_ != 0) { continue; }
```

`frame_skip_` 是私有成员、硬编码 `{1}`、无人赋值。改动就是照 `hand_frame_skip` 的现成写法把它接出来：

| 文件 | 改动 |
|---|---|
| `insight_full_node.hpp` | 新增成员 `uint32_t depth_frame_skip_ = 1;` |
| `insight_full_node.cpp` | `declare_parameter<int>("depth_frame_skip", userParamNumber("depth_frame_skip").value_or(1))`，下界钳到 1 |
| `insight_full_node_perception.cpp` | 构造 `DepthEngine` 时传进去；启动日志打出 `depth_frame_skip=%u` |
| `depth_engine.hpp` / `.cpp` | 构造函数末位新增 `uint32_t frame_skip = 1`（带默认值 → 源码兼容），初值列表里 `frame_skip_(std::max<uint32_t>(1, frame_skip))` |
| `readme.md` | 参数表补一行 |

> 🟢 **代码默认值保持 1 → 对所有其他设备零行为变化。** 只在这台的 `user_params.json` 里设成 4。
> 参数走 `userParamNumber`，所以既能写在 `user_params.json` 里，也能 `--ros-args -p depth_frame_skip:=4` 覆盖。

**20 fps ÷ 4 = 5 Hz。**

---

## 3. 怎么编（含四个坑）

镜像：`aliyunregistry.deepmirror.com.cn/dm/looper-hub-dev:latest`（**32 GB，压缩 10.2 GB**）。

> ⚠️ **匿名就能拉，不需要 `docker login`。** `curl https://aliyunregistry.deepmirror.com.cn/v2/` 返回 `HTTP/2 401` 只是标准的 token 挑战，不代表没权限 —— 直接 `docker pull` 即可，慢是因为 10 GB。

```bash
docker pull aliyunregistry.deepmirror.com.cn/dm/looper-hub-dev:latest
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/dm/looper/LooperHub:/LooperHub -w /LooperHub/tros_ws \
  --entrypoint bash aliyunregistry.deepmirror.com.cn/dm/looper-hub-dev:latest \
  -c 'bash ./build.sh --enable-vio'
```

编译只要 **40 秒**（7 个包）。踩过的四个坑：

| # | 现象 | 原因与解法 |
|---|---|---|
| 1 | `./robot_dev_config/build.sh: No such file or directory` | clone 时用了 `--depth 30`，**submodule 没初始化**。`git submodule update --init --recursive` |
| 2 | submodule 目录里**只有 `.git`，工作树是空的**（`git submodule status` 前缀 `+`，文件全显示 `D`） | 按索引记录的 SHA `git -C <sub> checkout -f <sha>` 强制恢复 |
| 3 | 🔴 两个私有 submodule 拉不到：`deepmirrorinc/RDK_VIO_Deploy`、`deepmirrorinc/kalibr_x5`（`Repository not found`）。**VIO 需要前者** | 看 `CMakeLists.txt:122-131`，`RDK_VIO_Deploy` 只是个装**预编译 `.so` + 头文件**的壳子（`add_library(vio SHARED IMPORTED)`）→ **自己搭替身**（见下）。`kalibr` 走 `KALIBR=prebuilt`，用仓库自带的 `tros_ws/prebuilt/kalibr-x5.tar.gz`，空目录被 colcon 忽略即可 |
| 4 | `vio_manager.cpp:296: error: 'VioFramePoseCovariance' was not declared` | 我最初从**镜像**的 `insight_tros_pak`（2026-04-22）拼替身，**API 太旧**。改从**板上**取（板上是 2.1.2 / 07-17 构建，版本更新且天然与部署目标一致） |

### `RDK_VIO_Deploy` 替身的正确做法

```bash
D=/home/dm/looper/LooperHub/tros_ws/src/RDK_VIO_Deploy
mkdir -p "$D/vio_deps"
scp root@169.254.10.1:/userdata/install/include/example/vio_xr_capi.h    "$D/"
scp root@169.254.10.1:/userdata/install/include/example/vio_scene_data.h "$D/"
scp -r 'root@169.254.10.1:/userdata/install/lib/example/vio_deps/*'      "$D/vio_deps/"
```

验证：`grep VioGetLatestFramePoseCovariance $D/vio_xr_capi.h` 要有命中；`libvio_xr_capi.so` md5 应为 `b47f243aef1f350b15a21008eb65be26`（51 个文件 / 37 MB）。

> ⚠️ 这是权宜之计，版本跟着板上固件走。**长期应该拿到 `deepmirrorinc/RDK_VIO_Deploy` 和 `kalibr_x5` 的访问权限。**

### ⭐ 顺带发现：sysroot 里全是现成的 aarch64 预编译库

这对后面 tinynav 上板的交叉编译价值很大 —— 而且**这些库和板上固件是同一套构建，ABI 一致性有保证**：

```
/opt/sysroot_docker/usr_x5/lib/libceres.so.2.0.0 + lib/cmake/Ceres/   ← tinynav_cpp_bind 要的 Ceres
/opt/sysroot_docker/usr_x5/include/eigen3                             ← Eigen
/opt/sysroot_docker/usr_x5/include/pybind11                           ← pybind11
.../example/vio_deps/libDBoW3.so                                      ← pydbow3 要的 DBoW3
.../example/vio_deps/{libglog,libcholmod,libamd,libatlas,...}.so       ← Ceres 运行时依赖
/opt/insight_tros_pak/files/opencv-release/lib/                        ← aarch64 OpenCV 4.5.4
```

⚠️ 但 `libDBoW3.so` **没有配套头文件**，头要从 DBoW3 源码取，并用 `nm -D --defined-only` 核对符号是否同版本。

---

## 4. 怎么部署（不打 OTA）

真正的代码在 **`libinsight_full_plugin.so`（2.6 MB）**里，那个 60 KB 的 `insight_full` 只是启动器（见 `CMakeLists.txt:228` 的 `add_library(${_lib_name} SHARED ${SOURCES})`）。

> ⚠️ **不要用 `local/looper-install.sh`** —— 它会先 `rm -rf /userdata/install`，那会连带删掉设备专属的 `user_params.json`（含 1lane 修复）、深度模型 `.bin`、标定文件。

```bash
B=/home/dm/looper/LooperHub/tros_ws/install
# 0) 整树备份（383 MB，/userdata 有 43 G 余量）
ssh root@169.254.10.1 'cp -a /userdata/install /userdata/install.bak.pre_depthskip'
# 1) 只覆盖插件 + 可执行文件
scp "$B/lib/libinsight_full_plugin.so" "$B/lib/libinsight_full_uvc_plugin.so" root@…:/userdata/install/lib/
scp "$B/lib/insight_full/insight_full" "$B/lib/insight_full/insight_param"     root@…:/userdata/install/lib/insight_full/
scp "$B/lib/insight_full/insight_full_uvc" root@…:/userdata/install/lib/insight_full_uvc/insight_full_uvc
scp "$B/lib/insight_full/libbmi088.so"     root@…:/userdata/install/lib/insight_full/      # 见坑 5
ssh root@… 'chmod 755 /userdata/install/lib/libinsight_full*.so /userdata/install/lib/insight_full/*'
# 2) 设参数（保留 1lane）
#    user_params.json 加 "depth_frame_skip": 4
# 3) 重启
ssh root@… 'systemctl reset-failed S99all_run.service; /etc/init.d/ota_project/scripts/insight-ctl s99 restart'
```

**RPATH 检查通过**：新插件的 RUNPATH 是 `/LooperHub/tros_ws/src/insight_full/../RDK_VIO_Deploy/vio_deps`，**与设备现有插件完全一致**。这个路径在板上不存在，靠 `all_run.sh` 里 `LD_LIBRARY_PATH` 的 `/userdata/install/lib/example/vio_deps` 解析 —— 和现在跑着的那份一样。

### 部署时又踩两个坑：**当前 HEAD 和板上固件 2.1.2 的文件布局不一致**

| # | 崩溃 | 原因 | 解法 |
|---|---|---|---|
| 5 | `Cannot find IMU config file: …/share/insight_full/config/bmi088.yaml` → `runtime_error: imu config file missing` → SIGABRT | 板上这个文件在 **`lib/imu_sensor/config/`**，新版找 **`share/insight_full/config/`**。两份 md5 相同（`d39157b7…`），纯路径变更 | `cp -a /userdata/install/lib/imu_sensor/config/bmi088.yaml /userdata/install/share/insight_full/config/` |
| 6 | `dlopen IMU sensor path: /userdata/install/lib/insight_full/libbmi088.so: No such file` → SIGABRT | 同理，板上在 `lib/imu_sensor/libbmi088.so`（361632 B），新版找 `lib/insight_full/` | 把**新编的**那份（121968 B，md5 `b0ea25f6…`）scp 到 `lib/insight_full/`。⚠️ 用新编的而不是板上旧的 —— dlopen 的接口可能随 HEAD 变过 |

> 💡 **这两个坑说明：板上固件 2.1.2 比 LooperHub HEAD 旧，配置/驱动的搜索路径被改过。** 以后混用「HEAD 编的插件 + 2.1.2 的 install 树」都要留意这类路径漂移。真要干净，应该整树部署一次并手工把设备专属文件补回去。

---

## 5. ⬤ 实测：资源占用对比

三个配置，**同一把尺子**（60 秒窗口，`insight_full` 的 CPU 直接读 `/proc/PID/stat` 的 `utime+stime`，BPU 按秒采样取平均，静止场景，VIO 全程开启，RGB 开启）：

| | `frame_skip=1`<br>**12.8 Hz** | `frame_skip=4`<br>**5.003 Hz** ✅ | `perception_engine=none`<br>**depth 关闭** |
|---|---|---|---|
| **`insight_full` CPU** | **1.945 核** | **1.412 核**（**−0.53**） | **1.088 核**（−0.86） |
| 全机 busy | 2.171 核 | **1.641 核**（**−0.53**） | 1.977 核 ⚠️ |
| **BPU 平均** | **89%** | **38%**（**−51 个点**） | **0%** |
| **峰值 CPU 温度** | **84.5 °C** | **79.1 °C**（**−5.4 °C**） | 77.5 °C |
| `insight_full` RSS | 189 MB | 187 MB（−2 MB） | **172 MB**（−17 MB） |
| `MemAvailable` | 896 MB | 899 MB | 945 MB |
| 线程数 | 62 | 62 | 38 |

### 三条读数说明

1. 🟢 **`frame_skip=1 → 4` 时，全机 −0.530 核 和 `insight_full` −0.533 核 完全吻合** —— 说明这两组是干净的。
2. ⚠️ **「depth 关闭」那列的「全机 busy 1.977」不可信**（比 5 Hz 还高，与 `insight_full` 的 1.088 矛盾）。那次用的是带逐进程归因的脚本，它每次快照要 fork ~90 个 `awk`，**采样开销和被测效应同一量级**。
   → **唯一免疫仪器开销的指标是 `insight_full` 自己的 `utime+stime`**（单次读累积计数器）。三个不同脚本测出的它是自洽的：12.8 Hz 1.90–1.95 · 5 Hz 1.41 · 关闭 1.07–1.09。**分析时以这一行为准。**
3. **降频几乎不省内存**（189 → 187 MB），因为 BPU 张量池是按 `kMaxBpuInFlight` 预分配的，跟频率无关。**只有完全关掉 depth 才省 17 MB**（`DepthEngine` 根本不构造，线程 62 → 38）。

### 其他话题不受影响（PC 端 `ros2 topic hz`）

| 话题 | 频率 |
|---|---|
| `depth/image_rect_raw` | 🟢 **5.003 Hz** |
| `infra1/image_rect_raw` | 19.833 Hz |
| `imu` | 394.4 Hz |
| `vio_100hz` | 99.2 Hz |
| `color/image_rect_raw/compressed` | 28.8 Hz |

---

## 6. 为什么先不关 VIO

`ENABLE_VIO` 是**编译期宏**（`CMakeLists.txt:92`），由 `-DDVIO` 控制，`build.sh` 自带 `--disable-vio` 开关，所以关它不用改代码。但**现在不该关**：

- PC 侧建图/评测链路全靠 `looper_bridge_node` 吃 Looper 自带 VIO（`/camera/camera/vio_image` → `/insight/vio_20hz`）
- 板上重定位也需要里程计，而**轮速里程计还没上实物验证**（见 [`wheel_odometry.md`](wheel_odometry.md)）
- 按 5 秒预算，关 VIO 省下的 ~2 核**用不上**，代价却是失去里程计冗余（方案 C 轮速+VIO 融合就没了）

> **顺序应该是：轮速里程计上实物验证通过 → 再决定要不要关 VIO。** 真要关时建议让他们出 VIO on/off 两个包便于 A/B。
> ⚠️ 唯一的运行时 VIO 暂停接口是 `calib_data_collector.cpp:323` 的 `on_vio_enable(false)`（标定录数据前用），**没有对外 service/topic，够不着**。

---

## 7. 回滚

```bash
ssh root@169.254.10.1 '
  rm -rf /userdata/install && cp -a /userdata/install.bak.pre_depthskip /userdata/install
  systemctl reset-failed S99all_run.service
  /etc/init.d/ota_project/scripts/insight-ctl s99 restart'
```

备份树 `/userdata/install.bak.pre_depthskip`（383 MB）里的原始 md5：
`libinsight_full_plugin.so` = `b599d7986562881dec634782855d9f7a` · `insight_full` = `acf988f4b7da6e067e48806b5912eaaa`

⚠️ 注意：回滚会把 `user_params.json` 也退回，**1lane 修复会一起丢** —— 回滚后要重新把 `stereo_sensor_name` 改成 `sc132gs-1088x1280-20fps-1lane`，否则相机起不来。

只想退掉降频、保留其他：把 `user_params.json` 的 `depth_frame_skip` 改成 1 并重启即可。

---

## 8. 遗留事项

| # | 事项 |
|---|---|
| 1 | 🔴 **`user_params.json` 会被 OTA 覆盖** → 1lane 修复和 `depth_frame_skip` 都会丢。解法见 [`fix_64gb_mipi.md § 10`](fix_64gb_mipi.md) |
| 2 | 要 `deepmirrorinc/RDK_VIO_Deploy` + `kalibr_x5` 的访问权限，替换掉现在的替身 |
| 3 | LooperHub 的 `x5/depth-frame-skip` 分支**尚未 push、未开 PR** |
| 4 | 温度：5 Hz 下静止 79 °C，距 95 °C 触发点 16 °C。**加上 ORB/SuperPoint 负载后要复测** |
| 5 | 可以考虑关掉 RGB（`rgb_sensor_name: ""`）再省一点 —— 当前 `rgb_pub` + `rgb_cap` + `jpg_encoder` 约 0.15 核，而导航用不到 RGB。⚠️ 但 imx415 低照度可能比 SC132GS 好，夜间检索还想评估它，暂时留着 |
