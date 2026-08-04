# TODO —— Looper X5 轮式导航

> 状态基准日：**2026-07-31**
> 框架见 [`README.md`](README.md) · 实测数据见 [`x5.md`](x5.md)

---

## 1. 当前走的方案

**分支 `x5/wheel-nav`，基线 = [PR #136](https://github.com/UniflexAI/tinynav/pull/136)（junlinp）→ 即「方案 1：ORB 全经典」。**

| 层 | 用什么 | 代码位置 | 为什么选它 |
|---|---|---|---|
| 全局检索 | **DBoW3 + 预训练 ORBvoc**（145 MB, Git LFS） | `tinynav/core/models_trt.py:352` `DBoW3Engine` | 不需要神经网络，~5 ms |
| 局部特征 | **ORB**（nfeat=1024） | `models_trt.py:197` `ORBFeatureTRTCompatible` | X5 CPU 实测 46.5 ms，比 SuperPoint 快 22 倍 |
| 匹配 | **BF-Hamming + RANSAC** | `models_trt.py:242` `ORBMatcher` | 实测 26 ms，比 LightGlue 快 110 倍 |
| 模式开关 | `--loop-closure-mode {embedding,bow}` | `build_map_node.py:1149-1161`，`map_node.py` 同构 | **不是硬替换**，能随时切回学习式路线 |

**单次重定位 ⬤ 0.39 s / ⬤ 310 MB / 1–2 核** —— 在 5 秒预算下有 **10 倍余量**，是九个方案里唯一"零神经网络、零 BPU 依赖、内存最安全"的。

### 为什么先做这个（而不是精度更高的方案 5a/6）

1. **第一个版本的目的是验证管路**（轮速 → 重定位 → 规划 → 底盘），不是刷 recall。这时候算法选哪个几乎不影响。
2. **它是唯一现成的完整路线** —— #136 三层都有实现，还自带 TRT soft-import 骨架和设备部署脚本。
3. **零外部依赖** —— 不用等 LooperHub 放开 VIO/depth 开关，不用赌 BPU 工具链，不用等 64GB 硬件修好。

### 它的已知代价（必须记住）

- 🔴 **ORB 的光照鲁棒性差，而且完全没有量化数据**。这是方案 1 唯一的真风险，也是唯一能否决它的因素 → 见 [T-2](#t-2)。
- 🔴 **依赖 `pydbow3`，不在 PyPI 上**，aarch64 能否装成未验证 → 见 [T-1](#t-1)。装不上就换 [T-6](#t-6) 的方案 5a。

### 长期目标：方案 6

`DINOv2 int8 上 BPU + SuperPoint CPU + cv2 BF 匹配` ≈ ○ 1.5 s / 480 MB，是唯一"检索质量已知 + 用时可接受 + 内存安全"三者兼得的方案。前提是 BPU 能与固件并发（[T-3](#t-3)）+ int8 量化不掉精度（[T-4](#t-4)）。

---

## 2. 已完成的工作

### 2.1 硬件实测（4 台 Looper）

- [x] 四台设备台账：`soc_uid`、固件版本、eMMC/分区、内存、CPU 核数 —— `x5.md § 2.1`
- [x] 确认 **64GB 改装版分区表真的扩了**（`/userdata` 53 G，剩 44 G）
- [x] 确认 **64GB 版内存一分没多**（1307/943 MB，swap 0，与标准版一致）→ 所有方案的内存判断不变
- [x] 四个硬件维度的余量：CPU **4.5 核**（VIO 开）/ **6.6 核**（VIO 关）· 内存 **~911 MB** · BPU 87–95% · 存储
- [x] CPU 算力基准（自造 onnx）：**fp16 Conv 比 fp32 快 2.5 倍** → 推翻"fp16 是瓶颈"的早期假设
- [x] VIO 负载拆分：静止 **0.92 核** vs 运动 **2.11 核**（ZUPT 导致 2.3 倍差）+ 线程级明细
- [x] 🔴 **64GB 机器 MIPI 故障根因定位** —— **不是排线，是 lane 数不匹配**：sensor 寄存器被写成 1 lane（`0x3018=0x12`、`0x3019=0x0e` 禁用 D1/D2/D3）而 host 配成 2 lane。**决定性实测：1 lane 下两路立体相机都跑满 ~62 fps、错误计数器全 0**。嫌疑是旧 OTA 残留的 `libsc132gs.so.1.0.0`（所有模式都只写同一张 1-lane 表，`0x301f` 恒为 `0x45`）
- [x] 排除排线/接插件/SoC CSI/DPHY/供电/过温（两条物理独立链路逐位相同失败 + lane0 在 1200 Mbps 跑满帧 + 所有错误计数器为 0 + 无 regulator/thermal 日志 + 全频满速）
- [x] 摸清相机拓扑：rx0=imx415 RGB 4lane（未测）· rx2=立体#1（i2c-4 @0x32）· rx3=立体#2（i2c-0 @0x32 + EEPROM）
- [x] 找到证据说明**这台机器改装前出过图**（`/app/calibration/` 下的双目和 RGB 标定文件与镜像默认值不同，日期 Jul 17 2026）
- [x] 发现 64GB 机器 **RTC 是坏的**（`hwclock -r` = 1970），系统时间每次启动恢复到同一时刻 → crash log 同名互相覆盖、OTA 版本判断可能失效
- [x] ⭐ **发现物理 DRAM 是 3.9 GiB，~2.5 GiB 被 ion 预留** → `MemTotal 1307 MB` 是分配决策而非硬件上限 → 见 [T-20](#t-20)
- [x] 整理相机故障诊断入口（`/sys/class/vps/mipi_host*/status/*`、`/sys/kernel/debug/sif*/fps`、`multi_isp_vflow -s N`）

### 2.2 算法实测（X5 板上，ONNXRuntime 1.18 CPU）

- [x] SuperPoint @**320×272**（Looper 真实分辨率）**1010 ms / 145 MB**
- [x] LightGlue 512 kpts **2870 ms / 169 MB**；线程扩展 1t→3t 提速 1.87×，**3 线程后饱和**
- [x] LightGlue 多实例并行：3×1 线程 **2.76×** 加速；3×2 线程 **2.23×**（超核数后有 1.39× 争抢损失）
- [x] DINOv2-base **5190 ms**，峰值 **816 MB**，**实际触发过 OOM 且杀到固件的 `imu_pub` 线程**
- [x] 🟢 **关 ORT arena：816 → 504 MB，延迟不变** —— 零代价省 312 MB
- [x] DINOv2 内存四档量化对比（ORT fp16 / ORT int8 / hbDNN int8 / small）→ 结论：**换运行时 ≫ 换精度**
- [x] ORB 路线全链路：提取 46.5 ms + BF-Hamming 26.0 ms + RANSAC 87.1 ms + PnP 2.6 ms，**47 MB**
- [x] ⭐ **cv2 BF-L2 匹配 SuperPoint 描述子只要 38.6 ms**（knn+ratio 20.6 ms）→ 比 LightGlue 快 **74–140 倍**
- [x] **SuperPoint 线程扩展曲线**：1t 3902 → 2t 2031 → 3t 1342 → **4t 1110（3.52×，饱和）** → 6t 1071 → 8t 1126。**4 线程后是平台期**
- [x] **ORB 完全不扩展**：1/2/4/8 线程都是 38 ms、43 MB（该 OpenCV 构建里 `detectAndCompute` 实质单线程）
- [x] ⚠️ 纠正 SuperPoint ONNX 输入规格：**输入是 `uint8` 不是 fp16**，且有第二个必需输入 `keypoint_threshold`
- [x] 由线程曲线得出：**关 VIO 对单模型延迟几乎没用**（SP 1110→1071 ms），价值在并发度

### 2.3 BPU 分析

- [x] 🟢 **BPU 能被第二个进程并发使用**（实测，方案 6 的一票否决项解除）：固件占 87–96% 时 `hbDNNInitializeFromFiles` rc=0，100 次推理零失败；连**固件自己的 `DStereo.bin`** 都能在第二进程再加载一遍
- [x] 延迟是**双峰不是抖动**：p50 70.6 ms（等一个固件 FC）vs min 5.4 ms（抢到空隙）；纯 BPU 计算 palm_det 2.3 ms / yolo11n@640 5.6 ms / stereonet 139 ms
- [x] 对固件**零可测影响**：FC 速率 12.8→13.0 FC/s（±2% 噪声）、`insight_full` CPU/RSS/线程数全不动、dmesg 0 错误、未重启
- [x] 🔴 `priority=PREEMP` **无效**（FC 下发后不可抢占）
- [x] 🟢 **BPU 路线内存便宜一个量级**：20 MB 基座 + 模型 ×2（stereonet 13.3 MB 模型只占 46.7 MB，对照 ORT SuperPoint 130–148 MB）
- [x] 工具链版本要求：板上 HBRT `3.15.54.0`，固件模型用 builder `1.24.3`/HBRT `3.15.55.0` 编的 → **编 DINOv2 要对齐这一代 OpenExplorer**

- [x] 三个模型的算子构成统计 → **DINOv2 最干净（23 种算子、无 Einsum）· SuperPoint 可切图 · LightGlue 最难（Einsum×36 + IsInf/IsNaN×60）**
- [x] 🔴 证实 **depth 帧率不可调**（无参数，sensor mode 最低 20 fps）→ "降到 5 fps 释放 5.5 TOPS" 做不到
- [x] 🔴 证实 **固件里不存在 VIO 开关字符串**
- [x] 澄清 **BPU 与 CPU 共用同一块 DDR，没有独立显存** → 上 BPU 不省内存

### 2.4 方案设计与代码调研

- [x] 六个算法（三层×2）的资源/延迟/BPU 可行性总表
- [x] **recall 与用时错位**的分析 → 确立"不要加速 LightGlue，而是不用它"
- [x] 九个方案的统一口径参数对比（含 4.5 核 / 6 核两列）
- [x] 5 秒预算论证 → **关 VIO 从前置条件降级为优化项**，LightGlue 被独立判死
- [x] 上游分支盘点：#136 / #150 / #172 / #194 / main 各自做了什么、能复用什么
- [x] 五个工程阻塞定位到代码行（`models_trt.py:1` 硬 import tensorrt · `lekiwi_control.py:5` lerobot 依赖 torch 且从不读轮速 · `looper_bridge_node.py` 不发 IMU · `imu_propagator_node.py:69` RealSense 命名 · `map_node.py:332` 无条件建图）
- [x] 里程计三变体设计（纯轮速 / **轮速+陀螺** / 融合 VIO）+ 误差量化
- [x] 日夜策略：**双时段地图**（这才是 64GB 的真正价值）

### 2.5 项目搭建

- [x] 建 `~/looper/tinynav-x5`（tinynav 独立 clone），分支 `x5/wheel-nav` 基于 `pr136`
- [x] fetch `pr136` / `pr150` / `pr172` / `pr194`
- [x] remote 设置：`origin` = 自己的 fork，`upstream` = UniflexAI（push 已禁用）
- [x] 文档：`docs/x5/README.md`（框架）+ `docs/x5/x5.md`（数据）+ 本文件

---

## 3. 📷 是否需要插着相机 —— 排期用

> 相机长时间通电发热，所以按"必须上板"和"纯 PC"分开排期：**相机在的时候优先清掉 📷 那一列，其余留给相机拔掉之后做。**

| 需要相机 📷 | 不需要相机 💻 |
|---|---|
| ~~T-1 试装 `pydbow3`~~ ✅ **已完成（在 PC 上解决，见下）** | ~~T-2 填 recall 空白~~ 🟢 **经典那两项已补完，见 § 10.2** |
| ~~T-3 验 BPU 并发~~ ✅ **已完成：能并发** | **T-4** `hb_mapper` 编 DINOv2 int8（1–3 天，PC + docker）🔴 **卡在 GPU 驱动** |
| ~~T-13 对比 sensor 库 md5~~ ✅ **已完成：确认不同** | **T-5** backend 抽象（1–2 天）🔴 上板前置 |
| ~~T-15 SP/ORB 线程曲线~~ ✅ **已完成** | **T-7** 轮速里程计节点 + IMU 融合（2–3 天，写代码不需相机）|
| **T-21** 验 RGB（imx415）是否正常 | **T-8** 轻量 Feetech 驱动（1–2 天，写代码不需相机）|
| **T-12** 办公室日夜采数（1 天，还要能走动）| **T-9** `map_node --localization-only`（1 天）|
| ~~抓帧判断是否红外~~ ✅ **已完成：可见光，无主动照明** | **T-10** 地图导出瘦身（1–2 天）|
| ~~把设备文件捞回 PC~~ ✅ **已完成（42 文件 / 35 MB，md5 双向校验）** | **T-11** 双时段地图合并（1 天，借 PR #150）|
| | **T-16** 多方案对比（T-2 之后）|
| | **T-19** 写 `setup_looper.sh`（写脚本不需相机，验证时才需要）|
| | **T-20** 问 Looper 能否降 ion 预留（发消息，零硬件）|

**💻 那一列足够填满好几个晚上，而且都是这个项目工作量最大的部分**（管路开发 + recall 量化）。相机可以拔掉。

⚠️ 两个例外要注意：
- **T-7 / T-8 最终要接底盘验证** —— 代码可以先写完、单元测试可以跑，但闭环测试要等 LeKiwi + 相机都在
- **T-12 需要一台能正常出图的相机** —— 64GB 那台修好之前只能用标准版机器录

---

## 4. 下一步

### 🟢 可立即开工（📷 = 需要插着相机，💻 = 纯 PC）

<a name="t-1"></a>
#### ✅ T-1 · `pydbow3` —— **已完成（2026-08-03）**
原计划是"在板上试装"，实际发现问题不在 aarch64 而在代码本身：上游 `foxis/pyDBoW3` **编不过 OpenCV 4.x**（自带的 `NumpyAllocator` 因基类新增纯虚函数而变成抽象类），x86 上也一样失败。

**解法：自己写了 ~70 行 pybind11 薄壳**（`pydbow3_shim.cpp`），只处理 `(N,32) uint8` 稠密描述子，接口与 `models_trt.py::DBoW3Engine` 期望的 API 源码兼容。已在 x86 docker 里跑通全流程。详见 [`x5.md § 5.6`](x5.md)。

→ **对上板的意义**：这个 shim 不依赖任何 OpenCV 版本耦合的遗留代码，**aarch64 照着同一个 Dockerfile 编一遍即可**。原来"装不上就退方案 5a"的分支不再需要。
→ **但换出了一个新约束**：预训练 ORBvoc（1M 词）常驻 **475 MB**，X5 上和固件抢内存 —— 这催生了下面的小词典实验。

<a name="t-2"></a>
#### 🟢 T-2 · recall 数据空白 —— **经典路线两项已补完（2026-08-03）**
已给 `tool/benchmark/map_retrieval_self_consistency.py` 加了 `--descriptor` 后端开关（`embedding` / `dbow3` / `dbow3-trained` / `bow-cv2` / `bf-l2`），**并修掉了一个方法学问题**：原 harness 的 SE(2) 真值是用被评测描述子自己的 Top-1 拟合的（循环论证），加了 `--transform-json` 让所有后端共用同一个变换。

已跑完（`bag_gt` 为底图，全 CPU 不用 GPU，详见 [`x5.md § 10.2`](x5.md)）：

| 待测组合 | 结果 |
|---|---|
| DBoW3 + ORBvoc | ✅ Day R@1 **88.4%**（打平 DINOv2 VLAD 的 88.87%）· Night R@1 17.6% |
| BoW（cv2.kmeans 自训练） | ✅ 8192 词 Day R@1 **89.5%** / Night R@10 **42.3%**，**反超 1M 词 ORBvoc** |
| SuperPoint BoW | ❌ 待做（需 SP 推理 → GPU 或装 onnxruntime）|
| SP 描述子 + BF-L2 匹配 | ❌ 待做（同上）|
| DINOv2 int8（需 T-4） | ❌ 待做 |
| DINOv2 fp16 在同一套地图上复测 | ❌ 待做（口径对齐用）|

🔴 **剩下四项共同卡在 PC 的 GPU 上**：内核升到 `6.8.0-136-generic`，NVIDIA 内核模块只装了 `-124` 的版本。**一条命令修好，不用重启**：
```bash
sudo apt install -y linux-modules-nvidia-595-open-6.8.0-136-generic && sudo modprobe nvidia && nvidia-smi
```

数据集已下好：`/home/dm/looper/datasets/vlad_eval`（8.1 GB，三个 bag，消息数与 metadata 核对一致）。三张地图在 `/home/dm/looper/x5_work/maps/`，评测结果在 `../results/`。

<a name="t-5"></a>
#### 💻 T-5 · backend 抽象 —— 1–2 天 🔴 **所有上板工作的前置**
`tinynav/core/models_trt.py:1` 硬 `import tensorrt`（第 9 行还有 `from cuda import cudart`），在 X5 上 import 即崩。
- 复用 PR #136 已有的 soft-import
- 拆出 `tinynav/core/backends/{trt,ort,hbdnn}.py`
- 顺手把三层拆成 `retrieval/` `features/` `matchers/`（见 `README.md § 5`）

#### 💻 T-7 · 轮速里程计节点（方案 A + B）—— 2–3 天（闭环测试才需硬件）
- 新写 `wheel_odom_node`：读轮速 → 发 `/slam/odometry`
- 加 IMU 陀螺融合（方案 B）：复用 `imu_propagator_node.py` 的模式，把低频源换成重定位位姿
- `tool/looper_bridge_node.py` 加 **IMU publisher**（现在不发）
- `imu_propagator_node.py:69` 的 `/camera/camera/imu` 要 remap

#### ✅ T-8 · 轻量 Feetech(scservo) 串口驱动 —— **已完成（软件侧）**
原状：`tinynav/platforms/lekiwi_control.py:5` 依赖 `lerobot` → 依赖 `torch` → **X5 装不下**；而且它只 `send_action()`，**从不调 `get_observation()` 读轮速**。

已做：
- `tinynav/platforms/feetech_bus.py` + `omni3_kinematics.py` + `core/wheel_odometry_node.py`（提交 `e811d3f`）替代 lerobot，只依赖 `pyserial` + `numpy`。
- **`lekiwi_control.py` 重写**：不再 import lerobot，也不再碰硬件，退化成纯「路径 → `Twist`」节点。总线由 `wheel_odometry_node -p enable_wheel_command:=true` 单一持有（半双工总线只能有一个主人）。
- 顺带去掉 `scipy` 依赖（约 85 MB，板上只有 ~850 MB 可用）：四元数运算改用 `omni3_kinematics` 里的纯 numpy 版，与 scipy 1.15.3 在 2 万个随机四元数（含近单位、近 180°）上最大差 1.4e-14。
- `np.clip(..., -2.0, 2.0)` 的假限幅换成按几何推导的真实上限 `Omni3Kinematics.max_body_velocity()`：**前进 0.266 m/s、横移 0.230 m/s、自转 1.84 rad/s**。原限幅是真实值的 **7.5 倍**，等于没限。
- 新增 `tool/x5_board/servo_scan.py`：**只读**总线扫描（ttyS3/ttyS5 × 波特率 × ID 1–20），接线完成后第一个跑的工具，已用 PTY 假舵机端到端验证。

**🐛 顺带修掉一个会静默失败的 bug**：`_configure_wheels_for_velocity_mode()` 原本在 EEPROM 未解锁时就写 `Operating_Mode`（addr 33 属 EEPROM 区），且结尾把 `Lock` 置 1。舵机对锁定状态下的 EEPROM 写**回 ACK 且 error=0 然后丢弃** —— 所以第一次之后每次启动都静默失败：轮子留在位置模式，`Goal_Velocity` 全部"写入成功"，车不动且没有任何报错。已改为 `Torque_Enable=0 → Lock=0 → 写 → Lock=1 → Torque_Enable=1` 并**回读校验**，读不到 `mode=1` 就拒绝驱动。`FakeFeetechBus` 现在也模拟这个锁语义，所以测试能真正抓到（`tests/test_wheel_odometry.py`，12/12 通过）。

**仍需硬件才能确定**：轮子转向符号 `wheel_signs`、`wheel_radius`/`base_radius` 标定、以及 `yaw_sign` 的正负（相机 y 轴朝下 vs 底盘 yaw 朝上，见 `lekiwi_control.py` docstring；垫高空转一次即可定）。

#### 💻 T-9 · `map_node` 加 `--localization-only` 模式 —— 1 天
现在 `keyframe_callback`（`map_node.py:332`）每个关键帧无条件跑 `keyframe_mapping()`（写盘存 depth/image + 重算 DINO/SP + 全量 Ceres），额外 **391.8 ms/帧**且随地图增长。

#### 💻 T-10 · 地图导出瘦身 —— 1–2 天
写 `tool/export_reloc_map.py`：丢掉 `depths.db` / `images.db`，只导出关键点 3D 坐标 + 描述子 + 位姿图 + 栅格。38 GB → ORB 路线 ~720 MB / SP 路线 ~500 MB。
> 依据：`keypoint_with_depth_to_3d`（`map_node.py:491`）只按关键点坐标采样 `depth[v,u]`，可预先算成 3D 点。

#### 💻 T-11 · 双时段地图支持 —— 1 天
借 **PR #150**（offline map merge via cross-map loop closure）合并白天/夜晚两张图。日夜方案的核心。

<a name="t-4"></a>
#### 💻 T-4 · `hb_mapper` 编 DINOv2 int8 —— 1–3 天（方案 6 前置）
① fp16 onnx → fp32 ② 冻结动态维为 `1×3×224×224`（BPU 要静态 shape）③ 采 ~100 张真实 Looper 图做校准集（预处理必须与推理完全一致）④ 编译 ⑤ 用 `hb_model_infer` 出描述子喂给 T-2 验 recall

---

### 📷 需要相机在位

<a name="t-3"></a>
#### 📷 T-3 · 验 BPU 能否被第二个进程使用 —— 0.5 h 🔴 **方案 6 的一票否决项**
板上 20 行 ctypes 调 `/usr/lib/libdnn.so` 反复推理任意 `.bin`，同时 `hrut_bpuprofile` 看固件深度推理的 FC 时间是否恶化。
> 💡 **可以用标准版 Looper 做**，不必等 64GB 修好。

#### 📷 T-12 · 办公室日夜采数 —— 1 天 🔴 **日夜方案生死**
同一条路线录三趟 bag：**白天 / 晚上开灯 / 晚上关灯**。然后 ① 肉眼看图判断是否红外、靠窗区域差多少 ② 拿这三段跑 T-2 → 得到**你办公室的真实数字**。
⚠️ 录之前**必须先给相机对时**（64GB 那台 RTC 坏了）。

#### 📷 T-13 · 修 64GB 机器 —— ⚠️ **不是排线问题，别开壳**

已深挖定位：**sensor 被写成 1 lane，而 CSI host 配成 2 lane**，host 永远等不到 D1 进入 LP-11。
决定性证据：用 `multi_isp_vflow -s 1`（1 lane 配置）**两路立体相机都能跑满 ~62 fps，MIPI 错误计数器全 0**。
排线/供电/SoC 已全部排除（两条物理独立链路逐位相同的失败 + lane0 在 1200 Mbps 跑满帧 + 所有错误计数器为 0）。详见 [`x5.md § 2.2`](x5.md)。

- [ ] 🔴 **找 Looper 要正确的 `libsc132gs.so.1.0.0`** —— 板上那个（46768 B，md5 `f171ab12…`）与 2026-04 旧 OTA 包的 blob 完全一致，而 OTA 2.1.2 已把它从 payload 删掉；但 `/userdata/postinst` 只 `cp -a` **从不删文件**，所以旧库永远留着。且 `libcamdev_manager.so` **硬编码 `/usr/hobot/lib/sensor`**，无法用 `LD_LIBRARY_PATH` 绕过
- [ ] 🔴 **拿一台正常的 insight9 对比三个文件**（一锤定音，10 分钟）：
      `md5sum /usr/hobot/lib/sensor/libsc132gs.so.1.0.0`（预期 ≠ `f171ab12…`）、
      `/userdata/install/share/insight_full/config/user_params.json`、
      `/etc/init.d/looper/sensor/insight9/imx415/stereo_camera.json`
- [ ] 🟡 **修 RTC 纽扣电池** —— 它同时是"证据被毁"（crash log 同名互相覆盖）和"OTA 每次开机重装旧包"两个问题的源头
- [ ] 🟡 查清 `/userdata/ota_packet/setting/update_v2_1_6cmsibv830.2.bundle`（名字像 2.1.6）与已装 2.1.2 的关系，以及为什么每次开机都在重装
- [ ] 🟢 备选绕法：既然 1 lane @1200 Mbps 实测能跑 1088×1280 ~60 fps，**对 20 fps 双目绰绰有余** —— 可以问 Looper 能否直接把 `stereo_sensor_name` 换成 1-lane 配置

<a name="t-20"></a>
#### 💻 T-20 · ⭐ 争取降低 ion 预留 —— 🔴 **价值最高的单项**
实测这颗 X5 **物理 DRAM 3.9 GiB，其中 ~2.5 GiB 被 ion 静态预留**，所以 `MemTotal 1307 MB` **是分配决策而非硬件上限**。
内存是全项目最硬的约束（DINOv2 曾 OOM 并杀到固件的 `imu_pub`），而这是唯一能从根上放宽它的手段 —— 比关 VIO（只省 2 核）和降 depth 帧率（只释放 BPU）价值都高。
- [ ] 问 Looper：当前 2.5 GiB 的依据、实际水位、能否降到 1.5–2 GiB、有无运行时查询接口
- [ ] ⚠️ **不要自己硬改** —— ion 池是相机流水线（VPF/ISP/深度推理/BPU 张量）在用的，砍太多会让固件启动失败

#### 📷 T-21 · 验证 RGB（imx415）是否正常 —— 仍未测
`insight_full` 死在第一路立体相机上，RGB 根本没走到（`/sys/class/vps/mipi_phy/status/host` 只有 host2 有痕迹）。板上 sample 工具的 sensor 列表里没有 imx415，无法单独拉起 rx0。
- [ ] 方法 A：先把立体相机改成 1-lane 配置让固件能起来，再看 RGB
- [ ] 方法 B：跑 `insight_full_uvc` —— ⚠️ **它会重配 USB gadget，会断掉 NCM SSH 链路，必须先准备串口**
- [ ] 可推断（非实测）：RGB 用的 `libimx415.so` 是未被覆盖的原厂 BSP 版本（May 21 2026），且在独立的 DPHY group 0 上，不受 sc132gs 库问题影响

---

### 🟡 待确认（不动手，但会影响方案）

- [ ] 🔴 **办公室晚上灯开着还是关着？** → 决定整个日夜方案（全黑则纯视觉不可能）
- [ ] 🔴 BPU 能否被第二个进程并发使用（问 Looper / LooperHub，也可自己验 T-3）
- [ ] 🟡 LooperHub 权限到手后按优先级索取：**BPU 并发** > VIO 开关 > depth 帧率 > 关 RGB 流 > IMU 出话题 > 小深度模型
- [ ] 🟡 Looper 立体相机是否近红外 + 主动投射？（`SC132GS` 是单色全局快门 → 通常配红外）→ 日夜方案的潜在大杠杆
- [ ] 🟡 LeKiwi 是否带 Raspberry Pi？轮速从哪读？ → 决定 T-7 / T-8 的落点

---

### 🔮 后续（Phase 3/4）

<a name="t-6"></a>
- [ ] **T-6a 上板第一版：方案 1**（#136 原样）—— 目标端到端能动，不追 recall
- [ ] **T-6b 备选：方案 5a**（#136 骨架 + #194 的 `bow_retrieval.py` + cv2 BF 替代 LightGlue）
- [ ] T-14 借 **PR #172** 修 `pose_graph_trajectory_publish` 的 332 ms 开销
- [ ] T-15 补测 **SuperPoint 线程扩展曲线**（现在只有 6 线程 = 1010 ms，4.5 核那列全是外推值）
- [ ] T-16 多方案对比：换 yaml 跑，产出资源占用 / recall（白天+夜晚）/ 实际导航表现对比表
- [ ] T-17 方案 6 完整落地：hbDNN Python 封装、建图侧改用量化模型保证描述子同源
- [ ] T-18 可选：SuperPoint 切图上 BPU（→ 方案 6+，0.58 s）
- [ ] T-19 把 `scripts/setup_device_deps.sh` 扩成"从零装好任意 Looper"的脚本 —— **别再手搬文件**

---

## 5. 推荐的开工顺序

```
今天  ─┬─ T-1  试装 pydbow3（0.5h，标准版机器）      ← 定第一版路线
       ├─ T-3  验 BPU 并发（0.5h，标准版机器）       ← 定长期路线
       ├─ T-13 拿正常 insight9 对比 sensor 库 md5     ← 一锤定音，解除板上阻塞
       └─ T-20 问 Looper 能否降 ion 预留             ← 价值最高，解最硬约束
             ↓
本周  ─┬─ T-2  填四个 recall 空白（1–2 天，纯 PC）   ← 所有方案的生死线
       ├─ T-5  backend 抽象（1–2 天，纯 PC）        ← 所有上板工作的前置
       └─ T-12 办公室日夜采数（1 天）               ← 日夜方案生死
             ↓
下周  ─── T-7/T-8/T-9/T-10/T-11 管路开发（纯 PC，与算法无关）
             ↓
之后  ─── T-6 上板第一版 → T-16 多方案对比 → T-17 方案 6
```

**T-1 / T-3 / T-13 都是半小时级的，而且各自否决或解锁一整条路线 —— 先做这三个。**
