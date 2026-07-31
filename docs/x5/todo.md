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
- [x] 🔴 **64GB 机器 MIPI D-PHY 故障定位**（`lane state error 0x1000d`）—— I2C 正常、时钟正常、高速数据通道失败；冷启动+手动重启均 100% 复现 → 物理层问题
- [x] 发现 64GB 机器 **RTC 是坏的**（`hwclock -r` = 1970），系统时间每次启动恢复到同一时刻

### 2.2 算法实测（X5 板上，ONNXRuntime 1.18 CPU）

- [x] SuperPoint @**320×272**（Looper 真实分辨率）**1010 ms / 145 MB**
- [x] LightGlue 512 kpts **2870 ms / 169 MB**；线程扩展 1t→3t 提速 1.87×，**3 线程后饱和**
- [x] LightGlue 多实例并行：3×1 线程 **2.76×** 加速；3×2 线程 **2.23×**（超核数后有 1.39× 争抢损失）
- [x] DINOv2-base **5190 ms**，峰值 **816 MB**，**实际触发过 OOM 且杀到固件的 `imu_pub` 线程**
- [x] 🟢 **关 ORT arena：816 → 504 MB，延迟不变** —— 零代价省 312 MB
- [x] DINOv2 内存四档量化对比（ORT fp16 / ORT int8 / hbDNN int8 / small）→ 结论：**换运行时 ≫ 换精度**
- [x] ORB 路线全链路：提取 46.5 ms + BF-Hamming 26.0 ms + RANSAC 87.1 ms + PnP 2.6 ms，**47 MB**
- [x] ⭐ **cv2 BF-L2 匹配 SuperPoint 描述子只要 38.6 ms**（knn+ratio 20.6 ms）→ 比 LightGlue 快 **74–140 倍**

### 2.3 BPU 分析

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

## 3. 下一步

### 🟢 立即可做（PC 侧，不被任何硬件阻塞）

<a name="t-1"></a>
#### T-1 · 试装 `pydbow3` 定第一版路线 —— 0.5 h ⚡ 最先做
在**标准版** Looper（不是坏的那台 64GB）上试装 `pydbow3`。
- 装得上 → 直接跑 [T-6a](#t-6) 方案 1
- 装不上 → 把 PR #194 的 `tinynav/core/bow_retrieval.py`（148 行，纯 cv2+numpy）接进来走 [T-6b](#t-6) 方案 5a

<a name="t-2"></a>
#### T-2 · 填掉四个 recall 数据空白 —— 1–2 天 🔴 **所有方案的生死线**
给 `tool/benchmark/map_retrieval_self_consistency.py` 加**描述子后端开关**（现在写死读 `vlad_descriptors.db`），然后一次跑齐：

| 待测组合 | 对标基线 | 决定哪条路线 |
|---|---|---|
| DBoW3 + ORBvoc | fp16 DINOv2 的 day 78.80% / 88.87% | **方案 1（当前基线）** |
| SuperPoint BoW | 同上 | 方案 5a |
| SP 描述子 + BF-L2 匹配 | SP + LightGlue | 方案 5a / 6 共同 |
| DINOv2 int8（需 T-4 先出模型） | fp16 同款 | 方案 6 |

数据集：`hf download --repo-type dataset UniflexAI/rosbag_tinynav_vlad_eval`

<a name="t-5"></a>
#### T-5 · backend 抽象 —— 1–2 天 🔴 **所有上板工作的前置**
`tinynav/core/models_trt.py:1` 硬 `import tensorrt`（第 9 行还有 `from cuda import cudart`），在 X5 上 import 即崩。
- 复用 PR #136 已有的 soft-import
- 拆出 `tinynav/core/backends/{trt,ort,hbdnn}.py`
- 顺手把三层拆成 `retrieval/` `features/` `matchers/`（见 `README.md § 5`）

#### T-7 · 轮速里程计节点（方案 A + B）—— 2–3 天
- 新写 `wheel_odom_node`：读轮速 → 发 `/slam/odometry`
- 加 IMU 陀螺融合（方案 B）：复用 `imu_propagator_node.py` 的模式，把低频源换成重定位位姿
- `tool/looper_bridge_node.py` 加 **IMU publisher**（现在不发）
- `imu_propagator_node.py:69` 的 `/camera/camera/imu` 要 remap

#### T-8 · 轻量 Feetech(scservo) 串口驱动 —— 1–2 天
`tinynav/platforms/lekiwi_control.py:5` 依赖 `lerobot` → 依赖 `torch` → **X5 装不下**。而且它只 `send_action()`，**从不调 `get_observation()` 读轮速**。
写 ~200 行纯串口驱动替代，同时提供轮速读取。

#### T-9 · `map_node` 加 `--localization-only` 模式 —— 1 天
现在 `keyframe_callback`（`map_node.py:332`）每个关键帧无条件跑 `keyframe_mapping()`（写盘存 depth/image + 重算 DINO/SP + 全量 Ceres），额外 **391.8 ms/帧**且随地图增长。

#### T-10 · 地图导出瘦身 —— 1–2 天
写 `tool/export_reloc_map.py`：丢掉 `depths.db` / `images.db`，只导出关键点 3D 坐标 + 描述子 + 位姿图 + 栅格。38 GB → ORB 路线 ~720 MB / SP 路线 ~500 MB。
> 依据：`keypoint_with_depth_to_3d`（`map_node.py:491`）只按关键点坐标采样 `depth[v,u]`，可预先算成 3D 点。

#### T-11 · 双时段地图支持 —— 1 天
借 **PR #150**（offline map merge via cross-map loop closure）合并白天/夜晚两张图。日夜方案的核心。

<a name="t-4"></a>
#### T-4 · `hb_mapper` 编 DINOv2 int8 —— 1–3 天（方案 6 前置）
① fp16 onnx → fp32 ② 冻结动态维为 `1×3×224×224`（BPU 要静态 shape）③ 采 ~100 张真实 Looper 图做校准集（预处理必须与推理完全一致）④ 编译 ⑤ 用 `hb_model_infer` 出描述子喂给 T-2 验 recall

---

### 🔴 被 64GB 机器 MIPI 故障阻塞

<a name="t-3"></a>
#### T-3 · 验 BPU 能否被第二个进程使用 —— 0.5 h 🔴 **方案 6 的一票否决项**
板上 20 行 ctypes 调 `/usr/lib/libdnn.so` 反复推理任意 `.bin`，同时 `hrut_bpuprofile` 看固件深度推理的 FC 时间是否恶化。
> 💡 **可以用标准版 Looper 做**，不必等 64GB 修好。

#### T-12 · 办公室日夜采数 —— 1 天 🔴 **日夜方案生死**
同一条路线录三趟 bag：**白天 / 晚上开灯 / 晚上关灯**。然后 ① 肉眼看图判断是否红外、靠窗区域差多少 ② 拿这三段跑 T-2 → 得到**你办公室的真实数字**。
⚠️ 录之前**必须先给相机对时**（64GB 那台 RTC 坏了）。

#### T-13 · 修 64GB 机器
- [ ] 断电开壳**重新插拔立体相机 MIPI 排线**（最可能的原因，改装机）
- [ ] 换带独立供电的 USB hub（次可能：MIPI PHY 对电压敏感）
- [ ] 问俊霖改装过程 + 板上有没有"改装后曾经出过图"的证据

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

## 4. 推荐的开工顺序

```
今天  ─┬─ T-1  试装 pydbow3（0.5h，标准版机器）      ← 定第一版路线
       ├─ T-3  验 BPU 并发（0.5h，标准版机器）       ← 定长期路线
       └─ T-13 拔插 64GB 机器排线                   ← 解除板上阻塞
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
