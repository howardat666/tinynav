# Looper X5 轮式导航 —— 任务框架

> **目标**：Looper 相机（内含 D-Robotics X5）+ LeKiwi 轮式底盘，在**办公室室内**正常导航，**白天和夜晚都要能稳定重定位**。
> **算力约束**：全部计算跑在相机内的单颗 X5 上，不外接算力。PC 只做离线建图。
> **代码基线**：分支 `x5/wheel-nav`，基于 [PR #136](https://github.com/UniflexAI/tinynav/pull/136)（junlinp）= **方案 1（ORB 全经典）**
> **数据与实测记录**：见 [`x5.md`](x5.md) —— 硬件占用、算法延迟/内存、BPU 分析、九方案参数对比

---

## 索引

| 节 | 内容 |
|---|---|
| [1. 系统架构](#1-系统架构) | 数据流图 · 三条核心设计约束 |
| [2. 里程计方案](#2-里程计方案) | A 纯轮速 / **B 轮速+陀螺（推荐）** / C 融合 VIO |
| [3. 算法方案总览](#3-算法方案总览) | 九方案一句话结论 · 三条已确立的判断 |
| [4. 白天+夜晚策略](#4-白天--夜晚策略) | 双时段地图 · 办公室四种夜间场景 |
| [5. 代码组织](#5-代码组织) | 一个仓库 · 三层插件化 · **五个工程阻塞** · 可借用的上游 PR |
| [6. 路线图](#6-路线图) | Phase 0 六个判定实验 → Phase 4 |
| [7. 待确认事项](#7-待确认事项) | 问谁 · 卡住什么 · LooperHub 索取优先级 |
| [8. 设备与环境](#8-设备与环境) | Looper 台账 · 常用命令 |

**当前状态一句话**：Phase 0 的板上部分**大部分跑完**（0a ✅ / 0f ✅ / SP+ORB 线程曲线 ✅ / **0e 漏了**），🟢 **方案 6 的一票否决项解除 —— BPU 实测可以被第二个进程并发使用**；64GB 机器的 MIPI 故障定位为**旧 OTA 残留的 sensor 库只有 1-lane 寄存器表**（已用正常机器 md5 + 寄存器表 dump 双向证实，**不是排线坏**）。剩下 0b/0c/0d（recall 评测 / int8 编译 / 办公室日夜采数）。

> ⭐ **顺带查出一个可能最重要的杠杆**：这颗 X5 的物理 DRAM 是 **3.9 GiB，其中 ~2.5 GiB 被 ion 静态预留**，`MemTotal 1307 MB` 是分配决策而非硬件上限。内存是全项目最硬的约束，这是唯一能从根上放宽它的手段 → 见 [`x5.md § 2.3`](x5.md)。

---

## 1. 系统架构

```
┌─ 相机内 X5（单芯片，8×A55 / 1307MB / BPU 10TOPS） ────────────────┐
│                                                                   │
│  insight_full（相机固件，不可改）                                  │
│    ├─ 深度图  ──────────────┐                                     │
│    ├─ VIO 位姿（可选不用）   │                                     │
│    └─ IMU                   │                                     │
│           ↓                 ↓                                     │
│  looper_bridge_node ──→ /slam/depth  /slam/keyframe_image          │
│           ↓                                                       │
│  wheel_odom_node ─────→ /slam/odometry     ← 【本项目核心改动】     │
│    （轮速 + IMU 陀螺积分，替代 VIO）                               │
│           ↓                                                       │
│  map_node --localization-only ─→ 重定位位姿（5 秒一次）            │
│    （检索 → 局部特征 → 匹配 → PnP，算法可插拔）                    │
│           ↓                                                       │
│  planning_node ───────→ /planning/trajectory_path                  │
│           ↓                                                       │
│  底盘驱动（轻量 Feetech 串口，替代 lerobot）                        │
└───────────────────────────────────────────────────────────────────┘
        ↑ 地图从 PC 拷入（离线建图，含白天+夜晚双时段）
```

**三条核心设计约束**：

| 约束 | 数值 | 影响 |
|---|---|---|
| **内存** | 1307 MB，**无 swap**，可用 ~911–943 MB<br>（物理 DRAM 其实有 3.9 GiB，**~2.5 GiB 被 ion 预留**）| 🔴 **最硬的瓶颈**。64GB 改装版也一样，一分没多。⭐ **但这是分配决策不是硬件上限** —— 降 ion 预留是最高价值的 LooperHub 请求 |
| **重定位周期预算** | **5 秒**（Jetson 版实测约 3 s，轮速机器人可放宽）| 判死了带 LightGlue 的方案；也让"关 VIO"从前置条件降为优化项 |
| **BPU** | 固件占 87–95% 时间，**帧率不可调** | 神经网络上 BPU 需先验证能否与固件并发 |

---

## 2. 里程计方案

「轮速替代 VIO」有三种做法，**两条都要试，优先纯轮速以缓解硬件压力**。

| | 里程计来源 | CPU 成本 | 5s 平移误差 | **5s yaw 误差** | 状态 |
|---|---|---|---|---|---|
| **A 纯轮速** | 轮速积分 | ~0 | ○ 3–7 cm | 🔴 ○ **1–3°** | 优先实现 |
| **B 轮速 + IMU 陀螺** | 轮速给平移，陀螺给 yaw | ~0 | ○ 3–7 cm | 🟢 ○ **<0.5°** | ⭐ **推荐默认** |
| **C 轮速 + VIO 融合** | VIO 给姿态，轮速抗滑/给尺度 | 🔴 2.11 核 | 最小 | 最小 | 最准但吃 2 核 |

> **关键解耦：纯轮速不需要关 VIO。** 两者正交 —— 下游只认 `/slam/odometry` 话题，VIO 在后台跑不跑不影响正确性，关它只是省 2.11 核。**所以里程计开发不被 LooperHub 权限阻塞。**

**为什么建议 B**：

1. **LeKiwi 是三轮全向轮（omni）**，侧滑比差速底盘严重得多。纯轮速的弱点不是直线距离而是 **yaw 累积**（原地转向、加减速时打滑）。5s 内 3° yaw 误差 → 走 1.5 m 后横向偏 **7.8 cm**，而 `planning_node` 栅格分辨率是 **0.1 m**。
2. 陀螺积分同期误差 **<0.5°**（横向 1.3 cm），好一个数量级，且 **IMU 本来就有**（`insight_full` 的 `imu_pub` 线程），CPU 成本近零。
3. **脚手架已存在且话题现成**：`tinynav/core/imu_propagator_node.py` 干的正是「低频位姿 + 100 Hz IMU 积分 → 高频 `/slam/odometry`」，而它订阅的 `/camera/camera/imu` **就是 Looper 自己发的话题**（实测 bag 里 ~400 Hz）。只需把低频源换成重定位位姿、平移换成轮速积分，**不用 remap、不用改 `looper_bridge`**。

---

## 3. 算法方案总览

重定位是**三层流水线**（全局检索 → 局部特征 → 匹配），每层都有「学习式」和「经典」两种选择。
**六个算法的完整资源/延迟对比表、九个方案的参数对比在 [`x5.md`](x5.md)**，这里只放结论：

| 方案 | 组合 | 单次用时<br>(4.5 核) | 内存 | recall 确定性 | 状态 |
|---|---|---|---|---|---|
| **1 全经典** | DBoW3 + ORB + BF-Hamming | ⬤ **0.38 s** | ~310 MB | 🔴 最低且未知 | ⭐ **第一版就做这个（= PR #136）** |
| 5a | BoW + SuperPoint + BF-L2 | ⬤ **1.50 s** | ~280 MB | 🟡 检索未知 | ✅ 退路 |
| **6** | DINOv2@BPU + SuperPoint + BF-L2 | ○ **1.58 s** | ○ **~350 MB** | 🟢 检索已知 | ⭐ 长期目标（🟢 **BPU 并发已验证可行**）|
| ~~0 / 2 / 3 / 4~~ | 含 LightGlue 的方案 | 4.5–20 s | — | — | ❌ **5s 预算判死** |

**三条已确立的判断**：

1. **不要试图加速 LightGlue，而是不用它** —— cv2 BF 匹配 38.6 ms 就能替代它的 2870 ms。它同时是 CPU 上最慢的（占 57% 总耗时）和最难上 BPU 的（Einsum×36 + IsInf/IsNaN×60）。
2. **影响 recall 最大的那层用时可以很小；用时最大的那层对 recall 只是中等影响** —— 检索层是第一道门（决定性）但只要 5 ms；匹配层占 57% 耗时却只是中等影响。这个错位是所有优化的依据。
3. **上 BPU 不省内存** —— X5 的 BPU 与 CPU 共用同一块 DDR，没有独立显存。省内存靠换运行时（ORT → hbDNN），不靠上 BPU。

⚠️ **四条路线各自的 recall 都没有数据**（DINOv2 int8 / SuperPoint BoW / DBoW3+ORBvoc / SP+BF），全部可以在 PC 上量化，是优先级最高的工作。详见 [`x5.md § 10.2`](x5.md)。

---

## 4. 白天 + 夜晚策略

现有数据是**跨条件**（白天地图 vs 夜晚查询）—— 问题的最难形式：DINOv2 CLS **78.80% → 23.34%**，patch VLAD **88.87% → 31.21%**。

**解法：离线建图 + 64GB 存储 → 建「白天 + 夜晚」双时段地图。** 夜晚查询匹配夜晚关键帧，退化为同条件检索，预期回到 80%+。办公室 ~30×30 m 双时段约 1.5 GB，64GB 机器 `/userdata` 剩 44 G，毫无压力。

> **这才是 64GB 机器真正的价值** —— 不是装更大的地图，而是装同一场景的多个时段版本。
> 🎁 [PR #150](https://github.com/UniflexAI/tinynav/pull/150)（junlinp，offline map merge via cross-map loop closure）正好是双时段地图合并需要的工具，已 fetch 到本地 `pr150` 分支。

### 🔴 已实测：立体相机是**可见光单色**，机身**没有主动照明**

原来指望"`SC132GS` 是单色 → 可能是近红外 → 日夜差异不大"这个大杠杆，**已被实测推翻**（详见 [`x5.md § 10.2`](x5.md)）：

- **抓到真实帧对照**：画面里的**绿色 LED 出口标志**（~525 nm，NIR 输出基本为零）在 mono 立体图里清晰可见、位置与 RGB 一致 → 镜头前没有 IR 滤片，**成像靠环境可见光**
- **主动照明全盘零命中**：无 `/sys/class/leds`；`strings` 搜固件和整个 `/etc/init.d/looper/` 的 `infrared|projector|illuminat|emitter|led|laser` 全部 0 命中；`gpio_init.sh` 只有 reset/power-on 序列
- 唯一的 PWM 是 **20 Hz 帧同步触发**（`period=50000 occupied=CAMSYS`，i2c 确认两颗 sensor 都在外触发从模式）—— **是立体硬同步，不是补光**

**所以日夜问题跑不掉。出路只有两条：双时段地图（灯开着的场景足够）+ 必要时外加照明。**

🔴 **待确认（决定整个日夜方案）：办公室晚上灯是开着还是关着？**

| 实际场景 | 能否重定位 |
|---|---|
| 晚上**灯开着** | 🟢 与白天差异不大 + 双时段地图 → 基本无损 |
| 晚上灯关、有走廊/应急灯漏光 | 🟡 极低照度，取决于 sensor 感光下限和运动模糊 |
| 晚上**全黑** | 🔴 **纯视觉直接失效，机身帮不上忙**，必须外加照明 |
| 靠窗区域 | 🟡 白天有阳光 → 日夜差异最大处，双时段建图可覆盖 |

🔬 **1 分钟就能做的关键验证**：**拿电视遥控器对着立体镜头按键，看 `infra1` 图里有没有亮点。**
- 有亮点 → 无 IR-cut，**850 nm 红外补光方案成立**（不刺眼、不影响办公）
- 没亮点 → 装了 IR-cut，只能上可见光补光

---

## 5. 代码组织

**一个仓库，不要为每个方案分仓。** 理由：① 多方案对比要求同一张地图、同一段 bag、同一套评测脚本，分仓会让对比失真；② 三层本来就可插拔，配置开关就够；③ 分仓 = 各自修 bug + merge 地狱，一个 `map_node` 的 fix 要改 4 遍。

```
tinynav/core/
├── retrieval/     dinov2.py  vlad.py  bow.py  dbow3.py
├── features/      superpoint.py  orb.py
├── matchers/      lightglue.py  bf.py
└── backends/      trt.py  ort.py  hbdnn.py      ← PC 用 TRT，X5 用 ORT/hbDNN
experiments/
├── configs/       plan1_orb.yaml  plan5a_bow_sp_bf.yaml  plan6_dino_bpu.yaml
└── results/       每次跑出来的 recall / 延迟 / 内存 / 实际表现
```

CLI 形如 `--retrieval {dinov2,vlad,bow,dbow3} --features {superpoint,orb} --matcher {lightglue,bf} --backend {trt,ort,hbdnn}`，「对比多个版本」就是换个 yaml。

### 5.1 已核查的五个工程阻塞

| | 问题 | 要做什么 |
|---|---|---|
| 🔴 | `tinynav/core/models_trt.py:1` 硬 `import tensorrt`（第 9 行还有 `from cuda import cudart`）| **backend 抽象必须先做**。PR #136 已有 soft-import 可复用 |
| 🔴 | `tinynav/platforms/lekiwi_control.py:5` `from lerobot...` —— **lerobot 依赖 torch，1307 MB 的 X5 装不下**；且该节点只 `send_action()`，**从不调 `get_observation()` 读轮速** | ① 轮速里程计节点新写 ② 写轻量 Feetech(scservo) 串口驱动替代 lerobot |
| 🟢 | ~~`looper_bridge_node.py` 不发 IMU~~ · ~~`imu_propagator_node.py:69` 订阅 RealSense 命名要 remap~~ | ❌ **两条都作废** —— **`/camera/camera/imu` 就是 Looper 自己的话题名**（Looper 的 topic 命名刻意仿照 RealSense），由 `insight_full` 直接发布，bag 里实测 ~400 Hz。所以 `imu_propagator_node.py` **原样就对**，方案 B 的 IMU 数据现成可用，不用改 `looper_bridge` |
| 🟡 | `map_node.py:332` `keyframe_callback` 每个关键帧无条件跑 `keyframe_mapping()`（写盘 + 重算 DINO/SP + 全量 Ceres），额外 391.8 ms/帧 | 加 `--localization-only` 模式 |

### 5.2 可直接借用的上游 PR

| PR | 作者 | 状态 | 内容 | 对我们的价值 |
|---|---|---|---|---|
| **#136** | junlinp | 🟢 OPEN | BoW loop-closure（ORB + BF + DBoW3）| ⭐ **本项目基线** = 方案 1，唯一现成的完整经典路线 |
| **#150** | junlinp | 🟢 OPEN | offline map merge via cross-map loop closure | ⭐ **双时段地图合并**正需要 |
| **#172** | junlinp | 🟢 OPEN | gate pose graph trajectory publishing | 修掉实测到的 332 ms `pose_graph_trajectory_publish` 开销 |
| **#194** | xiaolefang-dm | 🔴 **CLOSED** | SuperPoint BoW map node | ⚠️ 整体不能合（与 main 的 VLAD 冲突），但 **`tinynav/core/bow_retrieval.py` 可零冲突复用**（148 行，只 import cv2+numpy）→ **方案 5a 的检索层** |
| #167 / #171 | xiaolefang-dm | 🟢 OPEN | auto-localization assist（yaw sweep until localized）/ VIO guard | 重定位失败时的兜底策略 |

> **两条路都有现成代码，不会卡死**：方案 1 的检索层在 #136（但依赖 `pydbow3`，不在 PyPI，aarch64 待验）；方案 5a 的检索层在 #194（纯 Python，零新依赖）。半小时试装 `pydbow3` 就能定先走哪条。

---

## 6. 路线图

### Phase 0 — 六个判定实验（都不写正式代码，可完全并行，一周收口）

拿到这些结果之前，任何路线选择都是赌博。

| | 实验 | 在哪 | 工作量 | 决定什么 | 状态 |
|---|---|---|---|---|---|
| **0a** | ctypes 调 `/usr/lib/libdnn.so` 反复推理，同时 `hrut_bpuprofile` 看固件 FC 时间是否恶化 | 板上 | 0.5 天 | 🔴 方案 6 生死 | 🟢 **已完成 —— BPU 可并发，固件零影响**，见 [`x5.md § 4.2`](x5.md) |
| **0b** | 给 `tool/benchmark/map_retrieval_self_consistency.py` 加描述子后端开关，跑齐四个 recall 空白 | PC | 1–2 天 | 🔴 **所有方案的 recall 生死线** | ✅ 可立即开工 |
| **0c** | `hb_mapper` 把 DINOv2 编成 int8 `.bin` | PC + OpenExplorer | 1–3 天 | 🔴 0b 第一行的前置 | ✅ 可立即开工 |
| **0d** | **办公室日夜采数**：同一路线录三趟 bag（白天 / 晚上开灯 / 晚上关灯），肉眼看图 + 跑 0b | 办公室 | 1 天 | 🔴 **日夜方案生死 + 是否红外** | 🟡 需一台能用的 Looper |
| **0e** | X5 上试装 `pydbow3` | 板上 | 0.5 h | 🟡 第一版走方案 1 还是 5a | 🔴 **还没做** —— 相机在位时漏掉了 |
| **0f** | 64GB 新机基线复核 | 板上 | 0.5 h | 🟡 后续所有数字的前提 | ✅ **已完成，见 [`x5.md § 2.1`](x5.md)** |

### Phase 1 — PC 上做与算法无关的管路（工作量最大，不被任何 X5 不确定性阻塞）

| # | 工作 |
|---|---|
| 1 | **轮速里程计节点**（方案 A）+ **IMU 陀螺融合**（方案 B）—— 复用 `imu_propagator_node.py` 的模式 |
| 2 | `looper_bridge_node.py` **加 IMU publisher** |
| 3 | **轻量 Feetech(scservo) 串口驱动**替代 lerobot |
| 4 | `map_node` 加 `--localization-only` 模式 |
| 5 | **三层插件化 + backend 抽象**（其他上板工作的前置）|
| 6 | 地图导出瘦身 `tool/export_reloc_map.py`（38 GB → ~500 MB）|
| 7 | **双时段（白天/夜晚）地图支持** —— 借 PR #150 |

### Phase 2 — 上板第一版（方案 1，基于 PR #136）

目标是**端到端能动**（轮速 → 重定位 → 规划 → 底盘），**不追 recall**。
X5 环境准备：`pyproject.toml` 依赖解耦、装 scipy/numba/opencv-headless **4.6.0.66**（4.8 会挂，需 numpy≥1.22）、编译 `tinynav_cpp_bind`。

### Phase 3 — 多方案对比迭代

换 yaml 跑不同组合，产出对比表：硬件资源占用 / recall（白天 + 夜晚）/ 实际导航表现。

### Phase 4 — 方案 6（DINOv2 上 BPU）

前提是 0a（BPU 并发）和 0c（int8 编译）都通。完整风险清单与工作清单见 [`x5.md § 7.3`](x5.md)。

---

## 7. 待确认事项

| 问谁 | 问题 | 卡住什么 |
|---|---|---|
| 🔴 **自己** | **办公室晚上灯开着还是关着？** | 整个日夜方案 |
| 🔴 **Looper** | **64GB 机器：`/usr/hobot/lib/sensor/libsc132gs.so.1.0.0`（md5 `f171ab12…`）是 2026-04 旧 OTA 包的残留，只有 1-lane 寄存器表；2.1.2 已把它从 payload 删掉，但 `postinst` 只 cp 不 delete。请给正确的库，或确认这台是 1-lane 变体** | 🔴 **这台的相机流拉不起来**。⚠️ **不是排线问题**，详见 [`x5.md § 2.2`](x5.md) |
| 🔴 **Looper / LooperHub** | ⭐ **ion 预留 ~2.5 GiB 能否降到 1.5–2 GiB？** 现在的依据是什么、实际水位多少、有无运行时查询接口 | 🔴 **内存是全项目最硬约束，这是唯一能从根上放宽的手段**，价值高于关 VIO 和降 depth 帧率 |
| ~~🔴 Looper~~ | ~~BPU 能否被第二个进程并发使用~~ | 🟢 **已自己实测：能，且对固件零影响** → [`x5.md § 4.2`](x5.md) |
| 🟡 **硬件** | **修 64GB 机器的 RTC 纽扣电池** | 时间每次启动重置 → crash log 同名互相覆盖、OTA 版本判断失效（很可能每次开机重装旧包，把旧 sensor 库刷回去）|
| 🟡 Looper / LooperHub | VIO 开关、depth 帧率、IMU 出话题、关 RGB 流 | 优化项（非阻塞）|
| 🟡 自查 | Looper 立体相机是否近红外 + 主动投射 | 日夜方案的潜在大杠杆 |
| 🟡 — | LeKiwi 是否带 Raspberry Pi？轮速从哪读？ | 里程计节点落点 |

### LooperHub 权限到手后的索取优先级

| 优先级 | 要什么 | 收益 | 代价 |
|---|---|---|---|
| 🔴 **0** | ⭐ **降低 ion 预留**（现 ~2.5 GiB / 物理 3.9 GiB）| 🟢 **可用内存 +几百 MB —— 唯一能从根上解掉最硬约束的手段** | ⚠️ 砍太多会让相机流水线申请缓冲失败，需 Looper 评估水位 |
| ~~🔴 1~~ | ~~BPU 能否被第二个进程使用~~ | 🟢 **已实测解决，不用问了** | — |
| 🟢 2 | VIO 开关 | +2.11 核（4.5 → 6.6）。⚠️ **实测后降级**：SuperPoint 4 线程就饱和、ORB 完全不扩展，所以这对**单次重定位延迟几乎没帮助**；价值在"能并行跑别的东西" | 失去方案 C 的融合选项 |
| 🟡 3 | depth 帧率下调 | 释放 BPU 给方案 6 + 省 depth 线程 63.7% CPU | ⚠️ **规划输入率跟着降**：5 fps 时避障每 200 ms 更新，0.3 m/s 下每次 6 cm，可接受但需实测 |
| 🟢 4 | 关 RGB/JPEG 流 | +13.7% CPU | 导航不用 RGB 就白赚 |
| 🟢 5 | IMU 原始数据出 ROS 话题 | 方案 B 要用 | — |
| 🟢 6 | 换小深度模型 `DStereoV2.5_int16_544_448.bin` | 省 BPU | 深度质量可能降 |

---

## 8. 设备与环境

### 8.1 Looper 台账

| 设备 | `soc_uid` 尾段 | eMMC | `/userdata` | 状态 |
|---|---|---|---|---|
| 桌面 A | 未记录 | 15.7 GB | 9.2 G | 🟢 正常（我的 bench 文件在这台）|
| 桌面 B | `...a4408a00120040` | 15.7 GB | 9.2 G | 🟢 正常 |
| 北京狗挂载 | `...9d4b790012004` | 15.7 GB | 9.2 G | 🟢 正常 |
| **64GB 改装版** | `...56c08a00120040` | **62.6 GB** | **53 G（剩 44 G）** | 🔴 **MIPI 故障，固件起不来** |

⚠️ `/etc/machine-id` 在所有设备上完全相同（固件烧入），认设备只能用 `soc_uid`。

### 8.2 常用命令

```bash
# 连相机
sshpass -p 'looper@0731' ssh -o StrictHostKeyChecking=no root@169.254.10.1

# 认设备 + 看容量
cat /sys/class/socinfo/soc_uid; cat /sys/block/mmcblk0/size; df -h /userdata

# 看固件状态（起不来时先看这个）
systemctl status S99all_run.service --no-pager -l
journalctl -u S99all_run.service --no-pager -n 40
dmesg | grep -iE "sc132|mipi|csi|vin "

# 看资源（hrut_* 需要 LD_LIBRARY_PATH，否则缺 libalog.so.1）
export LD_LIBRARY_PATH=/usr/hobot/lib:$LD_LIBRARY_PATH PATH=$PATH:/usr/hobot/bin
hrut_somstatus            # CPU/BPU 频率与 BPU ratio
free -m; nproc

# 相机固件控制
/etc/init.d/ota_project/scripts/insight-ctl s99 {start|stop|restart|status|log}
```

### 8.3 本地分支

| 分支 | 内容 |
|---|---|
| `x5/wheel-nav` | ⭐ 工作分支，基于 `pr136` |
| `pr136` / `pr150` / `pr172` / `pr194` | 已 fetch 的上游 PR，见 § 5.2 |
| `main` | `origin/main` 快照（`25e705e`）|
