# Looper X5 轮式导航 —— 任务框架

> **目标**：Looper 相机（内含 D-Robotics X5）+ 轮式底盘，在**办公室室内**正常导航，
> **白天和夜晚都要能稳定重定位**。
> **算力约束**：全部计算跑在相机内的单颗 X5 上，不外接算力。PC 只做离线建图。

> # 🔴 先读这个：本目录里并存两套方案，别搞混
>
> 本文和 `x5.md` 写于 **2026-08-04**，通篇是**旧方案**（LeKiwi 三轮全向 + ORB + DBoW3）的语境。
> 项目在 08-14 变更方向，但**代码和大部分文档仍然是旧方案的**，所以读任何一句都要先确认它属于哪一套。
>
> | 维度 | 旧方案（2026-08-04 立项） | **新方案（当前）** |
> |---|---|---|
> | 底盘 | LeKiwi 三轮**全向** | **两轮差速 + 单脚轮**（仿扫地机三轮） |
> | 布局 | — | 相机 + **驱动轮前置**，脚轮后置，重心前移 |
> | 电机 | Feetech STS3215 **总线舵机** —— 3 线**串联菊花链**接驱动板，X5 经 `ttyS3` 直连总线 | **有刷直流 + PWM**（TB6612 双 H 桥）—— 每个电机**单独**接驱动板，中间加 **ESP32-S3**，`ttyS3` @115200 转 GPIO<br>🔑 **接线方式就是最快的判据** |
> | 执行器代码 | `TINYNAV_ACTUATOR=wheel` + `wheel_odometry_node` | `TINYNAV_ACTUATOR=diffcar` + `tinynav/platforms/diffcar_control.py` |
> | 特征/描述子 | **ORB** | **SuperPoint** INT8，跑 BPU（39.4 ms/帧） |
> | 检索 | **DBoW3** 自训练层次词典（约 300 MB 固定开销） | **VLAD** K=256（词典 256 KB，冻结、跨地图共享） |
> | 里程计 | 相机 VIO | **轮速里程计**（ESP32 固件积分，实测航向漂移 **< 0.05°/m**）；VIO 待评估后关闭 |
> | 通信 | USB-C 直连 `169.254.10.1` | **WiFi**（type-C 接网卡 = host 模式）；调试期切 device 模式走 USB |
> | 深度帧率 | 12.8 Hz（固件默认 `depth_frame_skip=1`） | **5 Hz**（`depth_frame_skip: 4`，BPU 89%→38%） |
> | 板上内存 | `MemTotal` 1307 MiB | **1787 MiB**（`ion_cma` 512→32 MiB，已持久化） |
>
> ## 文档归属
>
> | 属于**旧方案**，按上表换算后再读 | 属于**新方案** / 与方案无关（持续更新） |
> |---|---|
> | 本文（`README.md`）—— 九方案对比、里程计三变体、路线图 | [`x5_full_pipeline_plan.md`](x5_full_pipeline_plan.md) ⭐ **新方案的权威方案书** |
> | [`x5.md`](x5.md) —— 九方案参数表、ORB/DBoW3 recall 数据 | [`diffcar.md`](diffcar.md) —— 差速车接入、ESP32 协议 |
> | [`wheel_odometry.md`](wheel_odometry.md) —— LeKiwi **三轮全向**运动学 | [`board_bringup.md`](board_bringup.md) —— 板上环境、时钟、内存、USB 角色 |
> | [`servo_bus.md`](servo_bus.md) —— Feetech 总线丢包排查<br>⚠️ 但它的 **UART 拓扑表和 `ttyS3` 独占性结论仍然有效** | [`depth_frame_skip.md`](depth_frame_skip.md) —— 12.8→5 Hz |
> | [`odometry_comparison.md`](odometry_comparison.md) | [`nav_field_results.md`](nav_field_results.md) §7 —— 方向变更记录 |
>
> ## 两处曾经写错、现已推翻的结论
>
> 1. 🟢 **「X5 不能做 USB host」是错的** —— 两个控制器都是 OTG，有运行时角色开关。
>    详见 [`usb_host_mode.md`](usb_host_mode.md)、[`board_bringup.md`](board_bringup.md) § 2.6。
> 2. 🟢 **「固件没有关 VIO 的开关」是错的** —— LooperHub `54d4d7e` 已加 `vio_enabled` 启动开关
>    （2026-08-11）。同理 `depth_frame_skip` 是**固件侧**的 StereoNet 分频器（`d457ee0`），
>    不是订阅端丢帧，所以它确实省 BPU。详见 [`x5.md`](x5.md) § 3。

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

**本目录其他文档**

| 文档 | 内容 |
|---|---|
| [`x5.md`](x5.md) | 硬件占用 · 算法延迟/内存 · BPU 分析 · 九方案参数对比 |
| [`board_bringup.md`](board_bringup.md) | **把重定位真跑在 X5 上**：环境配置 · 时钟与 QoS 陷阱 · 依赖 · 板上实测结果 |
| [`depth_frame_skip.md`](depth_frame_skip.md) | 把 depth 从 12.8 Hz 降到 5 Hz 换热余量 |
| [`fix_64gb_mipi.md`](fix_64gb_mipi.md) | 64GB 相机 MIPI 故障的定位与修复 |
| [`wheel_odometry.md`](wheel_odometry.md) | LeKiwi 三轮全向底盘的轮速里程计 |
| [`todo.md`](todo.md) | 待办清单 |

**当前状态一句话**：Phase 0 已完成 0a / 0b / 0e / 0f。⭐ **最重要的结论：决定 recall 的是描述子而非检索算法** —— 白天经典路线（ORB + 自训练 DBoW3）R@1 **90.1%**，超过 main 现行 DINOv2 配置且快 4 倍省 2 倍内存，**第一版直接上方案 1**；但夜间 ORB 只有 21.6%，而同样检索算法换成 SuperPoint 描述子就有 **44.0%**（DINOv2 调好 48.8%）。由此浮出一个**没人测过的最优候选 —— DBoW3 + SuperPoint**（兼得 #210 的 recall 与本项目的检索速度）。🟢 BPU 已实测可被第二个进程并发使用；64GB 机器的 MIPI 故障定位为**旧 OTA 残留的 sensor 库只有 1-lane 寄存器表**（**不是排线坏**）。剩 0c（int8 编译）/ 0d（办公室日夜采数）。PC 的 GPU 已于 2026-08-03 修复（内核 -136 缺配套 nvidia 模块包）。

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

| 方案 | 全局检索 | 局部特征 | 匹配 | 单次用时<br>(4.5 核) | 内存 | Day R@1 | Night R@1 | 状态 |
|---|---|---|---|---|---|---|---|---|
| **1 全经典** | **自训练 DBoW3** | ORB | BF-Hamming | ⬤ **0.38 s** | ⬤ **~160 MB** | ⬤ **90.1%** | ⬤ 21.6% | ⭐ **第一版（= PR #136）** |
| 5a | cv2 BoW | SuperPoint | BF-L2 | ⬤ **1.50 s** | ~280 MB | **96.0%** | **44.0%** | ✅ 夜间需求确认后走这条 |
| **⭐ 5b（未测）** | **自训练 DBoW3** | **SuperPoint** | BF-L2 | ○ ~1.5 s | ○ ~200 MB | ○ ≥96%? | ○ ≥44%? | ⭐ **理论最优，没人做过** |
| 6 | DINOv2@BPU | SuperPoint | BF-L2 | ○ **1.58 s** | ○ ~350 MB | 89.8% | **48.8%** | 🟡 长期目标（🟢 BPU 并发已验证）|
| ~~0 / 2 / 3 / 4~~ | 含 LightGlue 的方案 | | | 4.5–20 s | — | — | — | ❌ **5s 预算判死** |

**五条已确立的判断**：

1. 🔑 **决定 recall 的是描述子，不是检索算法** —— 同样用 cv2 BoW：SuperPoint 夜间 44.0%，ORB 夜间只有 9.9%（4.4 倍）。选 BoW 还是 DBoW3 只影响速度和内存（扁平词典暴力最近邻 0.15 s/帧 vs 层次树 4.8 ms）。三层的耦合关系详见 [`x5.md § 13.1`](x5.md)。
2. 🟢 **白天所有路线都够用（86–96%）** —— 经典路线 90.1% 已超过 main 现行的 DINOv2 配置，而且便宜一个数量级。**方案 1 是"性价比最优的第一版"，但不是精度最优**（SuperPoint BoW-128 的 96.0% 才是）。
3. 🟡 **夜间：ORB 不行（最好 21.6%），但换局部特征能救一大半** —— SuperPoint BoW 44.0%、DINOv2 VLAD-64 48.8%。机理是**夜间 ORB 特征点从 916 掉到 333 个/帧**，卡在特征提取而非检索。仍离可用有距离，需叠加双时段地图 / 感知混淆剪枝 / 补光。
4. **不要试图加速 LightGlue，而是不用它** —— cv2 BF 匹配 38.6 ms 就能替代它的 2870 ms。它同时是 CPU 上最慢的（占 57% 总耗时）和最难上 BPU 的（Einsum×36 + IsInf/IsNaN×60）。
5. **上 BPU 不省内存** —— X5 的 BPU 与 CPU 共用同一块 DDR，没有独立显存。省内存靠换运行时（ORT → hbDNN），不靠上 BPU。

⚠️ **一个必须记住的口径问题**：main 现在默认的 VLAD 词典大小是 512，而 PR #210 的扫描显示**那个默认值落在崩溃区**（256 词时夜间已只剩 10.7%）。所以"经典超过学习式"准确说是"超过了 main 当前的默认配置"，**不是"超过了调好的 DINOv2"**。

⚠️ 剩余数据空白：**DBoW3 + SuperPoint（最值得做）** / DINOv2 int8 / SP+BF 的 recall / 用共享变换重跑 #210 那条线 / #212 剪枝的增益。详见 [`x5.md § 10.4`](x5.md)。

---

## 4. 白天 + 夜晚策略

现有数据是**跨条件**（白天地图 vs 夜晚查询）—— 问题的最难形式。**所有方法的 Day → Night 落差**（R@1 @0.5 m，详见 [`x5.md § 10.2`](x5.md)）：

| 描述子 | 最好的配置 | Day → Night |
|---|---|---|
| **ORB** | 自训练 DBoW3 10 万词 | 88.8% → ⬤ **21.6%** |
| **SuperPoint** | cv2 BoW 128 词 | 96.0% → **44.0%** |
| **整幅图像** | DINOv2 patch VLAD 64 词 | 86.2% → **48.8%** |

**没有一个够用**，但落差的**根因是 ORB 在低照度下提不出特征点**（916 → 333 个/帧），换 SuperPoint 能救回一倍多。

**解法：离线建图 + 64GB 存储 → 建「白天 + 夜晚」双时段地图。** 夜晚查询匹配夜晚关键帧，退化为同条件检索，预期回到 80%+。办公室 ~30×30 m 双时段约 1.5 GB，64GB 机器 `/userdata` 剩 44 G，毫无压力。**另一条正交手段：[PR #212](https://github.com/UniflexAI/tinynav/pull/212)（已合入 main）的感知混淆关键帧剪枝** —— 作者实测 night 有 3.3% 的问题关键帧、day 0%，说明夜间失败有一部分不是算法弱而是地图里本来就有辨识度极低的帧。

> **这才是 64GB 机器真正的价值** —— 不是装更大的地图，而是装同一场景的多个时段版本。
> 🎁 [PR #150](https://github.com/UniflexAI/tinynav/pull/150)（junlinp，offline map merge via cross-map loop closure）正好是双时段地图合并需要的工具，已 fetch 到本地 `pr150` 分支。

### 🔴 已实测：立体相机是**可见光单色**，机身**没有主动照明**

原来指望"`SC132GS` 是单色 → 可能是近红外 → 日夜差异不大"这个大杠杆，**已被实测推翻**（详见 [`x5.md § 10.3`](x5.md)）：

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
| **0b** | 给 `tool/benchmark/map_retrieval_self_consistency.py` 加描述子后端开关，跑齐 recall 空白 | PC | 1–2 天 | 🔴 **所有方案的 recall 生死线** | 🟢 **已完成** —— ORB 三档由本项目实测，SuperPoint / DINOv2 各档取自 PR #210，见 [`x5.md § 10.2`](x5.md)。⭐ 剩 **DBoW3 + SuperPoint** 未测 |
| **0c** | `hb_mapper` 把 DINOv2 编成 int8 `.bin` | PC + OpenExplorer | 1–3 天 | 🟡 方案 6 前置（**优先级已下调** —— 经典路线白天够用）| ✅ **可开工**，GPU 已于 2026-08-03 修复 |
| **0d** | **办公室日夜采数**：同一路线录三趟 bag（白天 / 晚上开灯 / 晚上关灯），肉眼看图 + 跑 0b | 办公室 | 1 天 | 🔴 **日夜方案生死 + 是否红外** | 🟡 需一台能用的 Looper。⚠️ **现在是最高优先的未知量**：0b 显示夜间 ORB 21.6% / SuperPoint 44.0%，**够不够用完全取决于你办公室夜间的真实光照**，这个数决定要不要为夜间付 SuperPoint 的 1.0 s |
| **0e** | 试装 `pydbow3` | 原计划板上 | 0.5 h | 🟡 第一版走方案 1 还是 5a | 🟢 **已完成** —— 上游编不过 OpenCV 4.x，自写 70 行 pybind11 shim 解决，aarch64 可照搬，见 [`x5.md § 5.6`](x5.md) |
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

- [`grid_resolution.md`](grid_resolution.md) — 局部栅格分辨率 0.1→0.05 的完整账（内存实测、矮障碍反而更差、OctoMap 为什么不对症）
- [`odometry_vio_calibration.md`](odometry_vio_calibration.md) — 轮速里程计标定：⚠️ 那份「±0.6%」是 VIO 时代的，VIO 关掉后 `diffcar_calib_vio.py` 已经跑不了；**图像闭合实测偏航高报 2.4%、左右不对称 2.3%**（2026-09-02 增补）
- [`vio_to_wheel_odom.md`](vio_to_wheel_odom.md) — 用轮速里程计换掉 VIO 的 CPU 账、唯一阻碍（姿态）、迁移步骤
- [`command_vs_actual.md`](command_vs_actual.md) — 指令/编码器/VIO 三路速度的实测：滞后 0.2 s、`C/A`=1.000、原地小角速度以前完全不转及其修法（§9~§11）；**§13 逐跳延迟：相机双目 120 ms、排队 230 ms、planning 计算 200 ms**
- [latency_chain.md](latency_chain.md) — **全链延迟**：图像采集→轮速指令的逐跳测量方法与空闲基线；四个节点的 `LAT` 行 + `lat_report.py`；⚠️ 纠正了 `in_age` 其实是**位姿**龄（不含任何传感器延迟）这个长期误读，以及位姿内容那 50 ms 的隐形延迟
- [planning_cost.md](planning_cost.md) — 「明明能过去就是不走」的三层原因：承诺段门限把半径算两遍、障碍势垒在 soft 边界是断崖、代价函数缺静止代价
- [vio_reset_investigation.md](vio_reset_investigation.md) — 固件 VIO 重启的九格消去实验：CPU/内存/BPU/DDS/优先级全不是；含调度优先级 A/B 与 insight_full 线程拆解
- [map_odom_and_reloc.md](map_odom_and_reloc.md) — 「跑偏」与「要手动重开」的机制：map->odom yaw 日志 bug（tilt 越小越野）、拟合等权 150 秒历史（recent5 残差 p90 123°）、重锁三道前置、高度门 0.30 m

---

## 9. 板上当前生效配置（2026-09-02 19:00 基准）

改参数之前先看这里，改完**回来更新**。真值在板上：
`tr '\0' '\n' < /proc/$(pgrep -f "[u]vicorn")/environ | grep TINYNAV`

### 9.1 写在 `/userdata/x5/env.sh` 里的（备份 `env.sh.bak-0902b`）

全部 7 条，`grep -E "^export TINYNAV_" /userdata/x5/env.sh` 逐字核对过：

| 变量 | 值 | 为什么 |
|---|---|---|
| `TINYNAV_ACTUATOR` | `diffcar` | 平台 |
| `TINYNAV_CAMERA_HEIGHT_M` | `0.124` | 实测；估错会把整片地板标成障碍 |
| `TINYNAV_RAYCAST_STEP` | `3` | 墙面召回（step=10 在 1~3 m 只召回 42~56%） |
| `TINYNAV_LOW_OBS` | `0` | 矮障碍层在硬件上贡献 76% 的格子且全是噪声 |
| `TINYNAV_LOW_OBS_MIN_PTS` | `2` | 关掉之后无效，留着是为了 `LOW_OBS=1` 时 A/B |
| `TINYNAV_MAX_VX` | `0.25` | 0.35 试过，两次撞击的直接原因（一拍从 fc=1.17 到 0.03） |
| `TINYNAV_ALLOW_REVERSE` | `1` | 楔住后 110 条轨迹被拒 96 条，关着倒车就一个动作都没有 |

⚠️ **`/userdata/x5/env.sh` 在 `app_start.sh` 之前 source，而且不在 git 里** ——
仓库里那份是 `tool/x5_board/env.sh`，两份内容不同。改错那份白改一轮。

### 9.2 走代码默认、没写进 `env.sh` 的

| 变量 | 默认 | 含义 |
|---|---|---|
| `TINYNAV_MAP_ODOM_HALF_LIFE_S` | `20` | map→odom 约束的时间半衰期（0=关，回到等权 150 秒历史） |
| `TINYNAV_RELOC_MAX_Z_DEV_M` | `0.30` | 解的相机高度必须贴住参考关键帧 |
| `TINYNAV_RELOC_RELOCK_MIN_ODOM_M` | `1.0` | 重锁前置：streak 内里程计位移 |
| `TINYNAV_RELOC_RELOCK_MAX_CLEAN_FRAC` | `0.30` | 重锁前置：当前坐标系还有多少干净约束 |
| `TINYNAV_RELOC_RELOCK_MAX_SPREAD_M` | `1.5` | 重锁前置：触发解的候选散布 |
| `TINYNAV_PLAN_LATEST_ONLY` | **`0`** | 0=旧的 FIFO 取用；1=回调只存、20 Hz 定时器取最新。**待真车 A/B** |
| `TINYNAV_MAX_INPUT_AGE_S` | `0.5` | 超过这么老的同步集直接丢 |
| `TINYNAV_NOPROGRESS_ESCAPE_S` | `5.0` | no-progress 脱困的时间预算 |
| `TINYNAV_ROBOT_Z_BOTTOM` | `-0.11` | 障碍波段下界（离地 +0.014 m） |
| `TINYNAV_MIN_WALL_SPAN_M` | `0.05` | 障碍 z 跨度门限 |

### 9.3 启动日志里该长什么样

```
Robot: diffcar (circle r=0.1175m hull=0.122m, cam=(0.067,0.05), ctrl=(0.0,0.0),
                safety_r=0.05m, hard/soft=0.17/0.27m, z_band=[-0.11,+0.40],
                vx<=0.25/rev0.2 yaw<=0.6 (clamp 0.8/1.2), reverse=True)
planning pose source: /wheel/camera_pose
planning pose sync: approximate slop=0.06s, FIFO intake
obstacle: raycast step=3 span>=0.05 zbot=-0.11(离地+0.014m) dilation=0 low_obs=off
reverse: ENABLED (TINYNAV_ALLOW_REVERSE, 一次脱困上限 0.30m) —— 楔住时会倒车脱困
```

🔑 **这四行是唯一可靠的"跑的是什么配置"凭据** —— 别拿 `env.sh` 的内容当结论
（`app_start.sh` 里的 `${VAR:-默认}` 会覆盖，而它还有一份陈旧的孪生文件）。

### 9.4 决策行里的新字段

```
... cycle=0.20s stamp_lag=0.31s in_age=0.19s
```

- `in_age` = 进规划回调时数据已经多老 = **相机 + bridge + 排队**
- `stamp_lag − in_age` = **本节点的计算耗时**
- `in_age − 0.163` = **排队掉的时间**（0.163 s 是 `/slam/depth` 到手时的常态数据龄）

⚠️ 排队是**随负载出现**的：车基本没动时 `in_age` p50 只有 0.19 s，负载上来才到 0.33~0.50 s。

### 2026-09-03 新增的环境变量（都在 planning_node）

| 变量 | 默认 | 作用 |
|---|---|---|
| `TINYNAV_DECISION_LOG_HZ` | **0 = 逐拍** | `decision:`/`candidates:`/`best forward:` 的节流。原来写死 1 Hz，而规划周期 0.25 s —— 5 拍只看得到 1 拍，把"障碍首次出现在多远"量偏了 |
| `TINYNAV_ROUTE_PROBE_M` | 2.0 | 沿全局路线往前探多远，出 `route_block` / `route_clear` |
| `TINYNAV_ROUTE_UNPIN_ON_BLOCK` | **1 = 开** | `route_clear < hard_clearance` 时本拍 `w_path_follow=0`，日志打 `ROUTE-UNPINNED` |
| `TINYNAV_BARRIER_INCLUDE_START` | **0 = 不含** | 障碍势垒的最小净空是否算轨迹起点。含起点时它对所有候选恒等，退化成常数 |

判据速查：`route_clear` 比 `front_clearance` 早报警（前向探针对侧面障碍是瞎的）；
`decision:` 行里 `route_block=inf` 表示路线畅通。详见 `planning_cost.md`。
