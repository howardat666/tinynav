# X5 全流程方案（建图 + 重定位都在板上）

初版 2026-08-15，2026-08-17 大幅修订：BPU 路线已在板上真机跑通，多处初版外推被实测推翻。
检索层实验数据在 `x5_work/results_x2/SUMMARY.md`，方法学在同目录 `README.md`。

> **测试设备 ≠ 部署设备。** 本文的 BPU 性能数据来自一台**只装了相机固件、没有部署 tinynav** 的
> Looper，它只用于性能验证和代码准备。
> - 🟢 **性能数字可迁移**（同型号 X5、同 BPU、同 CPU），但 `sp_backbone.bin` 仍需在部署设备上加载验证一次
> - 🟢 **内存预算已在目标设备实测**（2026-08-20），并已把 `MemTotal` 从 1307 扩到 **1787 MiB** ——
>   见「内存」一节。大索引（293 MB）现在装得下，K 和关键帧密度可以纯按精度选

> 标 **[实测]** 的来自板上或工具链实测；标 **[估]** 的仍是外推。

## 方案

**SuperPoint（INT8 主干跑 BPU）+ VLAD 检索 + 固化词典**

| 层 | 选型 | 依据 |
|---|---|---|
| 特征提取 | SuperPoint，**主干跑 BPU** | 唯一必须跑的模型；ORB 夜间只有 32.4%（R@3） |
| 特征后处理 | **自己写的 numpy 版**（非 ONNX 子图） | 板上 78.8 ms vs ONNX 子图 190.8 ms **[实测]** |
| 全局检索 | VLAD K=256 | 词袋要 53 分钟训词典换 0 收益；DBoW3 实测 R@3 49.5% |
| 词典 | **PC 预训、固件固化**（256 KB） | 见「固化词典」 |
| 索引存储 | **float32** | float16 慢 6.5×、int8 慢 2.3×（A55 无原生窄位宽矩阵乘）**[实测]** |
| 关键帧密度 | **约 23 cm**（现状 5.8 cm） | R@3 反涨 1.6 点，索引降到 1/4 **[实测]** |
| 局部匹配 | cv2 BFMatcher + crossCheck | LightGlue 在 X5 要 2.78 s，不可用 |
| 位姿 | solvePnPRansac | 不变 |

## BPU 落地（已在板上跑通）

工具链 `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8`，`hb_mapper` 1.24.3，`--march bayes-e`。

### 模型拆分

整模型无法转换 —— 关键点数量本身是动态的（输出 shape 含 `Wherekpts_dim_0`），BPU 要求全静态。
拆分点在 `Softmax`（节点 24）之前：主干 0–23（Conv×12 / Relu / MaxPool）上 BPU，
其余（Softmax / TopK / GridSample / ScatterND / Where…）留 CPU。

输入定死 `1×1×640×544`（⚠️ Looper 的 infra 图是 **640 行 × 544 列**，不是 544×640）。

### 编译与实测

```
15/15 算子分配到 BPU，零 CPU 回退，单一子图
编译器报告  latency = 15.69 ms/帧   FPS = 63.75   DDR = 24.3 MB/帧
sp_backbone.bin = 1.64 MB
```

| | 板上实测 |
|---|---|
| `hbDNNInfer` 全程（含数据搬运、cache flush） | **39.4 ms** |
| SuperPoint 端到端（BPU 主干 + numpy 后处理） | **146.7 ms** |
| 同一台板子纯 CPU 跑完整 ONNX | **3260 ms** |
| **加速** | **22.2 倍** |

⚠️ 编译器的 15.69 ms 是**纯计算**，实测 39.4 ms，多出的 24 ms 是 7 MB 输出的 memcpy。
⚠️ `hb_mapper checker` 报的 40.2 ms / 179 MB 是**未优化编译**的假象，不要引用。
⚠️ BPU 输出是 **NCHW 且已反量化**（`quanti=0`），不是从 hbdk 参数推断的 NHWC。

### 运行时接口

板上有 `/usr/lib/libdnn.so` 和完整 C 头文件 `/usr/include/dnn/`，但**没有 Python 绑定，也没有 pip**。
已写 ctypes 封装（`tool/x5_board/hbdnn.py`），结构体照 `hb_dnn.h` / `hb_sys.h` 逐字段对齐。

⚠️ `libdnn.so` 依赖 `libcnn_intf.so.1` / `libhbmem.so.1` / `libalog.so.1`，都不在默认库路径，
必须 `LD_LIBRARY_PATH=/usr/hobot/lib`。

💡 `hbDNNInferCtrlParam` 带 `priority` 字段（0–255，255 为抢占级），**优先级可以每次推理指定**，
不一定要改固件的 `hw_prio_en`。未验证在 `hw_prio_en=0` 时是否生效。

## INT8 精度：零损失

夜间查询 1163 帧，参考 `map_gt` 1120 帧，词典 `map_day`（无泄漏），共享 transform：

| 配置 | R@1 | **R@3** | R@5 | R@10 |
|---|---|---|---|---|
| fp32 全流程 | 61.6% | 66.6% | 68.6% | 70.8% |
| INT8 描述子 + fp32 关键点 | 62.3% | 67.8% | 69.4% | 71.5% |
| **INT8 全流程**（板上真实条件） | **64.6%** | **68.4%** | 70.2% | 72.2% |
| **INT8 全流程 + 关键帧 23 cm** | 62.0% | **70.0%** | **71.5%** | **75.5%** |

**INT8 全流程不低于 fp32**（高出的 1.8–3.4 点部分在 k-means 自身 ±1.8 点噪声内，
不应解读为"量化更好"）。地平线真量化（per-tensor）的描述子余弦 0.9617，还优于
ORT 代理（per-channel）的 0.9567 —— 代理实验是保守下界。

⚠️ **一个曾经误导过我的中间量**：BPU 与 fp32 的关键点重合率只有 0.426，VLAD 余弦只有 0.790。
那是拿「BPU 查询」对「fp32 参考」比，**部署中不存在这种混搭**（建图和重定位都在板上用 BPU，
偏移一致互相抵消）。上表最后两行才是部署条件。

⚠️ **绝对值不能和 `SUMMARY.md` 的表混用。**本实验必须重新提描述子，只能从地图的
`video.mp4` 解码（h264 有损，压缩比 21:1），比用 `features.db` 里的原始描述子低约 9 点。
**推论：地图里的 mp4 只能留档，不能拿来重提特征。**

## 后处理必须自己写

初版估 47 ms（拿 PC 的 4.2% 占比乘 1110 ms），**错了**：板上实测 ONNX 后处理子图 **190.8 ms**，
占比 17% 而非 4.2% —— A55 跑访存和控制流密集的算子比 x86 吃亏得多。

自己写的 numpy 版（`tool/x5_board/sp_post.py`）与原模型**逐位一致**
（关键点 recall=1.000，描述子 cos=1.00000）：

| 步骤 | 板上耗时 |
|---|---|
| heatmap（softmax + 重排） | 21.9 ms |
| NMS（阈值筛出约 1 万候选后贪心抑制） | 32.9 ms |
| 描述子采样 | 24.0 ms |
| **合计** | **78.8 ms**（ONNX 子图的 2.4 倍快） |

省下的 112 ms 来自算法：ONNX 图忠实执行 **5 次全分辨率 640×544 的 9×9 max-pool**，
而在阈值筛出的候选点上做贪心 NMS 结果完全相同。

⚠️ 试过"先取 top-N 候选再 NMS"，recall 掉到 0.82 —— **分数低但位置孤立的点在 NMS 后本该入选**，
截断会静默丢关键点。不要做这个优化。

关键参数（从 ONNX 图里挖出，改动必须对齐）：NMS 半径 4、simple_nms 迭代 2 次、TopK=512 固定、
`GridSample align_corners=1`、采样前 desc_map 先做通道 L2。

## 🟢 2026-08-20 板上真机复测（新代码路径，`insight_full` 在跑）

代码已同步到板上（`/userdata/x5/tinynav`），`make_sp_extractor()` 正确选中 `SuperPointBPU`，
`sp_backbone.bin` md5 与本地一致。**这是第一次用接线后的代码在真机上跑，不再是手写命令。**

```
[BPU_PLAT] BPU Platform Version(1.3.6)  soc info(x5)
[DNN] Runtime version = 1.23.10_(3.15.54 HBRT)   model builder version = 1.24.3
加载 0.14 s   输出 (1,65,80,68) + (1,256,80,68) float32
```

⚠️ 版本不一致告警照旧出现（hbrt 3.15.54 vs model build 3.15.55），已知、无害，见 `x5_work/bpu/README.md`。

| 阶段 | 记录值（08-17） | **08-20 实测（真实 infra1 帧）** | 差异 |
|---|---|---|---|
| BPU `hbDNNInfer` | 39.4 ms | **40.9** | ✅ 吻合 |
| heatmap | 21.9 | **25.3** | ✅ 接近 |
| NMS | 32.9 | **35.8** | ✅ 接近 |
| **描述子采样** | **24.0** | 🔴 **58.1** | **慢 2.4×** |
| 四项合计 | 146.7 | **160.0** | |
| **端到端（含封装）** | — | 🔴 **201.0**（min 158.8 / max 214.4） | 比合计多 **41 ms** |

🔴 **两个新发现：**

1. **`sample_descriptors` 比记录慢 2.4 倍**（58.1 vs 24.0 ms），是唯一偏离的一项。
   候选原因未区分：板上 numpy 已从 1.21.5 升到 **1.26.1**、`OPENBLAS_NUM_THREADS=2` 限制了
   grid-sample 的矩阵乘、或与 `insight_full` 抢 CPU。**要查先固定这三个变量。**
2. **封装层有约 41 ms 开销**，记录的 146.7 ms 不含它。主要是
   `ascontiguousarray(float32).reshape()` + `/= 255.0` 在 640×544 上产生一个 1.4 MB 的新数组，
   加上每帧一次 `asyncio.run()` 新建事件循环。⚠️ 导航代码本来就是这么调的
   （`asyncio.run(self.extractor.infer(...))`），**所以 201 ms 才是部署会看到的数**。
   可用预分配缓冲省掉一次分配，但先量清楚再改。

**对时延预算的影响**：SuperPoint 从 146.7 → **201 ms**，重定位单次从约 750 → **约 805 ms**，
预算 5 s，**仍然宽裕**。下面那张表的 SuperPoint 一行按 201 读。

⚠️ **不要用随机噪声图测这条链** —— 阈值 5e-4 下噪声图的 NMS 候选爆炸，端到端量到 **506 ms**，
是病态输入不是真实性能。我先踩过一次。

其余同时验证通过的：`SuperPointMatcher` 直接吃 BPU 输出（自匹配 512/512）、
打包词典可加载 `(256, 256) float32`、`compute_vlad` 板上 **19.4 ms**、
移动后的旧路径 `tool/x5_board/{hbdnn,sp_post}.py` 在板上不存在（无残留副本）。
板上依赖齐全：numpy **1.26.1**、cv2 4.11.0、einops 0.8.2、codetiming 1.4.0、scipy 1.15.3、numba 0.61.2。

## 重定位单次成本

| 步骤 | 1120 帧地图 | 来源 |
|---|---|---|
| SuperPoint（BPU + 后处理） | 146.7 ms | **[实测]** |
| BPU 排队 | ~35 ms | **[估]** 单核串行，前面压着 `insight_full` 的 80 ms 任务 |
| VLAD 查询编码（K=256） | 131.8 ms | **[实测]** |
| 索引搜索（float32） | 267.2 ms | **[实测]** |
| 匹配 ×3 候选 | 116 ms | **[实测]** |
| PnP | ~50 ms | **[估]** |
| **合计** | **~750 ms** | 预算 5 s |

**瓶颈已换人**：VLAD 那两项占 53%，SuperPoint 只占 20%。BPU 做完后下一个该优化的是检索。

BPU 占用：关键帧 23 cm（约 2.6 Hz）× 39.4 ms = **10%**；若维持现状的 10.5 Hz 则是 41%，
超过 `insight_full` 留下的余量。**关键帧密度同时是精度、内存和 BPU 占用三者的杠杆。**

**不需要为 SuperPoint 预留 BPU** —— 冗余买不到吞吐（用不掉），只能买一点延迟
（占用从 87% 降到 44% 才省 17 ms），不划算。

## 建图与回环

🔴 **x5 分支的建图回环被禁用了，而 main 是开着的 —— 这是本分支的功能回退。**

| | main | x5 分支 |
|---|---|---|
| 实现 | `find_loop()` 函数 | `LoopClosure` 类（x5 重构并加了 bow 模式） |
| 是否在跑 | 🟢 **在跑**：`build_map_node.py:770 detect_loop_closure()` → `find_loop` → 位姿图加回环边 | 🔴 **禁用**：`TINYNAV_MAP_LOOP_CLOSURE=0`，三处调用点注释掉 |
| 位姿图约束 | 相邻帧 + **回环边** | 只有相邻帧（链式，无环可闭，优化退化成轻微平滑） |

禁用原因写在代码注释里：`LoopClosure` 一旦以 bow 模式构造，`DBoW3Engine` 就加载 ~300 MB 词典常驻，
占 1307 MB 板子的 23%，是它逼出了 `--play-rate` 和 `--sync-queue-size` 限制。

**VLAD 模式拆掉了这个两难** —— 词典只有 256 KB，不必为省内存牺牲回环。

### 重新打开回环的代价

| | |
|---|---|
| 建图时间 | 每关键帧多一次检索 + 约 1.1 次 PnP 验证；关键帧 23 cm（280 帧）总共多约 **27 秒** |
| 内存 | 建图过程要常驻 VLAD 索引：1120 帧 293 MB，280 帧 73 MB |
| 收益 | **消除累积漂移**（现状是纯里程计位姿链） |

### 阈值标定（2026-08-17 完成）

**相似度不是最终判据。**候选要先做特征匹配 + PnP，`len(inliers) >= 100` 才真的加进位姿图
（main `build_map_node.py:801`）。所以阈值只是粗筛，低 precision 由 PnP 兜底，
标定目标是**高召回 + 控制候选数量**。

**回环（同 session，`map_gt` 1120 帧，真回环 = 空间 <0.5 m 且时间 >100 关键帧，63 对）：**

```
真回环:  mean=0.406  p5=0.373  min=0.367
假配对(>5m): mean=0.248  p95=0.294  p99=0.319  max=0.430
```

| 阈值 | 召回 | 每帧假候选 | precision |
|---|---|---|---|
| 0.34 | 100% | 1.91 | 5.6% |
| **0.35** | **100%** | **1.11** | 9.2% |
| 0.37 | 96.8% | 0.51 | 17.6% |
| 0.40 | 61.9% | 0.09 | 44.3% |
| **0.90（原值）** | **0%** | 0 | — |

🔴 **VLAD 的相似度尺度整个落在 0.25–0.43，原值 0.90 比真回环的最大值还高一倍 —— 直接打开开关，
回环一次都不会触发。**

🔴 **第二个瓶颈：`loop_top_k = 1`。**按排序看召回：top-1 **34.5%**、top-3 65.5%、top-5 **79.3%**。
即使阈值对了，只取 top-1 也漏掉 65%。

**重定位（跨 session，夜间查询 × 白天地图）：**

```
正确配对(<0.5m): mean=0.312  p5=0.216  min=0.149
错误配对(>5m):   mean=0.239  p95=0.287  p99=0.304  max=0.444
```

两个分布**重叠严重**（正确的 p5 低于错误的 p95），**没有阈值能分开**：0.25 召回 83% 但每查询留 499 个候选，
0.30 召回只剩 57%。而 top-3 相似度的 p5 只有 0.297 —— 阈值定在 0.30 会让 5% 的查询连候选都返回不了，
**静默失败**。所以重定位应当靠 top-k 排序 + PnP，阈值只做兜底。

### 已改的参数

`LOOP_CLOSURE_DEFAULTS`（`build_map_node.py`）按后端分别给默认值，两个节点共用：

| mode | loop_thr | loop_top_k | reloc_thr | reloc_top_k |
|---|---|---|---|---|
| embedding | 0.90 | 1 | 0.85 | 3 |
| bow | 0.90 | 1 | 0.85 | 3 |
| **vlad** | **0.35** | **5** | **0.10** | 3 |

PnP 内点门槛 100 不动 —— 它才是真正的判据。

### ⚠️ 标定的三个局限

1. **样本小**：63 对真回环、29 帧有回环机会，全来自一条轨迹。0.35 是起点不是定论。
2. **这条轨迹只有 2.6% 的帧有回环机会**（72 米走廊来回走，重访少）。**打开回环对这个 bag 收益有限**，
   绕圈场景会大得多 —— 不能只凭这一条数据判断回环值不值得开。
3. **阈值在 K=256 上标的**。若因内存降 K，相似度分布会变，必须重标。

### 打开的建议顺序

**目标设备内存实测 → 定 K 和关键帧密度 → 用定下的 K 重标阈值 → 再打开开关。**
建图过程要常驻 VLAD 索引，而内存预算目前整个未知；打开回环 + 索引常驻可能正好把建图推到 OOM，
那会表现成"建图莫名其妙被杀"，很难查。

## 内存：已实测，并已把板子从 1307 扩到 1787 MiB

🟢 **2026-08-20:目标设备实测完成,而且顺手把 ion 预留砍掉一块。**
`ion_cma` 512 → 32 MiB,`MemTotal` **1307 → 1787 MiB**,已持久化开机生效。
操作步骤和陷阱见 [`board_bringup.md § 2.7`](board_bringup.md)。

### 目标设备实测值

| 场景 | `MemAvailable` |
|---|---|
| 只跑相机固件 + app(改之前) | 914 MiB |
| 只跑相机固件 + app(**改之后**) | **1392 MiB** |
| 导航全栈跑着(改之后,未主动导航) | **1104 MiB** |
| 导航中(按历史基准推算) | **约 603 MiB**(改之前约 125 MiB) |

导航栈真实消耗 **约 789 MB** —— 不是早前引用的 954 MB,那个数是各节点 RSS 相加,
把共享库(numpy、ONNX runtime)在多个进程里重复计了。

节点实测(全栈运行):`insight_full` 193 MB / 139% CPU、`looper_bridge_node` 195 MB / 84.5% CPU、
`planning_node` 234 MB / 45.2% CPU、`uvicorn` 102 MB / 40.2% CPU。

### 🟢 大索引现在装得下了

| VLAD 配置 | 索引 | 改之前 | **改之后** |
|---|---|---|---|
| 23 cm 密度,280 关键帧 | 73 MB | 余 52 MB(很薄) | **余 530 MB** |
| 1120 关键帧,K=256 | 293 MB | **差 168 MB,装不下** | **余 310 MB** |

**所以 K 和关键帧密度可以纯按精度选,不再被内存绑住。** 而 23 cm 密度恰好也是精度最高的
(INT8 + 23 cm:70.0% R@3 / 75.5% R@10,比全量 1120 帧的 68.4% 还高)。

### 两种检索后端的内存缩放不同

| | 固定开销 | 每关键帧 |
|---|---|---|
| **DBoW3** | **~300 MB**（词典树，代码注释实测） | 约 4 KB **[估]**（稀疏 BoW 向量） |
| **VLAD K=256** | 0.26 MB | **262 KB**（K×256×4，稠密） |

每帧差约 65 倍，**交叉点在约 1160 关键帧**：小地图 VLAD 更省，大地图 DBoW3 更省。

| 关键帧数 | DBoW3 | VLAD K=256 |
|---|---|---|
| 750 | 303 MB | **197 MB** |
| 1160 | 305 MB | 304 MB（打平） |
| 3000 | **312 MB** | 786 MB |
| 10000 | **340 MB** | 2.6 GB |

### 索引 dtype：float32，不要压窄 **[实测]**

| K=256, 1120 帧 | 内存 | 搜索 |
|---|---|---|
| **float32** | 293.6 MB | **267 ms** |
| float16 | 146.8 MB | 1725 ms（**慢 6.5×**） |
| int8 | 73.4 MB | 604 ms（慢 2.3×） |

A55 没有原生窄位宽矩阵乘，每次都要转 float32，转换比省下的带宽贵。
**省内存要靠降 K 或降关键帧密度，不要靠压 dtype** —— 后两者同时省内存和时间。
现在内存宽裕了,这条更没有理由违反。

### 还能再挖多少

另两个 ion 池还有 750 MiB 和 911 MiB 空着(`cma_reserved` 274/1024、`carveout` 113/1024),
但**现在不能砍** —— 那两个水位是在「彩色关、3D 网格关、深度 5 Hz、我们的 SP 没跑 BPU」下测的,
真实峰值从未测过,而且我们的 SuperPoint 是往 `carveout` 里加的(约 +25~50 MB)。
要砍先在最重工况复测峰值。

## 大地图：分层 + 分区

VLAD 索引正比于关键帧数，现有方案（23 cm 间距 + K=256）覆盖约 **750 关键帧 ≈ 195 米轨迹**。
超过这个规模需要：

**两级索引。**粗层常驻（小），细层按空间分区加载。冷启动用粗层全局搜索定位到大致区域，
加载该区域的细层做精确检索；之后位置已知，只搜附近。

粗层的要求比细层低得多 —— 只要正确区域落在 top-50/100 即可，不需要 top-1 准。候选做法：

| 粗层做法 | 1 万帧的粗层 | 需要新东西 |
|---|---|---|
| 更狠的降采样（3 米间距） | 82 MB | ❌ |
| 小 K（K=4，1024 维） | 40 MB | ❌ |
| 块内降维（每个 256 维残差块共享投影） | 40 MB | ✅ 要训投影 |
| 粗层用 DBoW3 | 300 MB（不随帧数长） | ❌ |

（全局 PCA 不可行：65536→4096 的投影矩阵本身就要 1 GB。）

💡 现实角度：**冷启动全局重定位未必是常态**。开机位置往往已知（充电桩、上次停车点），
真正需要全局搜索的是"被搬走"的情况。

## 固化词典

实测：用**完全另一张地图**训的词典，精度与用参考地图自己训的持平（69.6% vs 68.5%，噪声内）。
所以词典不必每建一张图重训。

**这连带解决一个鸡生蛋问题。**VLAD 词典原本要等所有关键帧提完才训得出来，建图进行到一半时
VLAD 根本不存在，所以回环只能靠 DINOv2 顶着。词典固化后建图第一帧就能算 VLAD，
**回环检测和重定位共用一套描述子，DINOv2 可以彻底移除**。

### 词典采集要求：**尚未确定**

曾断言「必须跨光照采集」，依据是夜间词典查夜间比白天词典高 7.5 点 —— 但那个对比里词典训练数据
包含了查询集本身，是数据泄漏，**结论已撤回**。

现有干净证据只支持一条：**异源词典不掉分**（`map_day` 训的 69.6% vs `map_gt` 训的 68.5%），
而这两张图是同一场景。**「异场景词典是否可用」「是否需要覆盖夜间」都还没有答案**，
需要另一个场景的 bag。

同一条纪律适用于**量化校准集** —— 它也是训练数据。本文 INT8 结果用 `map_day` 的 150 帧校准，
`map_night` 从未进入校准。

## 已完成的代码改动

| 文件 | 改动 | 验证 |
|---|---|---|
| `tinynav/core/vlad.py` | 去 scipy（板上没有，且是顶层 import，一 import 就崩）；`cKDTree` 换矩阵乘；残差累加换排序分组 | 描述子 cos=1.00000000，`compute_vlad` 快 5.1× |
| `tinynav/core/build_map_node.py` | `LoopClosure` 新增 **`mode="vlad"`**；`load_vlad_centres()`；CLI `--loop-closure-mode vlad --vlad-centres` | top-3 与直接计算 12/12 一致；增量 `add_timestamp` 正常；自查询 top-1 是自己 sim=1.0 |
| `tinynav/core/map_node.py` | 同上接线（两个 `LoopClosure` 实例） | ⚠️ 运行时词典叫 `frozen_vlad_centres`，与已有的评测钩子 `vlad_centres` 区分 |
| `tinynav/core/build_map_node.py` | `IntKeyShelf` 后端自适应（板上 Python 只有 `dbm.dumb`） | 新旧地图都能开 |
| `tinynav/core/build_map_node.py` | `LOOP_CLOSURE_DEFAULTS`：阈值和 top_k 按后端分别取值 | 见「阈值标定」 |
| `tool/benchmark/retrieval_sweep.py` | 加 `--descriptor-npz`（注入外部描述子）、`--map-a-stride`（地图降采样） | 量化和密度实验用 |
| **`tool/x5_board/hbdnn.py`**（新） | hbDNN 运行时的 ctypes 封装，结构体照 `hb_dnn.h` / `hb_sys.h` 对齐 | 板上真机加载 `.bin` 并推理成功 |
| **`tool/x5_board/sp_post.py`**（新） | SuperPoint 后处理的纯 numpy 实现 | 与原模型逐位一致；板上 78.8 ms vs ONNX 子图 190.8 ms |

三个 `LoopClosure` 实例（建图回环、导航回环、重定位）共用一个 `mode` 参数，一处改动三处受益。

板上跑法（两个模块都只依赖 numpy）：

```bash
LD_LIBRARY_PATH=/usr/hobot/lib python3 -c "
import hbdnn, sp_post, numpy as np
m = hbdnn.BPUModel('sp_backbone.bin')
lg, dm = [o[0] for o in m.infer(img.astype(np.float32) / 255.0)]
kpts, scores, descs = sp_post.postprocess(lg, dm, 5e-4, 512, 'fast')
"
```

## 建图成本（1120 关键帧 → 23 cm 间距后约 280 帧）

`build_map_node.py` 自带 `SequentialReader` + `BagPlayer` 且 `play_rate` 可调，算力跟不上就放慢。

| 步骤 | 1120 帧 | 280 帧 |
|---|---|---|
| SuperPoint | 2.7 min | **0.7 min** |
| VLAD 词典训练 | **0**（固化） | 0 |
| VLAD 编码 | 2.5 min | **0.6 min** |
| 回环检测（若打开） | 2.5 min | **9 s** |
| 位姿图 / 占据栅格 / 深度 | ~10 min **[估]** | ~10 min |
| **合计** | ~18 min | **~12 min** |

## vlad 模式的接线（2026-08-20 完成）

在此之前 **`hbdnn.py` / `sp_post.py` 全仓库没有任何地方 import** —— BPU 那条路只在一条手写命令里跑通过，
而 `--loop-closure-mode vlad` 会掉进 `else` 分支去加载板上不存在的 TensorRT。现已接通：

| 改动 | 位置 / 说明 |
|---|---|
| 两个模块从 `tool/x5_board/` 移到 **`tinynav/core/`** | 它们已经是运行时代码，不是诊断工具 |
| **`SuperPointBPU`** 类，输出与 `SuperPointTRT` / `SuperPointORT` 同构 | `models_trt.py` |
| **`make_sp_extractor()`** 后端选择 `bpu` / `trt` / `ort`，默认 auto | `models_trt.py`，可用 `TINYNAV_SP_BACKEND` 覆盖 |
| **vlad 分支**：`SuperPointBPU` + `SuperPointMatcher` + `DummyEmbeddingEngine` | `build_map_node.py` / `map_node.py` 两处 |
| `sp_backbone.bin`（1.6 M）+ `vlad_centres_k256.npy`（256 K）进 `tinynav/models/` | 词典成为 vlad 模式默认值，不必每次传 `--vlad-centres` |
| 部署脚本排除规则 `tinynav/models` → **`tinynav/models/*.onnx`** | 否则新加的两个小文件传不到板上 |

🔑 **auto 后端的判据是「`sp_backbone.bin` 和 `/usr/lib/libdnn.so` 都在」** —— 板上选 BPU、PC 上选 TRT，
所以**离线建图在 PC 照常跑**。⚠️ 显式 `TINYNAV_SP_BACKEND=bpu` 而环境不满足时**抛异常而不是降级**：
静默退回 3.3 s 的 CPU 路径看起来像卡死，不像报错。

⚠️ **输入尺寸定死 640×544**（编进 `.bin`，改要重编，见 `x5_work/bpu/README.md`）。这正好是 Looper infra
的原生尺寸，部署路径上不缩放、坐标 scale=1；喂别的尺寸会自动 resize 并换算坐标。

⚠️ 故意**没加** `alru_cache_numpy`（`SuperPointTRT` 有）：无 swap 的板子上缓存 32 帧结果代价大于收益。

PC 已验证（docker `uniflexai/tinynav:latest` + 假 BPU 打桩）：输出契约与 TRT 同构、描述子 L2 归一、
非原生分辨率坐标换算正确、彩色自动转灰度、`SuperPointMatcher` 能直接吃 BPU 输出、
板上条件下 auto 正确选中 BPU 且不触碰 TensorRT。**真机端到端仍是 C1。**


## 剩余工作

| 优先级 | 事情 | 说明 |
|---|---|---|
| ~~P1~~ | ~~目标设备内存实测~~ | ✅ **2026-08-20 完成**，并把 `MemTotal` 1307 → 1787 MiB（`ion_cma` 512→32 MiB，已持久化） |
| ~~P2~~ | ~~回环阈值标定~~ | ✅ **不需要做了** —— 阈值 2026-08-17 已在 K=256 上标完并写进 `LOOP_CLOSURE_DEFAULTS`（vlad: 0.35 / 5 / 0.10 / 3）。当初说「必须重标」的唯一前提是「若因内存降 K」，而内存实测证明 K=256 装得下，**不降 K 就不用重标** |
| ~~A1~~ | ~~`SuperPointBPU` 类不存在~~ ✅ **2026-08-20 完成** |
| ~~A2~~ | ~~提取器选择没有 vlad 分支~~ ✅ **2026-08-20 完成** |
| ~~A3~~ | ~~`SuperPointMatcher` 从没被选中~~ ✅ **2026-08-20 完成** |
| ~~A5~~ | ~~模型和词典不在仓库里~~ ✅ **2026-08-20 完成** |
| 🔴 **C1** | **端到端从来没跑过一次** | 「重定位单次 ~750 ms」是**分项相加**，其中 BPU 排队 ~35 ms 和 PnP ~50 ms 还是 **[估]**。A1–A3 做完必须在板上真跑一次完整重定位 |
| 🔴 **C0** | **板上固件缺 `vio_enabled`** | 2026-08-20 实测：板上 `user_params.json` **没有这个 key**，跑的是 `d457ee0`(08-03) 之后、`54d4d7e`(08-11) 之前的版本 —— **现在关不掉 VIO**。本地 `LooperHub/tros_ws/install/lib/libinsight_full_plugin.so`(md5 `cb732ae8…`) 已编好，推上去 + 重启 `insight_full` 即可。⚠️ 会停相机 |
| 🔴 **C2** | **`hbdnn` 在 ROS 节点里和 `insight_full` 并发未验证** | BPU 并发验证过，但那是独立 ctypes 脚本。在 rclpy 节点里（GIL、每帧 `asyncio.run(extractor.infer())`）没跑过 —— **最可能出意外的地方** |
| ~~B1~~ | ~~固化 BPU 编译配方~~ | ✅ **2026-08-20 完成** —— 全部抢救到 `/home/dm/looper/x5_work/bpu/`（配方 `build.yaml`、产物 `sp_backbone.bin`、150 帧标定集、VLAD 词典 `centres_k256.npy`、11 个脚本、两步日志、重跑步骤）。🔴 原先它们全在 `$CLAUDE_JOB_DIR/tmp/` —— 会随 job 删除清空的临时目录 |
| ~~B2~~ | ~~钉住工具链版本~~ | ✅ **2026-08-20 完成**，写进 `x5_work/bpu/README.md`：板上 hbDNN 1.23.10 / HBRT 3.15.54，固件模型 builder 1.24.3 / HBRT 3.15.55，别用更新的 OpenExplorer |
| ⬇️ P3 | 粗层索引验证 | 大地图分层方案的前提；纯 PC 实验。`coarse_tier.json` 只有大小和耗时，**召回率那半没做**。⬇️ **不在首次部署关键路径上** —— 现有方案覆盖 750 关键帧 ≈ 195 米，办公室在 23 cm 密度下未必超 |
| P4 | 打开建图回环 + 验证地图质量 | ~~依赖 P2~~ → **P2 已无需做，现在只依赖 A1–A3**。离线建图在 PC，做完 A 组就能跑。⚠️ 现有 bag 只有 2.6% 的帧有回环机会，**要真验证得录一段绕圈的** |
| 🟡 **S1** | 地图瘦身 | 🟢 **已不是阻塞** —— 每关键帧 2.95 MB 严格线性，**23 cm / 280 帧 + 不存 `patch_tokens` = 616 M/图，双时段 1.23 G**，标准版 2.3 G 也装得下。见 [`board_bringup.md § 2.10`](board_bringup.md)。剩下的动作只是「建图时不生成 `patch_tokens`」，A4 移除 DINOv2 时顺手带掉 |
| **S2** | 脚轮基线测量 | 改装后测：`WHEEL_CIRC` / `WHEEL_BASE`、`cam_offset` / `ctrl_offset`（前驱后符号可能反）、相机光心高度、footprint。**倒车禁/留等这个结果再定**（2026-08-20 决定保留倒车能力，先测里程计） |
| ⬇️ P5 | 词典固化打包进固件 | ⬇️ **可降级** —— 词典只有 256 KB，进仓库 `tinynav/models/` 或放 `/userdata/x5/` 就够，没必要动固件（见 A5） |
| P6 | 异场景词典验证 | 需要另一个场景的 bag → 🟢 **正好可以用手上这两台标准版 Looper 录**，不占存储（bag 存 PC） |
| ~~A4~~ | ~~移除 DINOv2（原 P7）~~ ✅ **2026-08-20 完成**（vlad 分支用 `DummyEmbeddingEngine`） |

## 板上环境备忘

Ubuntu 22.04.5 aarch64，8 核，**无 swap**；Python 3.10.12，numpy 1.21.5，
cv2 4.6.0 在 `/userdata/tmp/pylibs`，**没有 scipy**，onnxruntime 1.18.0（仅 CPU），**没有 pip**。

⚠️ `/userdata/hobot/opt/hobot/deps/` 下有个 `types.cpython-310-*.so` 会盖掉标准库的 `types`，
放进 `PYTHONPATH` 会导致 `Fatal Python error: init_import_site`。

⚠️ 板上无 RTC，每次上电时钟回到 2025-08-26，开工先跑 `tool/x5_board/sync_board_time.sh`。

⚠️ 在板上分配大数组会触发 OOM killer（实测 786 MB 就被杀，`insight_full` 未受影响）。
写板上基准脚本要设内存上限。

## 2026-08-21 `match` 优化与关键帧密度扫描

### `match` 段的真实构成 —— 不只是描述子匹配

`match` 这个计时段包着四件事:`cv2.BFMatcher` 互检、两个建点数组的 Python 循环、
`cv2.findFundamentalMat` 的 RANSAC、回写索引的 Python 循环。板上实测(同一张图的关键帧配对):

| | 近帧(内点 194/285) | 远帧(内点 13/132) |
|---|---|---|
| BFMatcher | 62.7 ms | 51.0 ms |
| **RANSAC** | **9.4 ms** | **77.5 ms** |
| Python 循环 ×2 | 2.4 ms | 2.4 ms |

🔴 **RANSAC 的耗时随内点率变化 8 倍** —— 迭代次数按 `1/内点率^k` 增长,而调用没有迭代上限。
线上 178 ms/候选的形态接近远帧那一列。
⚠️ **"match 比 38.6 ms 的 BFMatcher 基线慢 4.6 倍"这个说法是错的比较** —— 那个基线只含
BFMatcher,多出来的时间大半在 RANSAC,RANSAC 从来不在基线里。

### 改动 A:matcher 换成一次 GEMM(已合入)

描述子是 L2 归一化的,单位向量上「L2 距离最小」和「内积最大」**数学等价**,所以暴力匹配
就是一次矩阵乘。45 对关键帧(间隔 1/3/20/100/200 帧)**匹配集逐个完全相同**,速度 **1.91×**
(80.4 → 42.1 ms;绝对值随负载在 67–118 ms 波动,倍数才可靠)。顺带去掉 DMatch 强制的
两个 Python 循环。

🔑 **一个反直觉的实测结果 —— 互检不要用 `argmax(axis=0)`:**

| 写法 | 耗时 |
|---|---|
| `matmul q@t.T`(参考) | 10.04 ms |
| `argmax(sim, axis=1)` 按行 | **0.60 ms** |
| `argmax(sim, axis=0)` 按列 | **25.73 ms** |
| `argmax(ascontiguous(sim.T), axis=1)` | 24.15 ms |
| `argmax(sim.copy(order='F'), axis=0)` | 23.60 ms |
| ⭐ **再算一次 `matmul t@q.T` 后按行 argmax** | **10.67 ms** |

在 512×512 的 C 连续矩阵上按列归约比生成它的矩阵乘还贵;转置拷贝带着同样的跨步访问,
省不下来。**重算一遍矩阵乘反而快 2.4 倍且结果逐位相同** —— 调优过的 GEMM 打得过朴素的
跨步归约。

### 改动 B:RANSAC 加 500 次迭代上限(已合入)

远帧那一列 77.5 → **39.9 ms,内点 13 → 13**(一个不少)。
⚠️ `USAC_MAGSAC` 在远帧最好(12.8 ms 且内点 16 > 13),但**近帧慢 3 倍**(9.4 → 27.4 ms),
要端到端评测才能定,**未采用**。

**预期**:每候选 178 → 约 67 ms,三候选 533 → 约 200 ms,整次重定位 **876 → 约 545 ms**。
⚠️ 这是外推,还没有端到端复测。

### 关键帧密度扫描 —— 23 cm 是巧合,但落在最优区

🔴 **先纠正出处**:`results_x2/` 里只有 `full_int8full_s1.json` 和 `_s4.json` ——
当初**只测了 stride 1 和 4 两个点**,「23 cm」就是平均间距 5.8 cm × 4 的副产品,
不是扫出来的最优值。

补齐的扫描(`map_gt` 参考 / `map_night` 查询 1163 帧 / `map_day` 词典 / 共享 transform /
判定阈值 0.5 m,结果在 `x5_work/results_density/`):

| stride | 关键帧 | 间距 | 索引 | R@1 | **R@3** | R@5 | R@10 |
|---|---|---|---|---|---|---|---|
| 1 | 1120 | 5.8 cm | 293.6 MB | **68.6%** | 74.9% | 78.2% | 81.0% |
| 2 | 560 | 11.6 cm | 146.8 MB | 68.0% | 76.0% | 78.7% | 81.0% |
| **4** | **280** | **23.2 cm** | **73.4 MB** | 67.8% | **76.9%** | **79.5%** | 81.6% |
| 8 | 140 | 46.4 cm | 36.7 MB | 64.9% | 74.9% | 78.4% | **82.9%** |
| 16 | 70 | 92.8 cm | 18.4 MB | 50.4% | 61.7% | 65.3% | 69.6% |
| 32 | 35 | 185.6 cm | 9.2 MB | 29.0% | 35.9% | 38.3% | 42.0% |

- **1 → 4 几乎无代价**:R@1 掉 0.8 点,R@3 反涨 2.0 点。23 cm 确实是 R@3/R@5 的峰值。
- 🔑 **真正的分水岭在 stride 8 与 16 之间,不在 4。** stride 8(46 cm)R@3 74.9% 与全密度
  持平、R@10 82.9% 是全场最高,索引再减一半;到 stride 16(93 cm)才崩。
- **R@1 单调降而 R@3/R@5 先升后降**:全密度下 top-K 名额被同一位置的重复邻居占满,变稀之后
  K 个候选覆盖 K 个不同地点。

⚠️ **绝对值不能和上面「INT8 精度」那张表混用** —— 那次用 `--descriptor-npz` 注入了特定的
INT8 描述子,这次直接用 `maps2` 里存的 SuperPoint 特征。**同序列内部自洽,可横向比。**
⚠️ 这是**跨时段**(白天图查夜间)的数字,且只覆盖这一条过道。

**结论:暂不改密度**(用户 2026-08-21 决定)。真要动的时候 stride 8 比 stride 4 更值,
代价是 R@3 掉 2 点,换索引 / 加载时间 / 地图体积再各减一半。

## 2026-08-20 SP+VLAD 板上首次端到端实测

同一天先修掉了一个把所有实验都拖垮的根因,见 `board_bringup.md § 7.5`(外来 DDS 发现报文 →
1.5 GB 分配 → 全局 OOM;修法是固件和 tinynav 两处都设 `ROS_LOCALHOST_ONLY=1`)。

### 建图(`bag_2026_08_11_13_51_48`,90.4 s → 345 关键帧,墙上 368 s)

| 阶段 | 次数 | 均值 | 占比 |
|---|---|---|---|
| `mapping_loop` | 345 | 218.9 ms | 44.2% |
| **`feature_extractor`(SuperPoint@BPU)** | 345 | **175.2 ms**(123–242) | 35.4% |
| **`occupancy_raycast`** | **1** | **19167 ms** | 11.2% |
| `tf_publish` | 44 | 171.9 ms | 4.4% |
| `msg_decode` | 345 | 8.8 ms | 1.8% |
| `save_image_and_depth` | 345 | 8.0 ms | 1.6% |
| `get_embeddings` | 345 | 4.4 ms | 0.9% |

🟢 SuperPoint 在真实建图里 **175.2 ms**,比单独微基准的 201 ms **快 13%**。
🟢 `get_embeddings` 4.4 ms 是 `DummyEmbeddingEngine` 返回零 —— **VLAD 在建图阶段不算**,
描述子在加载地图时从 `features` 现算,所以 vlad 模式的建图成本就等于 SuperPoint。
🔴 `occupancy_raycast` 一次 19.2 秒是固定尾部开销,和关键帧数无关的部分很大。

**`--play-rate` 的实际行为**:实测播放 **0.246×**(比实时慢 4.07 倍),低于 0.3 也低于 1.0,
所以**两个值一次都没触发过 sleep,行为完全等价**。它只在消费端快过设定值时才起作用。
⚠️ 它保护的是 `looper_bridge_node`(另一个进程,跨 DDS,**无反压**),不是 `build_map_node`
(同进程,`spin_once` 阻塞,天然逐帧等待)。这次 bridge 只用 34% CPU,没丢帧。

### 与 BoW/ORB 的对照 —— 同 bag、同路线、同位姿源

`map_n1_wheel`(08-11 建,bow)vs 新图(08-20 建,vlad),轨迹范围逐位相同:

| | BoW/ORB | SP+VLAD |
|---|---|---|
| 关键帧 | 342 | 345 |
| `features.dat` | **37.6 MB**(110 KB/帧) | **164.6 MB**(477 KB/帧) |
| 地图总计 | 494.8 MB | 625.8 MB |
| 外部词典 | `voc_office_k10L5.dbow3` **53 MB** | `vlad_centres_k256.npy` **256 KB** |

差别全在描述子:ORB 32 字节 uint8 vs SuperPoint 256×float32 = 1024 字节,单个差 32 倍。

🔴 **"vlad 省空间"要说清对谁省**:对 `map_day` 的 DINOv2 embedding 方案省 39%(省掉
`patch_tokens`);**对 BoW 反而大 26%**。SP+VLAD 的真实优势是①运行时内存 —— DBoW3 的
k10L5 词典加载要 407 MB,VLAD 只要 256 KB 中心 + 262 KB×关键帧数的索引(`map_node`
实测 394 MB);②召回率 R@3 68.4% → 70.0%。

### 重定位(752 次,948.9 s)

| | |
|---|---|
| 成功率 | **728/752 = 96.8%**(`few_landmarks` 13 + `pnp_inliers` 11) |
| 尝试频率 | **0.79 Hz**,每关键帧一次,`skipped_rate_limit=0` |
| 单次 p50 | **876 ms**(min 540,max 1096),对 5 s 预算 |
| `dropped_stale` | 7/760 = 0.9% |
| 质量 | inlier p50 56.1%,landmarks p50 208,候选 sim p50 0.386 |

阶段 p50:**`match` 533 ms(61%)** > `feature_extract` 186 ms > `candidate_search` 61 ms
> `pnp` 31 > `db_load` 20 > `depth3d` 15 > `embedding` 3.1。

🔴 **`match` 是唯一值得优化的地方**:3 个候选各约 178 ms,而 `cv2.BFMatcher(NORM_L2,
crossCheck)` 对 512×256 描述子实测只要 **38.6 ms**,慢 4.6 倍。怀疑 `OPENBLAS_NUM_THREADS=2`
加上满负载争抢。砍下来能把 876 ms 压到 500 ms 以下。

**关键帧频率不是算力决定的**:间隔实测全部 ≥ 1040 ms、p50 1207 ms,是 bridge 的
`--keyframe-static-interval 1.0` 在兜底(车基本静止)。真跑起来关键帧最高到 5 Hz,
那时 876 ms 才会成为瓶颈(上限约 1.14 Hz)。

⚠️ **web 的 "last ok" 会涨到 10 s 再跳回,是显示假象**:`map_node._RELOC_STATS_INTERVAL_S
= 10.0`,`lastSuccessEpoch` 只在发布时刷新,后端每次 `/device/status` 用当前墙钟重新计龄
→ 锯齿 1→11 s。真实频率看 `window.successHz`。同一时刻实测:窗口内 10.7 s 成功 9 次,
而该字段显示 8.0 s。

### 🔴 CPU 已经饱和,导航还没真开始

⚠️ **`ps -o pcpu` 是进程自启动以来的平均值,不能用来量瞬时占用** —— 我用它得出过
"5.1 核 / 8,还有余量"的错误结论。要用 `/proc/stat` 差分。工具:`tool/x5_board/` 外的
`/userdata/x5/cpu_bpu.py`。

12 秒窗口实测(重定位在跑,导航未启动):

| 进程 | CPU |
|---|---|
| **`insight_full`** | **276.2%** |
| `map_node` | 151.0% |
| `looper_bridge` | 117.7% |
| `uvicorn` | 74.5% |
| `planning_node` | 67.8% |
| `cmd_vel_control` | 40.1% |
| `diffcar_control` | 39.8% |
| **合计** | **767.2% = 7.67 / 8 核** |

整机 CPU **85.9%**(单核 74–92%),**iowait 0.0%**,load 16 就是 16 个线程抢 8 个核。
BPU ratio p50 **43%**、mean 48%、**max 98%**。温度 CPU 93.8 / DDR 92.8 / BPU 92.0 °C,
**8 个核一个都没降频,dmesg 零热告警** —— 贴着上限但没被限速。

🔑 **只剩 0.33 个核,而 `map_node` 的 A* 路径搜索一次都还没跑过**(全程
`result=skip_no_poi`,16.7 ms)。导航一开,`planning_node` 要按周期评轨迹库、
`cmd_vel_control` 要跟踪轨迹、`map_node` 要跑 `search_within_sdf_map_numba`
(记录过 14.3 s 的离群),这些都要从这 0.33 个核里出。可预期的失效链是:
重定位延迟 > 关键帧间隔 → `dropped_stale` 涨 → 重定位停摆(这个失效模式已经发生过一次),
以及轨迹过期 → 车中途停(见 `nav_field_results.md`)。

**最大的一块是 `insight_full` 的 276%,其中 VIO 线程占 98.8%(静止;运动时 2.11 核)。**
方案 c 用轮速里程计,**没有任何东西订阅 VIO 话题**,所以它是纯浪费。
🔴 **但板上现在关不掉**:`user_params.json` 没有 `vio_enabled` key,板上
`libinsight_full_plugin.so`(md5 `fb0ac60a09fbbf1dc69f89d8c2e91ef7`)里也 grep 不到这个
字符串。带开关的本地构建产物在
`/home/dm/looper/LooperHub/tros_ws/install/lib/libinsight_full_plugin.so`
(md5 `cb732ae809c416d232b088de1ce5f400`,含 `vio_enabled`),**部署它是导航前的第一优先级**。

### 待查(2026-08-21)

🔴 **20:24 整机重启一次,原因未确认。** 用户当时移动过板子,**接触/供电是首要怀疑**。
反证热的证据:`dmesg` 零 thermal/throttle 告警、重启前 8 核全部满频未降频。
⚠️ journal 不跨重启(`--list-boots` 只有一条),**看不到重启前的日志** —— 要查得先开
persistent journal(`/etc/systemd/journald.conf` 的 `Storage=persistent`),否则下次重启
同样什么都留不下。


## 2026-08-26 建图时间的真实构成 —— 瓶颈不是算力，是单线程搬图

对照包 `bag_2026_08_24_17_35_41`（255.5 s，3.8 GB，173469 条消息），板上跑，app 停掉。

### 🔴 先纠正三处认知

**1. 阶段计时表的百分比是错的，`Grand total` 也不是墙上时间。**
`mapping_loop` 是父计时段，`feature_extractor` / `msg_decode` / `save_image_and_depth` /
`get_embeddings` / `loop_and_pose_graph_update` 全部**嵌套在它里面**，而 `log_summary` 把所有段
直接相加算百分比。837 关键帧那次打出 `Grand total: 402 s`，实际墙上是 **942 s**，402 既不是
总和也不是任何真实的量 —— 每关键帧的时间被数了两遍。

**2. 「app 和建图不能同时跑，会 OOM」是过期结论。** `map_build_ab.py` 的 docstring 和
`app_running()` 拦截都基于它，三条依据全部作废：

| 当时写的 | 现在 |
|---|---|
| 板上 1338 MB | ion_cma 改过后是 **1787 MB** |
| 词典峰值 407 MB | 那是 **DBoW3** 词典；08-25 起用 VLAD，质心文件 **256 KB** |
| app 占 950 MB | 那是**导航时**。点建图时 `_start_rosbag_build_map` 起的是 bridge + build_map_node，导航节点不在跑 |

实测建图峰值 **527 MB**（build_map_node 351 + bridge 176），全程可用内存没低于 959 MB。
**在 app 里点建图是正确且受支持的路径。**

**3. 建图既不吃内存也不吃 CPU 总量。**88.8% 进度时实测：

```
内存 796/1787 MB 用，可用 959 MB      load 3.99 / 8 核      BPU 25%      温度 88.6 C
build_map_node  RSS 351 MB  CPU 100%   <- 正好一个核，Python 单线程
looper_bridge   RSS 176 MB  CPU  33%
insight_full    RSS 181 MB  CPU 155%
```

**瓶颈是 `build_map_node` 单线程占满一个核，另外七个核闲着。**播 bag、发消息、收关键帧、
跑 SuperPoint、写盘全在同一个线程里串行。所以"往 BPU 搬"没有意义：SuperPoint 那 179 ms 里
只有约 39 ms 真在 BPU（其余是 numpy 后处理），而 BPU 才用了 25%。

### 关键帧密度对照（同包，同代码）

三条门限是**或**的关系（`should_add_keyframe`），只放宽位移没用：1 度只占 79.2 度水平视场的
1.3%，一转弯就是每度一帧；「静止 1 秒也发」那条是保底下限，255 s 的包光它就保底 255 帧。
10 cm 在 3 m 景深处等于 1.91 度，所以配 3 度；静止那条放到 5 s。

| | 3 cm / 1 度 / 1 s | **10 cm / 3 度 / 5 s** | |
|---|---|---|---|
| 关键帧 | 837 | **442** | **-47%** |
| 墙上时间 | 942 s | **865 s** | **-8%** |
| 地图体积 | 1837 MB | **971 MB** | **-47%** |
| `depths.dat` | 1.1 GB | 588 MB | |
| `features.dat` | 423 MB | 223 MB | |
| `vlad_index.npz` | 210 MB | 111 MB | |
| 每关键帧 | 220.9 ms | 222.4 ms | 不变 |
| `occupancy_raycast` | 31.4 s | 22.3 s | |

而且新的那次还**多做了 `vlad_index_build` 12.1 s**（旧的那次索引是 22 分钟后另外补存的，
阶段表里根本没有这一行），多做了活还快 77 秒。

🔴 **关键帧砍一半，时间只降 8%。**新的那次所有计时段加起来（去掉嵌套重复）只有 143 s，
**83.5% 的时间不在任何计时段里** —— 全在 bag 播放和跨进程传图上。降密度买到的不是速度，
是**地图体积和加载内存**：2.06 GB 的地图配 1.79 GB 的内存，范围再大就危险，这才是大范围
建图必须降密度的理由。

参数经 `TINYNAV_KEYFRAME_TRANSLATION` / `_ROTATION_DEG` / `_STATIC_INTERVAL` 传入
（commit `2868074`），空值＝用 bridge 默认。**只影响建图**：导航路径上
`--keyframe-min-interval 1.5 s` 比这三条都严，把它们全掩盖了。

### bag 的话题构成 —— 彩色是最贵的一路

| 话题 | 帧数 | 频率 |
|---|---|---|
| `/camera/camera/imu` | 101698 | 398.1 Hz |
| `/camera/camera/vio_100hz` | 24237 | 94.9 Hz |
| **`color/image_rect_raw/compressed`** | **7671** | **30.0 Hz** |
| `color/camera_info` | 7670 | 30.0 Hz |
| `infra1/image_rect_raw` | 5111 | 20.0 Hz |
| `depth/image_rect_raw` | 1278 | 5.0 Hz |

跳过 imu 和 vio_100hz 之后还剩 47534 条，**彩色两路占 32%，而且它是频率最高的图像话题**。

### 已做的三项优化

| commit | 改动 | 依据 |
|---|---|---|
| `2868074` | 三条关键帧门限可配 | 上表 |
| `d8d9dc7` | TF 广播放进 `--no-visualization` 门 | `publish_all_transforms` 每次为**至今所有**位姿造 TransformStamped 再整批广播，837 帧那次单次均值 317 ms / 峰值 1411 ms，442 帧那次 196/805 —— 和已被这个门关掉的 `pose_graph_trajectory_publish` 是同一个二次方开销，当时**漏了这个调用点**。建图自己不消费它：`/tf` 订阅只为算一次 `T_rgb_to_infra1` 就退订 |
| `afe6d75` | `--no-rgb-video` 时整条彩色链路跳过 | 之前只关了**写**（h264 编码 95.3 s），**读没关**：7671 帧照样播出来、逐帧 JPEG 解码、以约 1 MB 原始图重发、占四路同步的第 4 位、每关键帧再 cv_bridge 转 bgr8，最后因 `rgb_video_db is None` 全丢。证据是两次建图的 `rgb_images_db mode=read write_count=0` |

⚠️ 第三项的风险点是同步从四路改三路 —— 配错会**零关键帧且完全不报错**（删 `keyframe_depth`
那次就是这样，12 分钟白跑）。判据：`grep -c "Synced TinyNavDB" ` 在开跑 2 分钟内应该 >= 1。

### 还没做的，按性价比

| # | 改动 | 预计 | 工作量 | 风险 |
|---|---|---|---|---|
| 1 | BPU 推理与 CPU 后处理流水化 | 每帧约 39 ms（22%） | 中 | 低 |
| 2 | **离线建图去掉 bridge 一跳** | 那 83.5% 的大部分 | 大 | 高 |
| 3 | `top_k` 512 -> 256 | 描述子采样与地图体积各减半 | 极小 | 中 |
| 4 | 在 PC 上建图 | 数量级 | 中 | **低可信**：板上重定位用 BPU 的 INT8 描述子，PC 上没有 BPU。INT8 vs fp32 的检索差异在 k-means 自身噪声内（见本文「INT8 精度」一节），所以大概率可行，但要先跑一次 PC 建图 + 板上重定位的交叉验证 |

第 2 项是"两个进程之间跑一个来回"：`build_map_node` 播 bag 发原始话题 -> DDS -> bridge 做
时间戳对齐和关键帧筛选 -> DDS -> `build_map_node` 的 `keyframe_callback`。离线时 bag 本来就在
它自己手里，这两件事不需要另起进程。

### app 录制没有磁盘检查

旧的 `tool/x5_board/map_record.sh` 有 `MIN_FREE_MB=4096`，**app 的录制路径一处空间判断都没有**
（`node_manager.py` / `routers/files.py` grep 命中 0）。实测码率 **14.9 MB/s**，录满会静默毁掉
整个包，且在机器人上看不出来。开跑前自己算：剩余 GB x 1024 / 14.9 = 可录秒数。
