# 冻结窗口完整性计算性能优化与验证报告

日期：2026-09-16（Asia/Shanghai）

## 结论

三项优化均已实现，并保留了原有单故障、已启用双故障及全部模式间交叉项。默认 ramp、KEEP_ALL、61,104 假设的主实验中，完整单帧核心 wall time 的合并 p50 从 3155.690 ms 降到 744.144 ms，速度为 4.241 倍，降幅 76.419%；p95 从 4718.360 ms 降到 1176.540 ms，速度为 4.010 倍，降幅 75.065%。40 ms 目标未达到。

优化不是通过减少故障模型、降低诊断频率或增加总线程资源获得：主实验两边每帧都是 61,104 个假设、128 个已生成候选、1 个实际 KEEP_ALL candidate kernel 和 1 次 PL 评估；CPU 亲和性均为 `0,2,4,6`，总 worker 预算为 4，Eigen/BLAS 线程均为 1。

## 实现

### 1. 同一冻结窗口共享证据和 PL 数值结果

- 新增只读 `FrozenHypothesisNumerics`。它以冻结窗口身份、图/排序/噪声/线性化版本、内容 fingerprint、数值契约、假设集合、故障模式映射和有效基为一致性边界。
- KEEP_ALL 证据阶段一次构造连续模式矩阵、协方差乘模式 RHS、完整 mode Gram 和每个假设的小矩阵结果；PL 直接消费同一上下文。
- 只共享已证明等价的底层量。证据与 PL 各自的秩阈值、monitorability 和 slope 语义保持独立，没有为了命中缓存放宽判据。
- 内容 fingerprint 增补原始/白化 Jacobian 与 residual、协方差、whitener、权重、因子 ID/顺序和变量顺序。观测删除、桥接、重白化、边缘化或线性化变化不能误命中。
- 稳态 300 帧中 PL 共享命中 300、失效 0；每个新冻结窗口只在本帧内共享，不做无条件跨帧复用。A→B 消融把每假设 LDLT 从两次降为一次。

### 2. 低维连续批处理和假设层并行

- 模式数据、组合索引和输出改为连续存储；所有模式的 `H^T A` 与 covariance RHS 合并计算，完整 `dense^T*dense - cross^T*covariance_modes` 保留双故障交叉项。
- 1/2/3 维使用固定尺寸 Eigen 分解/求解；非常见维度和临界情况走原通用路径。没有显式求逆或裸行列式。
- 默认满窗口实际维度为：1 维 464、2 维 41,440、3 维 19,200、其他 0，总计 61,104；因此本次主序列全部命中低维路径。独立测试用 4 维模式验证通用回退，并覆盖奇异 2 维模式。
- 使用已有持久 worker pool 做静态分块，每帧 4 个块；假设级与候选级阶段串行复用同一 4 线程预算，不嵌套扩张。输出按原索引写入，最大值归约和并列选择确定。
- 缓存只读，计数在线程局部累积后合并，避免逐假设原子操作和小锁。

### 3. 窗口数值准备

- Jacobian 空间 SVD 默认改用 `BDCSVD`，仍保留 SVD rank/condition 检查、LLT、残差/前向误差验证和 spectral fallback。显式检查 singular values、V 和 solve 的完整性/有限性；不能用“LLT 成功”替代保护性 SVD。
- 将边界 provenance、因子线性化/白化、稠密装配、SVD、正规方程、LLT/状态求解和 fingerprint 分别计时，删除重复装配/复制并合并 RHS。
- 没有生成完整协方差逆矩阵，没有跨帧复用上帧线性化或分解。尚未证明等价的增量/稀疏分解继续保留原路径。

### 受控路径

- A：共享关、低维批处理关、`JacobiSVD`、1 worker。
- B：仅打开证据/PL 共享，1 worker。
- C：共享 + 低维批处理，1 worker。
- D：C + 4 worker 假设分块并行。
- E：D + 窗口准备优化。

这些路径由环境开关组合，不维护五套实现。

## 基线身份和实验条件

基线来自本次修改前的当前 dirty 工作区，而不是旧 HEAD。没有覆盖或清理已有修改。

| 项目 | 值 |
|---|---|
| HEAD | `e6ded3069bbc4cd5cf98f14257e96f61066685b8e` |
| 修改前 45 个工作区文件内容清单 SHA256 | `dad3ae23a90fce5f0cf0dad39c351f281f22da8c73824637dfc4b34bf5305065` |
| 基线 executable SHA256 | `92938d2864a05a50da5c784d422a95ce3442734d7d68d5655947e5c0ffc556e7` |
| 基线 `libuwb_imu_pl.so` SHA256 | `f12314e393b8235838c908cdf5d02f54cd28f37dd3a0606d20db68406dfe90b5` |
| 优化 executable SHA256 | `b5b7b5e3b31e20ceb36f49f602de30b8cb8aad76366ce434d462e05136e2a5bd` |
| 优化 `libuwb_imu_pl.so` SHA256 | `958af2ac4aa5b7588124648e1b1d1f65aa7ce1550fc4bd241dcd12e01dcf0841` |
| 实际加载 GTSAM SHA256 | `00d83aa618194aefc4b2011e1d29bd9aba107a8b5e0ee8e946eb2455351219da` |
| 构建 | Release，`-O3 -DNDEBUG`，GCC 9.4.0 |
| CPU | Intel Core Ultra 5 125H，WSL2，18 logical CPUs |
| 亲和性 | `taskset -c 0,2,4,6`，4 个不同物理 core |
| 线程 | worker budget 4；OMP/OpenBLAS/MKL/Eigen 1 |
| 配置 SHA256 | `7c465463a64299695ec40c580db7a2ff08a0db845697c5f5dfbed60b03ef9171` |
| 场景 manifest SHA256 | `f556003bbe3dfc5c8f3991dc0bf5543cdddc258b4b00f42b776dbe03fcbd37e4` |
| seed | `20260901` |
| 输入/诊断 | `A_default_ramp`；性能运行关闭逐假设 full audit，必要候选/数值诊断保留 |

优化 executable 的 `ldd` 确认从独立目录加载上述优化动态库；基线同理加载冻结的基线库。

## 正确性

### 结果

- 21 帧 baseline/optimized full audit 比较了 480,684 条假设记录。
- 模式/假设集合及顺序、fault rank、monitorable/plausible、检测、候选资格与顺序、选择、提交/拒绝、健康状态和所有 finite/inf/NaN 分类完全一致，离散差异为 0。
- 状态最大绝对误差为 0；HPL/VPL 最大绝对误差为 0；排除计时后候选数值最大绝对误差为 0。
- SVD 派生诊断量在 CSV 输出精度下，最大绝对差为 `condition_number=0.01`。全字段最大相对差为极小 `log_evidence` 上的 `2.59448e-4`（`1.04067e-19` 对 `1.04040e-19`，对应绝对差 `2.7e-23`）；排除绝对差不超过 `1e-12` 的近零量后，最大相对差为 `conditioned_statistic=7.4796e-6`。没有跨阈值或改变 plausible 状态。
- C/UWB、D/IMU、E/联合三种 32 帧故障序列的状态、检测、候选、事务和有限性分类均完全一致。真实场景没有成功选中排除动作：C 在故障帧拒绝、26–32 帧恢复提交 KEEP_ALL；D/E 在 25–32 帧维持 fail-closed 拒绝。没有伪造正例。
- 修改观测集合的正路径由 `GateDSelection.DenseAndOperatorPathsSelectSameSuccessfulExclusion`、`GateDOracle.RealCurrentHistoricalBridgeAndUnionActionsMatchDenseAndWorkers` 和 `IntegrityV2ProtectionLevel.RecomputesPostCandidateAndStaysResearchOnly` 覆盖。
- 226 帧 `H_mature_union` 真实管线完成 25 次边缘化、2 次故障拒绝，v5 run schema 和 v9 diagnostic attachment 均 PASS。`FixedLagMatchesUnboundedBeforeAndAfterMarginalization` 覆盖边缘化前后 fixed-lag/无界数值一致性。
- 相关 C++ 测试 96/96 通过，Python 诊断工具测试 13/13 通过。覆盖冻结内容突变失效、秩/条件阈值两侧、近奇异、不可监测、双故障交叉项、候选观测变化和 worker 确定性。

成熟运行暴露并修正了 v9 校验器继承 v8 规则时漏列版本号的问题；这是诊断验证修复，不改变算法结果或计时数据。

## 主性能实验

运行顺序为 baseline-1、optimized-1、baseline-2、optimized-2、baseline-3、optimized-3。每个进程 140 帧，前 40 帧预热，统计 41–140 共 100 帧；下表合并三轮 300 个样本。分位数使用 nearest-rank：`sorted[ceil(p*n)-1]`。阶段分位数没有相加。

| 完整 `core_total` | baseline ms | optimized ms | 加速 | 降幅 |
|---|---:|---:|---:|---:|
| p50 | 3155.690 | 744.144 | 4.241× | 76.419% |
| p95 | 4718.360 | 1176.540 | 4.010× | 75.065% |
| p99 | 7992.950 | 1668.990 | 4.789× | 79.119% |
| max | 27742.300 | 2389.100 | 11.612× | 91.388% |
| mean | 3517.741 | 803.261 | 4.379× | 77.165% |

第 2 轮基线有 24.3–27.7 s 的宿主尾延迟；没有删样。逐轮 p50 分别为基线 3132.990/3450.340/2926.080 ms，优化 822.504/744.772/686.566 ms。

### 完整进程和 RSS

| 轮次 | baseline elapsed / RSS | optimized elapsed / RSS | elapsed 加速 |
|---|---:|---:|---:|
| 1 | 416.27 s / 371148 KiB | 113.27 s / 286800 KiB | 3.675× |
| 2 | 508.33 s / 370080 KiB | 99.56 s / 288168 KiB | 5.106× |
| 3 | 414.62 s / 372056 KiB | 89.73 s / 288988 KiB | 4.621× |

median RSS 从 371148 KiB 降到 288168 KiB，减少 82980 KiB（22.358%）。每个满窗口共享上下文报告 8,799,264 bytes；它是窗口生命周期内的只读数据，没有造成 RSS 上升。

### 主要阶段

| 阶段 | baseline p50 / p95 ms | optimized p50 / p95 ms | p50 加速 |
|---|---:|---:|---:|
| candidate evaluation + PL | 1135.946 / 1905.739 | 66.751 / 99.477 | 17.018× |
| integrity window | 946.113 / 1578.097 | 147.250 / 261.733 | 6.426× |
| hypothesis evidence | 687.579 / 1016.816 | 199.437 / 331.140 | 3.448× |
| model generation | 179.504 / 286.699 | 174.810 / 262.257 | 1.027× |

最终窗口准备 p50 的细分是：SVD 106.271 ms、fingerprint 15.858 ms、boundary provenance 8.652 ms、线性化/白化 8.467 ms、正规方程 6.026 ms、LLT/状态求解 1.539 ms、dense assembly 0.730 ms。这些是各自分位数，不能相加为窗口 p50。

### 实际数值工作

主实验每个版本的 300 个稳态帧都执行 18,331,200 个假设、300 次 base SVD、300 次 base LLT 和 600 次状态求解。最终版执行 18,331,200 个低维假设、0 次通用回退、1200 个并行块、300 次共享命中、0 次共享失效、300 次 covariance RHS solve（236,100 列）和 0 次 numerical-contract mismatch。基线同样是 300 次 covariance RHS solve（235,200 列）；最终版多出的每帧 3 列是一起求解的受保护坐标，并非重复 solve。

旧基线的 v8 计数器没有区分 fixed-size、generic fallback 和共享命中；因此这些细项使用同一优化二进制的 A–E 路径作可比计数，不对旧二进制虚构数据。

## 消融

同一 40 帧默认 ramp 序列，统计已装满假设窗口的 21–40 帧，共 20 样本。p99 由于样本短仅作观察。

| 路径 | p50 / p95 / max ms | 相邻步骤 p50 收益 |
|---|---:|---:|
| A reference | 2744.530 / 3163.310 / 4021.860 | — |
| B shared | 1842.800 / 2182.870 / 2232.610 | 1.489×，降 32.856% |
| C batch, 1 worker | 1545.280 / 2167.360 / 2713.490 | 1.193×，降 16.145% |
| D hypothesis parallel | 1483.190 / 1674.330 / 1689.630 | 1.042×，降 4.018% |
| E window prep | 659.283 / 872.049 / 1154.840 | 2.250×，降 55.550% |

各增量百分比基于相邻版本，不能相加。A 的 `fault_gram_ldlt=3,283,320`，B 为 `1,641,660`，直接验证共享消除了证据/PL 的第二次每假设 LDLT；C 的 1,641,660 个假设全部进入低维路径；D/E 各执行 160 个假设分块。

## 故障帧性能

| 场景 | 帧/区间 | baseline ms | optimized ms | 加速 / 降幅 |
|---|---|---:|---:|---:|
| C UWB | 故障 input 25 | 1947.930 | 599.208 | 3.251× / 69.239% |
| C UWB | 恢复 input 26–32，mean | 2534.040 | 608.597 | 4.164× / 75.983% |
| D IMU | 故障 input 25 | 2212.020 | 860.240 | 2.571× / 61.111% |
| D IMU | 后续拒绝 26–32，mean | 2426.529 | 935.038 | 2.595× / 61.466% |
| E 联合 | 故障 input 25 | 1824.050 | 532.663 | 3.424× / 70.798% |
| E 联合 | 后续拒绝 26–32，mean | 1854.563 | 670.916 | 2.764× / 63.823% |

正常和故障帧均未接近 40 ms。没有混合正常/故障样本来掩盖尾延迟。

## 剩余瓶颈、未完成项和风险

- 新的最大 p50 阶段是 hypothesis evidence（199.437 ms），其次是 model generation（174.810 ms）和 integrity window（147.250 ms）。窗口内部仍主要受保护性 SVD（106.271 ms）支配。
- model generation 基本未改善，仍为每帧构造 61,104 个假设；下一步应在不改变顺序/覆盖集合的前提下复用不可变组合元数据。
- 没有实现增量/稀疏 SVD，也没有跨帧复用线性化或分解，因为本次未能证明其在边缘化、重白化和秩阈值附近与现有契约等价。
- 主实验到 140 帧，未到 fixed-lag 200 边界；边缘化由独立 226 帧真实运行和 fixed-lag/unbounded 测试验证。没有额外跑一份旧二进制 226 帧性能对照，因此成熟边缘化的 before/after 性能标为未测。
- 实验运行于 WSL2，三轮可见宿主调度尾延迟。报告保留全部样本和逐轮结果；绝对耗时应在目标部署机复测。
- 真实故障场景没有成功排除动作，候选改变观测集合的成功正例是候选级数值/事务测试，不冒充端到端真实场景结果。

原始大日志只保存在 `/tmp/uwb_imu_pl_perf_20260916`，未复制进仓库。仓库内仅保留本报告、[performance.csv](performance.csv)、[summary.json](summary.json) 和 [reproduce.sh](reproduce.sh)。
