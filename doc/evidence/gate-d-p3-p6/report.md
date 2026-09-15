# Gate D P3–P6 开发闭环报告

本轮完成了 P3–P6 核心机制的集成实现和开发级验证，状态为
`DEVELOPMENT_ONLY`。没有执行正式 3×12000 campaign，当前 smoke 的
`core_total p99=1353.91 ms`，未达到 40 ms 合同，因此不得宣称 Gate D
PASS。完整机器数据见 [summary.json](summary.json)，原始日志、冻结快照和
二进制保存在 `results/gate_d_p3_p6/`。

## 已实现

- P3：冻结窗口只构建一次精确 SVD/LLT、基础状态、统计量、logdet、谱界、
  块索引和内容指纹。候选使用基础白化低秩更新、薄 QR 与小型对称特征值界
  证明 SPD/rank/condition，并以矩阵自由残差约束状态误差。只有证书明确证明
  `norm(delta)-error_bound>0.25` 才提前拒绝；不确定或近门限候选回退到最终
  Jacobian 的精确 SVD。仅证书路径不再伪造精确条件数。
- P4：故障证据按唯一 mode 紧凑组装，一次求解全部基础协方差 RHS，再由小块
  交叉项构造联合 hypothesis。冻结 factor block 采用内容和完整版本键缓存，
  新 transaction、替换和 reinit 失效；统计边界采用浮点 bit 精确键缓存。
- P5：候选按 action ID 固定槽分成“数值内核+post”和“bridge+fault map+PL”
  两阶段。PL 仅处理剩余 hypothesis 的唯一 mode，把 protected map、mode normal
  cross 和 bridge Jacobian 合并为一次 `covarianceTimes()`，共享 mode Gram，
  bridge box margin 仍逐项相加。
- P6：在线流水线与 `candidate_replay` 共用常驻线程池。四个 worker 静态轮转、
  各自复用 scratch；异常在全部 worker 到达屏障后回传；析构和 reinit 会安全
  停止或清空状态。单 worker 开关保留，数值库线程固定为 1。
- 冻结候选格式升级到 v3，兼容 v1/v2；旧快照读取时关闭新证书并要求精确条件
  输出。诊断附件升级到 v4，兼容 v1–v3，并新增证书、条件界、精确回退、矩阵
  自由拒绝、协方差求解、scratch 和缓存审计字段。主候选 CSV 对未计算的精确
  条件数保持空值。
- fixed-lag 边界消元改用部分 multifrontal QR；开发入口完成 205 次显式提交并
  观察到 6 次边缘化，随后冻结的窗口有效，prepare/discard 保持零 backend
  更新。

## 数值与并发验证

全量 CTest 的 20 个 suite 全部通过；其中本轮直接复跑并留存独立日志的受影响
测试为 71 个 C++ case 和 21 个 Python case。覆盖内容包括明确证书
通过、最终奇异、病态与 rank/condition/0.25 门限两侧、dense oracle 等价、
相关 UWB 替换、IMU/bridge/跨传感器联合动作、共享 mode 的单次 PL 协方差求解、
缓存内容/版本 miss、线程池异常屏障、析构/reinit、单/四 worker 顺序一致，以及
prepare/discard 零更新与 commit 恰好一次更新。容差仍为 `1e-9 / 1e-7 / 1e-6`，
没有放宽数值或风险门限。

最新 v3 成熟 fixed-lag 快照含 128 个实际动作，单 worker 两次重放为
863.25/833.09 ms，四 worker 为 289.22/285.95 ms。排除 `wall_ms` 与线程相关的
scratch 复用计数后，单/四 worker 的所有输出字段逐项一致。四线程中两次重放
合计有 256 个证书/基础条件路径、零次候选 SVD；第二次调度复用了 scratch。

## 开发性能

P6 smoke 使用 140 次输入、前 100 次预热、后 40 次测量、CPU affinity
`0,1,2,3`、四 worker 和单数值线程；每个测量输入均执行 128 个候选内核。

| 指标 | P2 p50/p95/p99/max (ms) | P6 p50/p95/p99/max (ms) |
|---|---:|---:|
| core_total | 1645.35 / 1820.91 / 1855.82 / 1868.02 | 1292.29 / 1337.11 / 1353.91 / 1360.38 |
| candidate_evaluation | 592.20 / 680.49 / 684.54 / 686.71 | 542.73 / 576.05 / 576.45 / 576.50 |

`core_total p99` 相对 P2 降低约 27.0%，但仍超过 40 ms。P6 子进程峰值 RSS 为
627888 KiB。测量段 generated/kernel_evaluated=5120、coverage_rejected=5080、
certificate_passed=5080、reference_svd=40、scratch_reuse=5080；post_passed、
pl_evaluated、selected 与 covariance_solve_count 均为 0。因而此 smoke 只证明
持续拒绝路径的收益，不能作为成功 FDE/PL 路径性能证据。

当前主要 p50 成本仍为 candidate_evaluation 542.73 ms、integrity_window
165.29 ms、hypothesis_evidence 164.68 ms 和 base_factorization 164.47 ms。

## 未闭合项

以下计划项没有被开发证据覆盖，机器摘要均显式记录为 `false`：

- 真实全流水线中的成功 UWB 排除、成功 IMU generic bridge 和跨传感器联合选择；
- P6 smoke 中 post/PL 成功路径及其一次合并协方差求解的实际性能；
- v3 冻结文件的完整 post/PL 重放附件；
- 独立的 P3、P4、P5 checkpoint 二进制和同条件阶段性能；
- 正式 3×12000 campaign 与独立评审。

成功排除和 PL 等价性目前由 synthetic/dense-oracle 单元测试证明，不等价于真实
生产风险集的全流水线成功证据。205 次提交后的完整 pipeline 状态仍为
`NO_VALID_CANDIDATE`，虽发生一次 backend 更新，也不能用来宣称成功 FDE。

因此本轮的最终判定是：核心实现集成和回归验证完成，性能合同失败，场景覆盖不完整，
状态保持 `DEVELOPMENT_ONLY`。
