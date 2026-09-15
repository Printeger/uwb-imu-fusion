# Gate D P0–P2 改进报告

本批代码与开发验证已完成，状态保持 `DEVELOPMENT_ONLY / IMPLEMENTED_UNVERIFIED`，没有启动正式 campaign，也没有获得正式 Gate D PASS。第三轮已有修改与失败证据保持原样；本轮原始文件、阶段二进制和中间失败日志保存在 `results/gate_d_p0_p2/`。

每阶段 smoke 均为 140 次输入、前 100 次输入预热、后 40 次测量、四 worker，每个测量尝试准确关联到 128 个唯一且实际调用内核的候选。三个阶段的数据完整性检查通过；40 ms 延迟合同均失败。机器摘要中的阶段 `status=PASS` 只表示数据完整，延迟判定见 `latency_status`。

| 阶段 | core p50 (ms) | core p95 (ms) | core p99 (ms) | max (ms) | 子进程峰值 RSS (KiB) |
|---|---:|---:|---:|---:|---:|
| P0 | 2165.11 | 3222.24 | 3537.93 | 3539.24 | 659292 |
| P1 | 19952.12 | 30751.19 | 32772.43 | 33173.55 | 719972 |
| P2 | 1645.35 | 1820.91 | 1855.82 | 1868.02 | 626948 |

最终 P2 的 p99 比 P0 低约 47.5%。P1 引入最终 Jacobian 的精确检查与参考求解，显著增加耗时；P2 通过有误差界的提前步长拒绝、冻结窗口共享 solves 和索引回收部分成本。以上是开发观测：不同 affinity 上曾同时运行构建和诊断，未隔离 CPU 频率、共享缓存及内存带宽，不能据此给出正式性能保证。

运行环境为 Release、GCC 9.4.0、Eigen 3.3.7、GTSAM 4.2a5、Intel Core Ultra 5 125H，种子 20260901；数值线程环境均设为 1。smoke affinity 为 `0,1,2,3`，冻结重放各阶段统一使用 `4,5,6,7`。阶段二进制、配置和摘要均保留；工作区基于 `643ba957df541ab7aa3158ef3bb6b086166482c2`，为 dirty 状态。

## 数值正确性与行为变化

替换统一计算 `N_base + J_addᵀJ_add − J_removeᵀJ_remove`，先加入替代块，再删除旧块；同步更新信息右端项、残差能量、logdet 和协方差算子。增删块的冻结版本、白化/原始矩阵维度与有限性、删除目标均在计算前检查。低秩中间失败不再直接淘汰完整替换。

进入完整检查的变化候选使用最终白化 Jacobian 的 SVD 判定 rank/条件数；KEEP_ALL 使用同一冻结基础窗口的精确结果。近 rank/condition 门限按 ADR 的一个数量级范围打标签，步长边界另有标签。取消变化候选沿用基础条件数及 Cholesky 对角比作为通过依据。参考路径使用最终 Jacobian 的 SVD 解及协方差算子，在线不构造完整逆矩阵。

提前步长检查仍使用整个状态增量的欧氏范数门限 0.25。QR 解只有在保守误差界证明超限时才能提前拒绝；三角求解逐列累计范数/残差界，正常方程残差用扩展精度累计。不确定、近门限或不稳定情况进入 SVD 参考路径。未进行的完整检查通过数值路径和跳过原因区分。

三个固定窗口（101、139、140）的 P0→P1 对比中，247 个不同的 `(window, action)` 从“删除中间态非 SPD”变为“最终步长超限”；它们仍然无效。测量段“原先拒绝、修复后有效”为 0，后检通过/PL/选择变化也为 0。独立的“删除中间态奇异、完整替换有效”回归则验证了正向恢复，并与 dense oracle 一致。

全部 140 次输入的 `selected_action_id`、FDE 状态、backend 更新次数、stale 标志与输出时间，在 P0/P1/P2 间没有变化；三个 `states.csv` 的 SHA-256 也完全相同。前 60 次更新全部为 best-effort：51 次 `NO_VALID_CANDIDATE`、9 次 `RISK_BUDGET_INVALID`；后 80 次为 `AMBIGUOUS_UNAVAILABLE`。本轮没有修改这些既有风险门限或 best-effort 策略。

## 诊断、缓存与阶段耗时

既有 v5 CSV 列保持含义；独立诊断附件 v3 记录输入尝试编号/时间、事务/窗口、四部分冻结版本、输出状态时间、backend epoch、pending 时长、原始 IMU 样本数、连续拒绝、边缘化次数及实际冻结组 ID。计时行通过独立行号附件关联到输入尝试；预热按输入尝试划分。候选按尝试及实际写出行序关联，允许 reinit 后 transaction/window ID 重新开始。阶段明确记录 EXECUTED、SKIPPED、EXCEPTION；跳过项不写伪造耗时，候选正常拒绝不作为测量异常。

完整 validator 核对 pending 删除目标是否在实际冻结窗口内：合法 pending 因子可能在 discard 后从未进入提交 ledger。该缺口首次由完整日志校验暴露，修复前文件保留在 `p2_before_frozen_group_schema/`。prepare 异常会保存尝试编号、异常阶段和完整 core 计时，并保持零 backend 更新。

缓存仅存在于单个冻结基础窗口，绑定 window、全部版本与完整块内容；同 ID 不同内容分别处理。基础 solves 在 worker 启动前构建，group/unit/mode/hypothesis/evidence 索引只读共享。没有跨 epoch 的历史灵敏度缓存，没有覆盖剪枝，没有删减风险假设或动作集。所有 worker 结果按 action ID 归并。

以下为测量段阶段 p50；各阶段分位数不能相加重建 core 分位数。P0/P1 的历史灵敏度嵌套在 model_generation；P2 已从该阶段扣除。P0/P1 的最终事务/审计计时合并，P2 拆分为 commit/discard、state_audit 与其余 finalize。

| 阶段 | P0 (ms) | P1 (ms) | P2 (ms) |
|---|---:|---:|---:|
| prepare | 2.45 | 2.50 | 2.53 |
| integrity_window | 224.30 | 233.14 | 218.99 |
| all_in_detector | 5.84 | 5.85 | 5.87 |
| current_sensitivity | 11.32 | 12.02 | 10.78 |
| historical_sensitivity | 4.15 | 4.09 | 3.97 |
| model_generation | 61.95 | 65.59 | 55.53 |
| hypothesis_evidence | 420.95 | 434.57 | 388.59 |
| health_actions | 62.14 | 63.16 | 68.13 |
| hypothesis_audit | 76.51 | 81.54 | 45.43 |
| base_factorization | 1.36 | 175.95 | 165.12 |
| shared_cache | 未单列 | 未单列 | 24.49 |
| candidate_evaluation | 1246.26 | 18841.07 | 592.20 |
| fde_decision | 5.08 | 5.20 | 5.77 |
| candidate_audit | 0.32 | 0.40 | 0.70 |
| discard | 未单列 | 未单列 | 0.00 |
| state_audit | 未单列 | 未单列 | 7.16 |
| finalize_commit_audit | 27.70 | 28.83 | 21.50 |

P2 测量段：generated=5120、coverage_rejected=5080、kernel_evaluated=5120、post_passed=0、pl_evaluated=0、selected=0，基础 solve 缓存命中 135787 次。所有测量候选均走 `QR_CERTIFIED_STEP_REJECTION`，因此参考路径计数为 5120；这里的 slow_path 包括 QR 参考证书，不能解释成 5120 次 SVD。P1 的 5120 次为最终 Jacobian SVD 参考路径。

post-detector 不通过时，bridge margin、故障映射和 PL 显式跳过，数值有效性仍单独保留。由于本次测量段在数值步长门限就全部拒绝，不能将测得的全部收益归因于 PL 跳过。剩余主要耗时是候选内核、hypothesis evidence、窗口构建和基础 SVD。

## 冻结重放

最慢 P0 窗口为输入 139，最慢 P1 为 122，最慢 P2 为 110；101/139/140 还包含典型 SPD 中间态和步长拒绝。快照均通过独立诊断运行导出，不使用导出运行的耗时比较收益。三个阶段的状态文件完全相同，额外峰值窗口按同一输入流重建。文件保存矩阵、全部因子块、protected map、动作全集、状态布局、版本及数值配置；内容精确去重，不按 ID 盲目复用。

冻结格式当前为 v2，保存缓存和提前检查开关，并兼容读取原始 P0 的 v1 快照。编码为 little-endian/IEEE binary64。`candidate_replay` 重放候选数值内核，不重建原始 IMU 流或整个 PL 风险集；后检、PL 和最终选择的等价性另由回归测试验证。

下表为同一冻结窗口 139 的三次重放中位数，包含基础构建、候选执行与归并：

| 阶段 | 单 worker (ms) | 四 worker (ms) |
|---|---:|---:|
| P0 | 10411.50 | 4231.25 |
| P1 | 75487.90 | 28215.10 |
| P2 | 1984.39 | 766.91 |

同窗口 P2 消融单次测量：关闭缓存约 923 ms，关闭提前步长检查约 17310 ms；这两个单次结果仅用于定位成本，不是独立分位数结论。单/四 worker 的有效性与拒绝原因一致。窗口 139 的独立 dense oracle 重放也执行了 128 个候选，判定一致；步长范数最大相对差约 2.49e-12。在线算子、dense oracle、缓存开关、内容/版本变化、历史替换、reinit、边缘化及提交次数另有功能回归。

## 验证与明确缺口

最终 9 个受影响 CTest suite 全部通过；其中完整性 V2 为 29 个 C++ 用例，Gate D 日志工具为 11 个 Python 用例。完整 P2 smoke 和 fixed-lag 日志通过 v5 及诊断附件校验。测试覆盖奇异删除中间态、最终奇异、良好基础/病态候选、误导的 Cholesky 对角比、rank/condition/0.25 步长两侧、原始/白化块非法值、相关 UWB 主协方差替换、IMU/bridge/跨传感器/历史/并集动作，以及状态、protected covariance、covarianceTimes、残差、logdet、DOF、阈值、PL、bridge margin 与最终选择。沿用 1e-9 / 1e-7 / 1e-6 分级容差，没有放宽生产阈值。

成功 FDE/PL 的数值等价性包含独立残差故障夹具；这不等于完整生产风险集已经覆盖成功选择。额外 UWB、IMU、联合故障探测中，实际 IMU/联合窗口均注入并核对 10 个原始样本，能够检测到故障，但未出现正确排除、bridge epoch 或 union exclusion。短探测不足以覆盖实际注入区间的文件也保留，随后补跑了 46/36 输入的真实区间诊断。场景脚本的安全检查 PASS 不用于宣称成功路径覆盖。

另一个开发入口实际执行 205 次 nominal backend 提交，发生 6 次边缘化，随后冻结窗口 model_valid=true；prepare/discard 零更新，之后完整 pipeline 只更新一次。该次 pipeline 仍为 `NO_VALID_CANDIDATE` 的 best-effort 提交。故此轮已验证提交/边缘化机制，但没有证明成功 FDE 后持续提交超过 200 次的路径。

140 次 smoke 的最后一次输入时间为 7 s，输出仍是 3 s 的状态；backend epoch 停在 60，pending=4 s，原始 IMU（含边界）=801，连续拒绝=80，边缘化=0。这说明原 smoke 测量段覆盖的是拒绝积累路径，不能代表正常提交或成功排除的稳态延迟。

P3–P6 的快速条件证书、窗口缓存、PL 深度优化及常驻线程池仍未实施。正式合同保持每轮 12000 输入、100 预热、11900 测量、四 worker、每次 128 候选、三个 repeat 各自 p99≤40 ms。

证据入口：[机器摘要](summary.json)、[P0](p0-performance.json)、[P1](p1-performance.json)、[P2](p2-performance.json)、[快照与 SHA-256](snapshot_inventory.json)、[fixed-lag](fixed-lag-summary.json)、[CTest](ctest-final.log)。重放和独立导出命令见仓库 README。

最终源码重建材料为 `results/gate_d_p0_p2/source_final.patch`、
`source_new_files.tar.gz` 和 `source_manifest.json`；补丁包含开始本轮时已经存在的第三轮改动。
[测试计数与校验结果](verification.json) 由最终 CTest XML 和日志整理。
