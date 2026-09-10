# T10-A19-R02 完整自动评分与有限 Use/Suppress 比较协议

时间：2026-09-10。授权来源：本轮用户指挥消息。前置证据：
[`T10-A19-R01 VERIFICATION`](evidence/t10_a19_r01_stage2_integration_20260910T024847Z/VERIFICATION.md)。

本轮是 development 验证，不是正式 validation/test，不锁正式 gate，不升级 C1--C3。A19/A19-R01
历史失败、ticket、身份和结果只读保留。允许在全部 estimator 决策冻结后由独立 evaluator 读取 synthetic
evaluation-only truth；truth/GT 禁止进入 estimator CLI、配置、初始化、partition、score、cache 或 final
消费上下文。

## 1. 固定输入、身份与启动门

- 自动流程唯一输入为 A10 frozen P1 step seed10101 完整 raw 和原配置，从
  `ORIGINAL_RAW_NO_CHECKPOINT` 初始化。方法固定为
  `PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1`、`PAPER_CERTIFIED_PAIR_REDUCTION_V1`、333 bit；原 lambda、容差、
  L1/TV、非负、分段和分组规则不变。
- 复用 R01 中当前 hash 仍匹配的 core、Stage1/Stage2/scoring 实现和工程证据；launcher/preflight/后续
  development final 接线采用新的 R02 identity。所有新输出默认关闭、development-only、diagnostic-only、
  `consumable=false`，旧正式 reader 继续拒绝。
- 新 ticket 前必须在独立临时目录实际验证：父目录创建/写入、fresh leaf 原子创建、重复 leaf 拒绝且内容
  不变、输入/config/runner/动态库 identity、日志与原子 JSON 状态、timeout exit 和整个进程树终止记账。
  预检不得调用 estimator；仅 py_compile 不计通过。
- 自动流程基础设施失败只有日志明确证明未进入初始化/优化时可在本轮修复并重新预检，最多两次；每次新
  attempt identity、新 ticket、新输出，旧失败不覆盖。任何无法确认阶段的失败不得据此重试。

## 2. 完整自动流程预算与停止条件

复用 `AutomaticSupportProvider -> SegmentRefitter -> ScoreRefitRecoverability`。Stage1 outer `<=500`，每个
conditional block `<=50`，Stage2 outer `<=200`，新 estimator 进程树硬超时 `900 s`。Stage1 成功后立即
原子保存 snapshot/partition/obs/parent/hash；Stage2 成功后保存同一最终 joint graph/Values、live `c_s`、
幅值、short/boundary/KKT 与停止条件；评分保存所有 group/segment、eligibility/unavailable 原因、obs/factor
mask、ordering、F/G/N/R、谱、gamma、linearization 与 producer identity。算法开始后的首次失败立即停止，
不调参数、不加预算、不换精度。

## 3. 预登记的五种 development 处理

只有自动 Stage2 收敛、score 完整且身份闭合后才冻结 decisions 并进入 final。所有处理共用同一 automatic
partition、Stage2 graph/Values 和 decision-time scores：

1. `suppress_all`：全部候选抑制，共同参考。
2. `structured_debias`：按既有定义保留候选并联合估计幅值。
3. `fit_only`：合同 eligibility 且 `gamma <= 1`。
4. `s_fit`：合同 eligibility 且 `s <= 0.10 m`、`gamma <= 1`。
5. `full_gate`：合同 eligibility 且 `eta >= 0.10`、`s <= 0.10 m`、`gamma <= 1`。

bad-correction 标签阈值固定 `epsilon_bad=0.20 m`。不得用 truth 选择阈值，不做 sweep，不因决策或结果
难看改变工作点。每种处理最多一次 final 进程、硬限 `900 s`，含至多一次合同 fallback；五种合计
`<=4500 s`。等价决策/输入/目标/初始化可复用同一结果并标 `SAME_DECISION`。自动流程加 final 的科学计算
总硬预算 `<=5400 s`。共享 correctness 缺陷停止所有受影响 final；单策略局部失败保留该行并继续其余预登记
策略。

final 复用现有 cache/final 路径和 C++/GTSAM estimator，不复制估计器。最终图仍使用 original raw range 和
live `c_s`，不加入 corrected pseudo-range；trajectory、bias、residual、covariance/audit 来自同一 final
graph/Values。decision-time 与 final-time 分开，recovery failure/fallback 均保留。

## 4. 冻结后 evaluation-only truth 指标

只有全部可运行策略的 decisions 与 final outputs 已冻结后，独立 evaluator 才读取 synthetic truth。先核验
GT 点、raw frame、时间关联；分别报告完整记录和预声明历史遮挡区间 `[3,6] s` 的 raw-frame ATE RMSE/P95/
匹配数，以及 Stage2 decision-time 与各 final-time bias-field error、accepted bias RMSE、bad-correction rate、
good-correction rejection、candidate/eligible coverage、overall retained fraction、failure/fallback 和成本。
轨迹差值以 `suppress_all` 为共同基线。零接受时 accepted risk 为 `UNDEFINED`；bias error 不等同 trajectory
harm；全部 gate 同决策时明确写本输入未区分政策。本单输入不支持泛化、统计显著性、eta 增量或 LOS 无退化。

## 5. 交付状态

完整自动流程失败则记录首次失败并停止其科学分支；全部 group 不可评分则报告零 eligible 并停止 final。
若 final/evaluation 成功，本轮到配对结果表即停止，下一步转有限 validation 与 held-out 证据，不继续打磨
seed10101。所有命令、退出码、实际 runtime identity、失败/空接受/unavailable/fallback 和阶段成本进入新的
隔离证据目录 `evidence/t10_a19_r02_auto_score_compare_20260910T041247Z/`。
