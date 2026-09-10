# T10-A19-R03 Stage2 受限 inexact handoff 协议

登记时间：2026-09-10T06:11:42Z。角色：`development`。本协议在实现、签发实验 ticket、运行 estimator
和读取 evaluation-only truth 前登记。R02、R01、A19 与全部用户未提交工作保持只读历史；本轮使用新的
输出目录、schema、policy/implementation identity 和 ticket，绝不覆盖、续跑或重标旧产物。

## 唯一 amendment 与适用范围

本轮新增默认关闭的 `PAPER_STAGE2_INEXACT_HANDOFF_V1`，并与既有
`PAPER_CERTIFIED_PAIR_REDUCTION_V1`、`PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1` 组合绑定身份。它只适用于
Stage2 中存在非空待更新 live `c_s` 的交替 refit conditional-navigation block。Stage1、legacy、空候选、
全部抑制/无 live `c_s` 的 final 路径保持原语义。

唯一新增返回状态为 `INNER_NUMERICAL_STALL_INEXACT`。inner 仍为 `converged=false`，另记录
`handoff_qualified=true`；调用方只有看到该精确状态才可继续原非负 `c_s` 闭式更新。返回状态必须满足：

1. 当前 fixed-`c` block 至少有一次 certificate-accepted update；最后接受更新的原 GTSAM generic
   convergence 为真，且该接受更新的 scaled navigation step 不超过 `1e-6`。
2. 随后的失败精确为原 lambda upper exhaustion；该最后搜索中的每个拒绝试步均有完整有效 333-bit
   证书，`P.lo>0`、`D.hi<=0`，且没有 unsupported、unresolved、nonfinite、linear-solve 或其他失败。
3. 这些拒绝试步的 scaled navigation step 均不超过 `1e-6`，且 `P.hi` 不超过
   `Binary64ObjectiveIncreaseAllowance(J_current,J_current)`。不增加梯度门或其他阈值。
4. graph/range metadata、输入身份、状态和梯度审计有效；返回最后 certificate-accepted `Values`，不得
   返回任一拒绝端点。保存触发原因、generic/gradient/step、P/D/allowance、试步数、最后接受状态 identity
   和 handoff 次数。

任何保护条件失败均保留原失败。最终 Stage2 或 final 的 `CONVERGED` 仍严格要求闭式 `c_s` 更新后的同一
joint graph/Values 同时满足相对目标 `1e-8`、scaled step `1e-6`、KKT `1e-8`、navigation stationarity
`1e-6` 加原 roundoff allowance，并通过目标非增、有限性、可行性和 key 一致性。outer budget 耗尽仍失败。
GTSAM direction/native retract、333-bit 证书、lambda、L1/TV、固定 beta、IMU、初始化、partition、评分、
原始 range 与 live `c_s` 的 final 图均不变；不加入 corrected pseudo-range、人工 prior 或信息量 damping。

## 工程门与身份边界

fresh ticket 前必须通过：封存 outer15 的真实 12-trial guard/bitwise state replay；无接受、generic false、
大接受步、大拒绝步、大 `P`、无效/未决证书、其他 failure、空 `c`、Stage1 的 fail-closed 反例；真实小型
mixed graph 的 handoff→闭式 `c_s` 更新→outer 四项审计；以及真实小型 score→decision→final→独立
evaluation。development adapter 只读新 `consumable=false` schema，正式 cache reader 继续拒绝。另实际检查
fresh leaf、防覆盖、输入/config/runner/动态库身份、日志/原子状态和 timeout/进程树记账。只重跑受影响测试。

## 唯一科学运行与预算

工程门全部通过后签发一个新 ticket，从冻结 A10 P1 step seed10101 的完整 raw 输入和原 config 重新初始化。
不读取旧 checkpoint、oracle support 或 truth。Stage1 outer<=500、每 conditional block<=50、Stage2
outer<=200；自动 Stage1→Stage2→score 的进程树硬限 900 秒。算法开始后的首次失败立即封存并停止依赖
科学分支，无 retry、容差/精度/lambda/预算变化或参数搜索。

Stage2 真正通过最终四项 AND 且评分完整、有有效 eligible score 后，先冻结 decision-time 输入，再执行一次
预登记五策略：`suppress_all`、`structured_debias`、`fit_only(gamma<=1)`、
`s_fit(s<=0.10m,gamma<=1)`、`full_gate(eta>=0.10,s<=0.10m,gamma<=1)`。
每策略一个 final 进程树硬限 900 秒，至多一次合同 fallback；final 合计 4500 秒，连同自动流程科学计算
总硬限 5400 秒。只有完全等价决策/输入/目标/初始化才可标 `SAME_DECISION`。零 eligible 按有效负结果停止
final；单策略局部失败保留该行并继续其他仍有效策略，共享 correctness 缺陷停止受影响分支。

所有策略决定和 final 输出冻结后，独立 evaluator 才可读取 synthetic truth；truth 不进入 estimator CLI、
config、初始化、partition、score 或 final cache。固定 `epsilon_bad=0.20m`，报告全记录及 `[3,6]s` raw-frame
ATE RMSE/P95/matches、相对 suppress_all 差值、decision/final bias-field error、accepted risk、coverage、
failure/fallback 与成本。零接受 risk 为 `UNDEFINED`。本轮是单输入 development 记录，不锁 gate、不作
泛化/显著性/LOS 无退化结论，不升级 C1--C3。

## 停止边界

不修改 lambda、容差、333-bit 精度，不 sweep threshold，不新增场景/held-out，不做高精度全局 retract。
若唯一 fresh 运行仍因 joint stationarity 或其他首个算法错误失败，保存梯度、四项 AND、命令、退出码、身份
和完整失败分母后停止，不再临场修补。
