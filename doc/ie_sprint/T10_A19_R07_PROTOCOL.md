# T10-A19-R07 development suppress-all certified-policy amendment

登记时间：2026-09-10T10:41:17Z。状态：`PREREGISTERED_BEFORE_R07_CODE_CHANGE_OR_RUN`。

## 身份与用途

- 继续使用精确 payload schema `A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY`；不因 R07 编号更名。
- 新 implementation/run/library 身份必须重新计算并绑定；入口继续 default-off、role=development、
  `consumable=false`。
- P1 step seed10101 的 R05 truth 暴露和 R06 evaluator hash-prefix amendment 永久保留。本轮统一标
  `UNBLINDED_DEVELOPMENT`，不是 validation/test、盲测、正式 gate lock 或 C1--C3 证据。

## 唯一方法扩展

仅将既有 `PAPER_CERTIFIED_PAIR_REDUCTION_V1` 的 333-bit P/D 接受和原 generic AND navigation
stationarity 检查显式接入 development final 中 accepted 为空、全部候选抑制的无 C recovery 与唯一
fallback。旧路径为 `GTSAM_CHECK_ONLY_V1`；新路径必须接收实际 candidate-excluded 共同参考图的全部
remaining raw-range metadata/常数。32 条候选继续排除，不能重分类或进入 all-range 估计。

无 C 分支禁止 `INNER_NUMERICAL_STALL_INEXACT` handoff。lambda exhaustion、未决证书、零 accepted update、
非有限状态或最终 navigation gradient 超阈值继续失败。最终 objective/step/KKT/navigation 四项 AND、
200 outer、每 conditional block 50、lambda、容差、方向、native retract、初始化、IMU 模型、beta、
Stage1、有 C Stage2/其 handoff、score、gate 与工作点均不变。无 development request 的 legacy/default
调用保持原行为。

## 工程门

fresh ticket 前必须实际通过：

1. 非空 candidate partition、accepted 空的真实小图，核对被排除 obs、factor/Values key、全部 remaining
   raw range metadata、callback 次数、333-bit policy；不得用 mock/check-only 冒充。
2. recovery 正常路径和 recovery 受控失败后的 fallback 都使用相同 certified policy，输出目录互不覆盖；
   fallback 至多一次。
3. 无 C handoff、错误身份、未决 certificate、梯度不达标仍失败；default 路径不进入 development callback。
4. 对 R06 封存 Stage2 只做 candidate-excluded graph/Values/identity 静态核对，预期 339 factors、123 X/V/B
   keys、0 C 仅作核对值，不硬编码为通过条件，不读取或冒充参考 checkpoint。
5. 复用仍适用的 R06 production export/score/evaluator、CMake/ABI 与 prepare 证据；所有受本改动影响的
   refit/inference/runner 检查实际运行且测试数非零。

工程门任一 correctness failure 必须先修根因并重跑受影响检查；不签科学 ticket。

## fresh 运行、预算与停止边界

工程门通过后只签发一次 fresh P1 step seed10101 ticket，从冻结完整 raw 与原 config 以
`ORIGINAL_RAW_NO_CHECKPOINT` 初始化。不得读取 R06 trajectory/checkpoint、oracle support 或 truth/GT。

- Stage1 outer<=500、conditional<=50；Stage2 outer<=200；自动阶段硬限900秒。
- 五种 final 各至多一次、各硬限900秒并含至多一次合同 fallback；final 总限4500秒，全科学进程树限5400秒。
- 五策略固定为 suppress_all、structured_debias、fit_only(`gamma<=1`)、s_fit(`s<=0.10m,gamma<=1`)、
  full_gate(`eta>=0.10,s<=0.10m,gamma<=1`)；`epsilon_bad=0.20m`。
- 不调 eta/s/gamma，不 sweep、不加 outer、不放宽 `1e-6`、不改精度或引入新 estimator/先验/抖动。
- 同 decision、mask、目标、初始化及 solver 身份完全等价时可复用共同结果，但必须标 `SAME_DECISION` 并
  记录真实执行次数；R06 历史失败不改标。
- 无 C 同严格 policy 的首次算法失败须封存实际 graph/Values、最后 accepted state、完整 P/D、lambda、
  gradient 和失败行，停止受影响分支；其他预登记且不受共享 correctness 缺陷影响的策略仍保留。

## 评价与判据

所有 decision/final、源码、配置、runner/core/library hash 冻结后，独立 evaluator 才读取 evaluation-only
truth；estimator truth/GT/oracle open 必须为0。报告全记录和 `[3,6]s` raw-frame ATE RMSE/P95/matches及
相对 suppress_all 差值、decision/final bias RMSE、accepted RMSE、bad-correction、good rejection、候选
占比、candidate/eligible use coverage、overall retained fraction、failure/fallback、阶段时间和总成本。

bias-good 不等于 trajectory-beneficial；零接受 risk 为 `UNDEFINED`。三个 gate 同决定时明确本输入没有
eta 增量比较。只有 suppress_all 和相应策略都有有效同时间关联轨迹才回答改善/恶化。取得配对表或触及
首次失败边界即停止；成功后结束此单 seed 工程支线，下一研究任务转预登记有限 validation/held-out。
