# T10-A19-R08 limited synthetic validation admission and comparison

登记时间：2026-09-10（实际 UTC 时间见 evidence pre_registration/scope.json）。状态：`PREREGISTERED_BEFORE_R08_IMPLEMENTATION_GENERATION_OR_TRUTH_READ`。

## 范围、角色与历史边界

接受 R07 的限定 development 工程结果并永久结束 seed10101 求解器支线。本轮只开放绑定旧 split 的 opt-in synthetic `validation` 入口和固定 12 输入矩阵；不迁移默认路径、不开放 test、不锁 gate、不升级 C1--C3。A08 数值支持域限制、R05 truth 暴露及所有失败/旧 ticket 原样保留。R07 development schema 保持可读且不得与 validation 交叉消费。

新 validation envelope 固定为 `uifgo_t10_validation_admission_v1`，必须绑定 admission 文档 hash、split hash、reservation/base/ancestry/seed/scenario、raw cache/content、原始科学配置、runner/core/GTSAM/MPFR/GMP identity、预算与 metric implementation。truth 路径不得进入 estimator CLI、配置、context 或 cache。validation context 必须进入真实 Stage1 producer、Stage2、score/final identity；跨 role、错 parent/ancestry、test 和缺字段在启动前拒绝。

## 固定输入与顺序

仅使用旧提案的：

1. `a10_val_turn_01_seed20101`: los, step1, step2, step3, ramp_gentle, ramp_steep；
2. `a10_val_turn_02_seed20102`: los, step1, step2, step3, ramp_gentle, ramp_steep。

生成器沿用 split 中精确 motion、共同传感器/噪声和六个固定场景定义；不读取 seed10101 结果重设幅值。seed10101/10102 永久 development。test30101--30103 不生成、不读、不运行。两条 base 只算两个独立 validation 单位。

## 方法、工作点与正常空结果

固定 P1 `(lambda1,lambdaTV,b_min,change,merge)=(4,20,.10,.15,.10)`、`PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1`、R07 certified 333-bit 数值策略、原容差/四项 AND、Stage1 outer<=500、Stage2 outer<=200、每 conditional<=50。五策略固定 suppress_all、structured_debias、fit_only、s_fit、full_gate；`eta>=.10,s<=.10m,gamma<=1,epsilon_bad=.20m`。不扫 P2/P3、阈值、精度或求解器。

零候选必须走严格无 C certified Stage2/final 且禁止 handoff；有候选但零 eligible 必须输出完整 score 原因并执行全部 Suppress 参考 final。每个 LOS 另独立执行同配置/初始化/数值策略的 `all_range` final，不能以 suppress_all 改名代替。score not-applicable 与 numerical failure 分开。

## 工程门

同一生产 runner 的实际小 fixture 覆盖 LOS 零候选、候选但零 eligible、多组 mixed Use/Suppress、无 C recovery/fallback；逐场景 evaluator 核对 step/ramp obs_id/truth。正向 development compatibility 和 validation producer/consumer/final 握手必须通过；负向拒绝跨 role、错 parent/ancestry、test、truth CLI/context、no-C handoff 和错误动态库身份。验证父目录/fresh 叶目录/CMake ABI/日志状态/timeout/exit/进程树。随后两个完整 raw 只 prepare 不优化。保存实际源码快照与 hash。

## 预算、失败与运行

`B_total=14400s`，scheduled 累计 wall 硬限 `10800s`，保留 `3600s` 不自动消费，并行度1。生成/工程/prepare/estimator/evaluator均计 scheduled wall。单输入自动阶段900s；每 final 至多900s、fallback至多一次、单输入科学上限5400s。固定顺序，spawn 前 durable ledger 预留；remaining 不足不启动并记 `NOT_RUN_RESOURCE`。无自动 retry、换 seed 或失败退款加试。普通数值失败作为该输入数据点后继续独立输入；truth泄漏、错误truth关联、身份或共享 correctness 缺陷停止受影响批次。

R07 单输入 83.586104785s 给出 12 输入点估计约1003.03325742s，但场景/运动可能改变收敛，不能保证完成。

## 冻结、评价与判据

所有12输入的 score/decision/final、源码、配置、runner/core/library 先冻结，然后独立 evaluator 才读取对应 scenario truth。逐输入报告候选/段/short/boundary/group/eligible/unavailable、eta/s/gamma和 gate pass、全记录与[3,6]同时间 raw-frame ATE RMSE/P95/matches、decision/final bias risk、bad acceptance/good rejection、candidate/eligible/overall coverage、failure/fallback和成本。零接受 risk=`UNDEFINED`，失败=`UNAVAILABLE`，零候选不算 gate 成功。

LOS 相对独立 all_range 报实际 RMSE/P95 差值；新增失败或差值超过 `.05m/.10m` 才违反已登记容忍。统计只按两条 base 呈现，不把场景/packet 当独立重复。full_gate 与 s_fit 只有实际不同且都有有效配对时才比较；否则写“本矩阵未辨别”。取得完整表或触及停止边界即停止，不运行 held-out test，不回 seed10101，不自动追加弱几何/阈值搜索。
