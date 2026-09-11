# PL conditional RAIM/FDE 串行实施协议

状态：`DONE / PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM / PRODUCTION_AND_E2E_NOT_RUN`。
授权来源：用户于 2026-09-12 给定的「PL Conditional RAIM/FDE 适配与 locked Walk1
验证计划」。它替代未执行的 Projected GLRT v5 计划，不改写 Windowed FDE v4 负结果。

## 1. 基线、保护与停止门

- IE 基线：`d82d794e79f2d2550d6815ed687dff170af0e593`；PL 实现基线：
  `ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`；recovery backend 基线：
  `cf287b4fec9bc5689317f302ccf4cdd927bfba81`。
- `nlos_refit.*` 和 `nlos_recoverability.*` 相对 recovery 基线必须零差异。fixed UWB
  factor、Stage2 阈值/预算、Rc、local sigma、kappa、LCB、final solver/evaluator 科学语义不变。
- 受保护的未跟踪 `doc/v2/ie_0911/` 不读取、不修改、不暂存。旧结果和 legacy 默认不改。
- 拒绝覆盖的唯一输出根为
  `/home/mint/ws_fusion_uwb/res/pl_conditional_raim_preflight_20260912_01` 和
  `/home/mint/ws_fusion_uwb/res/pl_conditional_raim_e2e_20260912_01`。
- preflight 唯一通过状态是 `PL_CONDITIONAL_PREFLIGHT_PASS`。任意其他状态立即停止；
  provider 注册、production runner/cache 接入和 E2E 均 `NOT_RUN`。

## 2. 锁定 PL 合同

以 PL commit 的 `IncrementalUwbImuEstimator::predictTo/preMeasurementSnapshot/commitUwbBatch`
与 `IntegrityMonitor::evaluateConditional` 为一手定义，详细源码映射见
[`../ie_0911/PL_CONDITIONAL_DETECTOR_CONTRACT.md`](../ie_0911/PL_CONDITIONAL_DETECTOR_CONTRACT.md)。
`predictTo` 只提交 IMU/history，再提取当前 `X,V,B` 的 15×15 Bayes-tree joint marginal `P`。
当前 UWB group 在 detector 完成前不 commit。物理 `ν=z-h`、`H=∂(h-z)/∂x`、
`W=L^{-1}`，内部 `ν_w=Wν,H_w=WH,S_w=H_wPH_w^T+I`；
`T=ν_w^T S_w^{-1}ν_w`，DoF 为 group 测量数，`p_fa=1e-5`，`T<=threshold` 为 PASS。
PL 原实现 alarm 时拒绝整组。anchor hypothesis 只定义 physical incidence；failure slope 属于
PL 计算，不是 IE detector/isolation score，禁止排名。

## 3. 独立 preflight

Preflight 只新增 source-neutral conditional core、shadow replay diagnostic、独立 truth evaluator、
reference fixture 和测试目标。此时不增加 config switch/provider，不修改 cache admission、
production runner 或 six-input manifest。

locked Walk1 复用旧 manifest 的 clean/injected、seed 911、link `27956:20276`、闭区间
`[1664959678.3077347,1664959686.3077347]`、`+0.5m`、30 planned affected IDs 和原
keyframe plan。keyframe 0--4 为因果 bootstrap：只使用不晚于 keyframe 4 的 IMU/UWB，并将这些
group 标为 `BOOTSTRAP_HISTORY`。从 keyframe 5 起严格 detect-before-commit。每组保存已提交
UWB ID 集合/hash，若与 current IDs 相交则 fail-closed 为
`CONDITIONAL_PRIOR_CONTAMINATED_BY_CURRENT_UWB`。

shadow 使用 PL 的 iSAM2 relinearization 语义、15 维 marginal、fixed-lag 200 和 `0.02s` IMU
gap/reinitialize 语义；锁定 IE 输入的 IMU 噪声、重力、lever arm、anchor geometry 和逐观测 nominal
sigma 全部进入 identity。shadow state 永不传给 Stage2/final。v1 group 是一个 keyframe 的全部
planned observations，按 keyframe 时间/source order；每个 anchor 每组至多一条。

alarm group 对每个 anchor 用同一 prior 重建 leave-one-anchor-out `ν,H,R`，假设间不
commit。恰好一个 subset PASS 才 unique isolation；0 个为 `FDE_UNISOLATED_FAULT`；多于 1 个为
`FDE_ISOLATION_AMBIGUOUS`。isolated row 的 raw PL innovation `z-h>0` 才是 positive excess，
`z+=1m` 必须生成正 innovation；反方向只保留诊断。group PASS commit 整组；unique
isolation 只 commit healthy subset；ambiguous/unisolated/数值失败不 commit 整组并记
`prior_degradation_event`。

按现有 same-link、`gap>1s`、`count>=2`、`duration>=0.01s` 将 positive unique alarms
汇总为 `SupportPartition`；任意非 candidate planned row 中断该 link run。从 PL commit 导出独立
reference fixture，同一 synthetic `P,H,R,ν` 比较 physical/whitened innovation、S、statistic、DoF、
threshold 和决策；浮点容差 `1e-12+1e-10*scale`，离散字段精确。

clean/injected detector 必须在不接收 truth 参数的独立进程中运行并封存 hash/file-open
trace；之后 evaluator 才读 truth 依次裁决：

1. clean retained support 必须为 0；
2. affected groups 中至少一个 alarm；
3. 至少一次合法 unique isolation 命中 target anchor；
4. persistent support 命中 target link，且在时间或 obs IDs 上与 truth 重叠。

## 4. preflight PASS 后的 production 边界

只有 sealed preflight PASS 后才新增显式互斥 `nlos.pl_conditional_raim_fde_test:true`，provider/
version/rule 为 `pl_conditional_raim_fde_v1`、`UIFGO_PL_CONDITIONAL_RAIM_FDE_V1`、
`PL_CONDITIONAL_UNIQUE_LOAO_POSITIVE_TEMPORAL_V1`。identity 绑定 IE input、PL commit、
`p_fa`、group/order/whitening、bootstrap、prior/fixed-lag/IMU model、unique exclusion、positive sign、
temporal rule、healthy-subset commit 和数值策略。downstream 只接收 link、ordered obs IDs、时间和
segment provenance，不接收 innovation/shadow state/amplitude/slope/truth。

必需产物为 `pl_conditional_status.json`、`pl_conditional_epochs.csv`、
`pl_conditional_isolation.csv`、`pl_conditional_support.json` 以及字节一致的
`support_partition.json`/`partition.json`。C++ replay、Python publisher、SUCCESS_EMPTY 和 final admission
显式接受新 provider，legacy automatic/v2/v3/v4/oracle 规则不变，新旧 provider、Stage1
identity、partition hash 和 payload hash 必须双向拒绝。production 每 epoch 与 sealed preflight 的
离散字段精确 parity，浮点用上述容差；不一致停止为 implementation parity failure。

Stage2 始终从 IE batch raw Gaussian graph/Values 和新 support 启动；shadow navigation 不进 refit。
空 support 走现有 raw-reference 等价路径，非空 support 走冻结
`SegmentRefitter -> Rc -> sigma_c -> LCB/full -> final`。

## 5. 工程门、E2E 与裁决

测试覆盖 prior exclusion、PL parity、nominal full commit、正/反 fault、unique/ambiguous/unisolated、
假设间无 commit、persistent aggregation、healthy-subset 对后续 epoch 的影响、bootstrap 无未来信息、
IMU gap/reinitialize、cache 双向隔离和 preflight-production parity。完整 package/tests build、
`catkin run_tests uwb_imu_fgo`、`catkin_test_results` 必须比基线 384 项更多且
errors/failures/skipped 全为 0；Stage2/Rc/sigma/LCB/final/fallback/v4 replay 回归不得退化。

production 工程门后在新 E2E 根重建仅启用新 provider 的 clean/injected configs，并先通过
clean=0 support、injected=sealed support/target overlap 的 detector 门。之后共享单一 Stage2 cache 串行
`all_range`、`robust_cauchy`、`suppress_all`、`structured_debias`、`lcb_fixed_full`、
`lcb_partial`。用冻结 evaluator 报告所有可用指标、fallback 和 paired differences；严格
`lcb_partial RMSE < suppress_all RMSE` 才是 `BENEFIT_OBSERVED`。独立输出：
preflight PASS/FAIL、production detector PASS/FAIL、backend OPERATIONAL/FAILED、localization
BENEFIT_OBSERVED/NO_BENEFIT/NOT_EVALUABLE。

无论停在何门，都更新 `STATUS.md`、两份合同和 `paper/CLAIM_EVIDENCE.md`。只在真正进入
production 时创建 `doc/ie_0911/PL_CONDITIONAL_RAIM_E2E_RESULT.md`。不运行 Walk2/3/低冗余，
不扩展 C1--C3，不按结果调参。交付时保存命令/退出码/日志/hash/diff audit，显式暂存任务
文件，排除 `doc/v2/ie_0911/`。preflight 单独提交；若 PASS，production/E2E 再单独提交。
推送前 fetch 并核对远端起点，仅普通 non-force push；远端移动或认证失败时保留本地提交并报告。

## 6. 预检收口（2026-09-12）

独立 evaluator 的唯一裁决为 `PL_CONDITIONAL_PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM`。
clean 第一门以 0 retained segment 通过；injected 的 30/30 planned affected IDs 覆盖 30 个检测组，
但其中 alarm 数为 0，最大 `T=4.860523050243609`，同组门限 `30.856189940445919`。因此第三、
四门未准入，production/E2E 均 `NOT_RUN_PREFLIGHT_FAILED`。没有修改 `p_fa`、temporal 参数、注入
区间、Stage2/kappa/LCB/final 语义，也没有注册 production provider。完整证据和命令见
[`../ie_0911/PL_CONDITIONAL_RAIM_PREFLIGHT.md`](../ie_0911/PL_CONDITIONAL_RAIM_PREFLIGHT.md)。
