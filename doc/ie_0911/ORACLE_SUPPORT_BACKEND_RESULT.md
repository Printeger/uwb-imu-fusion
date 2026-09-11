# Oracle-support recovery backend sanity check

最终裁决：**RECOVERY_BACKEND_NOT_OPERATIONAL**。这是 backend-only oracle-support development diagnostic，不是端到端 detector 成绩。

## 1. 基线与锁定输入

实际 git HEAD 为 `15b32f5afa01c5a0f8bc432bbac78c53bfa36420`。使用既有 SFUISE Walk1 normal-redundancy constant-step injection：seed 911，link `27956:20276`，闭区间 `[1664959678.3077347,1664959686.3077347]`，30 个已锁定 planned obs IDs。输入 cache、truth 与 locked manifest 身份见 machine-readable result。

## 2. Oracle support 与 truth 隔离

复用已有 `OracleSupportProvider -> SupportPartition` 接口。传给 estimator 的 YAML 只含 segment ID、link 和 30 个 obs IDs；30/30 都是 valid/planned 且属于唯一目标 link。原 truth 目录和 evaluator GT 在 estimator 进程树内由 bwrap 隐藏；strace 观察到 support 文件读取，未观察到 injection truth 或 GT 打开。effective config、support manifest 和 Stage2 参数中没有 true amplitude 字段。`0.5 m` 只由本独立 evaluator 在 estimator 结束后读取。

证据：`oracle_support_alignment_audit.json`、`truth_separation_audit.json`、`oracle_support.json` 和原始 file-access trace。当前 runner 的 fixed-partition `input_manifest.oracle_support_read=false` 是已有导出字段局限；实际读取以 strace 为准，没有为本任务修改 runner。

## 3. Stage2 实际执行与失败

support 非空，Stage2 实际执行 50 次 conditional optimizer 调用（累计 110 LM iterations / 359 inner trials），未走 SUCCESS_EMPTY。objective 从 2029099755.93 降至 128.79193031。最后 objective、scaled-step 和非负 KKT 条件通过；KKT violation=1.83203996144e-14。但 navigation stationarity gradient=0.0750243858624，远高于冻结容差 1e-06 加 roundoff 2.2355735248e-11。50 次外层迭代后返回 `MAX_REFIT_ITERATIONS`，没有导出未收敛 Values 或 c_hat。没有发现 support/link/obs identity plumbing 错误，因此未修改算法、容差或 solver policy，也未重试。
首次 batch 在 estimator 启动前因 scheduler provenance 标签缺少既有 `TEST_ONLY/PENDING_VALIDATION` 标记而失败；trace 核验 estimator exec=0。仅修正元数据标签后使用 fresh 目录执行上述唯一科学运行。

## 4. Bias、Rc 与补偿

由于 Stage2 没有收敛，`c_hat`、其相对 0.5 m 的误差、Rc rank/conditioning、`sigma_c`、full correction 和 `max(0,c_hat-2*sigma_c)` 全部为 `UNAVAILABLE_STAGE2_FAILED`。没有手工 uncertainty 或 correction fallback，也没有把未收敛中间状态冒充有效估计。

## 5. Final optimization

Stage2 cache 未发布；suppress_all、structured_debias、lcb_fixed_full 和 lcb_partial 均为 `PARENT_CACHE_UNAVAILABLE`。它们的 final optimizer calls 为 0，corrected factors 不可用。没有 empty-support graph reuse，也没有 CSV-only correction。

## 6. 定位精度

使用冻结的同一 evaluator；coverage 是 planned trajectory coverage。

| condition | method | status | RMSE m | P95 m | horizontal RMSE m | vertical RMSE m | coverage |
|---|---|---|---:|---:|---:|---:|---:|
| clean | all_range | COMPLETE | 0.163853268 | 0.298978070 | 0.123735494 | 0.107412388 | 1.000000000 |
| clean | robust_cauchy | COMPLETE | 0.163847231 | 0.294238429 | 0.126963906 | 0.103566798 | 1.000000000 |
| injected | all_range | COMPLETE | 0.198923089 | 0.343576674 | 0.132749869 | 0.148148126 | 1.000000000 |
| injected | robust_cauchy | COMPLETE | 0.196904359 | 0.356312042 | 0.143714159 | 0.134601512 | 1.000000000 |
| injected | suppress_all | PARENT_CACHE_UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE |
| injected | structured_debias | PARENT_CACHE_UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE |
| injected | lcb_fixed_full | PARENT_CACHE_UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE |
| injected | lcb_partial | PARENT_CACHE_UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE | UNAVAILABLE |

相对 clean raw，injected raw RMSE 增加 0.035069820 m（21.403%）。

LCB-vs-suppress、full-vs-suppress、structured-vs-suppress 和 LCB-vs-full 的绝对 RMSE 变化与百分比改善均不可用；原因是 perfect support 下 Stage2 失败，而不是 detector 漏检。所有负结果和 parent-unavailable 状态保留。

## 7. 四个问题与裁决

- Q1 Bias estimation：不能回答接近程度；没有有效 `c_hat`。
- Q2 Recoverability：Rc 未执行，不能产生有限 `sigma_c`。
- Q3 Bounded compensation：LCB 未执行，属于 unavailable，既不能称 0 correction，也不能称 over-correction。
- Q4 Localization benefit：LCB 和 suppress final 均未运行，无法比较；没有观测到收益。

因此按预先规则唯一裁决为 **RECOVERY_BACKEND_NOT_OPERATIONAL**，暂停 detector 开发。normal backend 未通过，故锁定 low-redundancy oracle check 为 `NOT_RUN_NORMAL_BACKEND_NOT_OPERATIONAL`。没有修改 FDE、Stage2、Rc、LCB、优化器或 evaluator，没有运行下一阶段算法。

完整机器证据位于 `/home/mint/ws_fusion_uwb/res/oracle_support_backend_20260911_01`。
