# 0912 Windowed FDE v4 与锁定 Walk1 E2E 协议

状态：`DONE / ENGINEERING_PASS / DETECTOR_GATE_FAIL_MISSED_INJECTION / LOCALIZATION_NOT_EVALUABLE`。
授权来源：用户于
2026-09-12 给定的 “Windowed FDE v4 与锁定 Walk1 E2E 实施计划”。本协议替代此前
grouped FDE Walk1 失败后的停止边界，但不改写历史结果。实施基线是实际 HEAD
`cf287b4fec9bc5689317f302ccf4cdd927bfba81`；受保护的未跟踪
`doc/v2/ie_0911/` 不读取、不修改、不暂存。

## 1. 冻结边界与身份

- `src/nlos_refit.cpp`、`include/uifgo/nlos_refit.h`、
  `src/nlos_recoverability.cpp`、`include/uifgo/nlos_recoverability.h` 相对基线必须零差异。
  Stage2、共同参考 `R_c`、local sigma、`kappa=2`、LCB、final solver、一次 fallback
  和 evaluator 数值语义均冻结；downstream 只允许识别新的 provider。
- v2/v3 缺省与 replay 语义保留。v4 只能由显式
  `nlos.fde_windowed_test: true` 启用，并与 `fde_grouped_test` 互斥。
- 固定 provider 为 `imu_aided_windowed_fde_v4`，identity version 为
  `UIFGO_IMU_AIDED_WINDOWED_FDE_IDENTITY_V4`，partition rule 为
  `FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4`。

## 2. Windowed detector

每条 planned link 按 `(tag_id,anchor_id,time,obs_id)` 排序，并仅以严格
`gap>T_gap` 切 continuous chain。每条 chain 的 `base_count=max(4,n_min)`；依次使用
`base_count*{1,2,4,...}` 且不超过 `min(64,chain_count)` 的 dyadic window size，stride
固定为 `max(1,size/2)`。常规 full-size window 后，如末窗未右对齐 chain 尾部，则补且仅补
一个唯一右对齐 full-size 尾窗。

对一条 chain，`m` 是上述生成并实际执行 covariance test 的全部 full-size window 数；
duration 不从 `m` 删除窗口。每窗从同一次完整 Gaussian graph/Values 线性化的 `P_gg`
取精确子矩阵，使用 `e_i=residual_i/factor_sigma_i` 调用既有 `FdeCovarianceTest(e,Pww)`。
保留原 rank/statistic/原始 p=.99 threshold，同时计算

```text
p_adjusted = 1 - (1 - 0.99) / m
threshold_adjusted = chi_square_quantile(rank, p_adjusted).
```

只有严格 `statistic>threshold_adjusted` 才拒绝；等号不拒绝。GLS 沿用当前协方差伪逆
rank threshold，以 `d_i=1/sigma_i` 计算 signed common residual；只接受 `b_gls<0`。
window 不以 pointwise fault 为前置。significant window 同时满足 adjusted rejection、负 GLS、
原 `n_min` 与 `T_min`。

同一 chain 内，以观测序号将 observation-set 重叠或直接相邻的 significant windows 合并；
输出 obs ID 是窗口集合的精确 union，不补未覆盖观测，然后再次执行原 count/duration 过滤。
多个 link 产生有效 segment 时返回 `FDE_ISOLATION_AMBIGUOUS` 与空 support。chain/window/
segment ID 由 v4 identity/context 与有序 obs IDs 作确定性 SHA-256；所有合并窗口记录最终
`merged_segment_id`。

## 3. Artifacts、cache 与测试

v4 成功或失败均输出原 `fde_status.json`、`fde_observations.csv`、
`support_partition.json`，以及专属 `fde_local_windows.csv` 和
`fde_windowed_summary.json`；成功时 `partition.json` 与 support 文件字节一致。
window CSV 分列保存 `obs_ids` 与 `obs_ids_sha256`，并保存 chain/window identity、link、
位置/时间、`m`、rank、原始/adjusted p 与 threshold、statistic、GLS、significant、过滤状态和
merged segment identity。summary 固定保存完整 policy、family alpha、chain/window/test/
significant/merged/retained 计数及最终状态。

producer/cache identity 绑定 v4 provider、window/Bonferroni/merge policy、原 temporal 与 preliminary
solver context。Python publisher、C++ replay、SUCCESS_EMPTY 和 final provider 检查仅扩展到 v4，
且 v4 强制两个专属 artifacts；v3 payload/identity/historical admission 不变。双向 v3/v4 cache
错配必须拒绝，并证明 v3 自身 replay 仍接受。

确定性测试必须覆盖 dilution、clean、boundary merge、`10/base4 => m=6` multiplicity 与严格边界、
correlated covariance/GLS、positive sign、ambiguity、invalid covariance/mapping、tail、ID determinism，
以及全部既有 Stage2/Rc/local-sigma/LCB/final/fallback 回归。工程门为工作空间根的 package build、
tests build、`catkin run_tests` 和 `catkin_test_results`；只有总数不少于基线 370 且
errors/failures/skipped 全零才准入 Walk1。

## 4. 锁定 Walk1 与停止门

唯一输出根为 `/home/mint/ws_fusion_uwb/res/windowed_fde_e2e_20260912_01`，已存在即拒绝覆盖。
输入只从旧 `locked_manifest.json` 读取 Walk1 normal clean/injected，并核验 manifest/input/truth hash、
anchor subset、seed 911、link `27956:20276`、闭区间
`[1664959678.3077347,1664959686.3077347]`、`+0.5 m`、111 raw/30 planned；只有隔离
effective config 新增 v4 opt-in。

先运行 truth/GT/oracle 隐藏且有 file-access trace 的 clean detector-only；若 retained segment 非零，
写 `DETECTOR_GATE_FAIL_CLEAN_FALSE_SUPPORT` 并停止 estimator。通过后运行 injected detector-only；
进程结束、artifacts 冻结后独立 evaluator 才读取 truth，报告 planned TP/FP/FN、precision/recall/F1、
obs-set IoU 和 target-link temporal overlap/IoU/start/end error。无 target-link overlap 则写
`DETECTOR_GATE_FAIL_MISSED_INJECTION` 并停止。

两个 screen 通过后，自动对同一 injected input 串行运行 `all_range`、`robust_cauchy`、
`suppress_all`、`structured_debias`、`lcb_fixed_full`、`lcb_partial`。四个 candidate-dependent
final 必须共享 v4 Stage2 cache、partition hash 与 Stage2 Values；E2E producer support 必须与 screen
的 provider/hash/obs union 相同，并由 Stage2 直接接受，且 estimator trace 无 truth/GT open。
报告 Stage2 status、每段 `c_hat`、navigation gradient、Rc/local sigma、full/LCB delta、fallback、
final graph/Values identity与六方法 aligned RMSE/P95/horizontal/vertical RMSE/coverage。

detector PASS 当且仅当 clean=0、injected production support 与 truth overlap、且同一 production support
被 Stage2 接收。所需 final/metric 不完整则 localization=`NOT_EVALUABLE`；否则 fixed-full 或 LCB 任一
aligned RMSE 严格小于 suppress 时为 `BENEFIT_OBSERVED`，否则 `NO_LOCALIZATION_BENEFIT`。仅当
`lcb_partial<suppress_all` 时附加 `LCB_VS_SUPPRESSION_BENEFIT_OBSERVED`。任何 detector、Stage2、
Rc、LCB 或 final 失败均不调 policy、不扩大矩阵；即使全部通过也只报告允许下一轮扩大，不执行
Walk2/3 或低冗余矩阵。

## 5. 交付与 claim 边界

创建 `doc/ie_0911/WINDOWED_FDE_E2E_RESULT.md`，记录命令、退出码、证据路径、失败与 `NOT_RUN`；
同步更新 `STATUS.md` 和 `paper/CLAIM_EVIDENCE.md`。单次 locked Walk1 只是 development E2E 证据，
不构成总体误报保证或正式 held-out 结果；T10=C2-C、T11=C、C1--C3 不升级。

结束时再次证明四个 backend 文件相对 `cf287b4` 零差异，冻结论文结构/roadmap hash 未变。
只暂存本任务文件并排除 `doc/v2/ie_0911/`，提交消息固定为
`Add covariance-consistent windowed FDE v4`；正常 fast-forward push 到
`origin/feature/uwb-imu-fusion-ie-postprocessing`，非 fast-forward 时停止，不 rebase、不 force。
