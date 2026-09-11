# Windowed FDE v4 与锁定 Walk1 E2E 结果

状态：`DONE / ENGINEERING_PASS / DETECTOR_GATE_FAIL_MISSED_INJECTION / LOCALIZATION_NOT_EVALUABLE`。
本结果是单一锁定 Walk1 development gate，不是 held-out 结果、总体误报保证或论文性能数字。
完整机器证据位于
`/home/mint/ws_fusion_uwb/res/windowed_fde_e2e_20260912_01`。

## 1. 基线与初始工作树

实施基线和工程门时 HEAD 均为 `cf287b4fec9bc5689317f302ccf4cdd927bfba81`。实施前
`git status --short` 仅含受保护的 `?? doc/v2/ie_0911/`；该目录未修改、未暂存。证据见
`engineering/final/engineering_gate.json`。

## 2. 协议与合同

运行前已登记 [协议](../ie_sprint/WINDOWED_FDE_E2E_PROTOCOL.md)，并在
`METHOD_CONTRACT.md`、`EXPERIMENT_CONTRACT.md` 和 `STATUS.md` 登记本次 amendment。
冻结论文结构和 roadmap 未修改。

## 3. 冻结 backend

以下四个文件相对基线 diff 均为空；最终工程门记录的 SHA-256 分别为：

- `src/nlos_refit.cpp`: `3c74724387956d930ad4e9fe711d1af9f1b3db4cd69430f2e31b8087e1be5d4b`
- `include/uifgo/nlos_refit.h`: `a0bcc1de76092e31110644936d40048276e228c75efc5977b153aac56c97c5ca`
- `src/nlos_recoverability.cpp`: `cd9b631bdebddd7fdbfcf35af615984d302e5359b2531f255bf624f4dac11cdd`
- `include/uifgo/nlos_recoverability.h`: `7942dcc1aeedb7d42d43d5f651973a3946bf2af761122a69eb1e01e0707f0dfa`

## 4. 显式版本边界

新增配置 `nlos.fde_windowed_test: true`，与 `fde_grouped_test` 互斥。provider、version 和
partition rule 固定为 `imu_aided_windowed_fde_v4`、
`UIFGO_IMU_AIDED_WINDOWED_FDE_IDENTITY_V4`、
`FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4`。v2/v3 默认、replay 和 artifacts 语义保留。

## 5. chain 与 window bank

每 link 按 `(tag_id,anchor_id,time,obs_id)` 排序，严格 `gap > T_gap` 切 chain。实现固定
`base=max(4,n_min)`、不超过 `min(64,chain_count)` 的 dyadic size、50% stride，以及唯一右对齐尾窗。
`m` 是该 chain 实际生成并执行 covariance test 的全部 full-size window 数，duration 不从分母删窗。

## 6. covariance、GLS 与 merge

每窗从一次 FullGraph Gaussian linearization 的完整 whitened residual projection 取协方差子块，按
`e_i=residual_i/sigma_i` 运行原 covariance test，再使用
`p_adj=1-(1-0.99)/m` 和按实际 rank 的卡方分位数；只有严格 `statistic > threshold_adj` 才 reject。
GLS 沿用原伪逆秩阈值和 `d_i=1/sigma_i`，仅负 GLS 进入 significant。窗口无需 pointwise fault 前置。
significant windows 仅在同 chain 内按观测集合重叠/直接相邻合并为精确 union；多 link 合格时为空支持并报告
`FDE_ISOLATION_AMBIGUOUS`。

## 7. identity、artifacts 与 cache

chain/window/segment ID 绑定 v4 identity/context 和有序 obs IDs。v4 成功/失败路径生成
`fde_local_windows.csv`、`fde_windowed_summary.json`、`fde_observations.csv`、
`support_partition.json`；成功时 `partition.json` 与 support 文件字节一致。Python publisher、C++ replay、
SUCCESS_EMPTY 和 final provider 校验均识别 v4，且 v4 cache 要求专属 artifacts；v3 payload 要求未改变。

## 8. 确定性测试

新增并通过 dilution、clean、boundary merge、`m=6` multiplicity/尾窗/严格等号边界、相关协方差手算、
正号拒绝、ambiguity、invalid covariance/mapping 和 ID determinism 测试。缓存合同覆盖 v3 自 replay 继续接受，
以及 v3→v4、v4→v3 双向拒绝；既有 Stage2、Rc/local sigma、LCB、final/fallback 回归均包含在完整工程门中。

## 9. 完整工程门

工作目录均为 `/home/mint/ws_fusion_uwb`。最终隔离证据目录为 `engineering/final`：

| 命令 | exit | 日志 |
|---|---:|---|
| `catkin build uwb_imu_fgo --no-deps -j4` | 0 | `engineering/final/01_build.log` |
| `catkin build uwb_imu_fgo --no-deps --make-args tests -j4` | 0 | `engineering/final/02_build.log` |
| `catkin run_tests uwb_imu_fgo` | 0 | `engineering/final/03_run_tests.log` |
| `catkin_test_results build/uwb_imu_fgo/test_results/uwb_imu_fgo` | 0 | `engineering/final/04_catkin_test_results.log` |

权威汇总为 `384 tests, 0 errors, 0 failures, 0 skipped`，超过冻结基线 370。XML 根节点原始计数为
192/0/0/0（17 files），也保留在 JSON 中。第一次证据工具在第 4 条日志名中误用含 `/` 的参数而异常；
第二次四条命令均 exit 0，但自写汇总误把 XML 根计数直接与 catkin 的 370 口径相比。两次基础设施失败分别保留在
`engineering/` 和 `engineering/formal/`，没有作为产品失败、也没有覆盖；修正后在新 `engineering/final/`
完整重跑。

## 10. 锁定 Walk1 输入

原 `locked_manifest.json` 校验为
`sha256:1660f7a1e8d1bd32bfabda394adf9e69675fb849917b028b66fff7ec0e87e820`。
固定 subset `[7475,9524,10548,15155,20276]`、seed 911、link `27956:20276`、闭区间
`[1664959678.3077347,1664959686.3077347]` 和 `+0.5 m` 均一致；injected sidecar 为 111 个 raw affected、
其中 30 个进入实际 plan。input manifest、IMU/UWB payload 和两套 truth sidecar 的固定哈希全部通过；见
`locked_input_audit.json`。唯一隔离根此前不存在，本任务只使用
`/home/mint/ws_fusion_uwb/res/windowed_fde_e2e_20260912_01`。

## 11. estimator truth 隔离

clean 与 injected detector 进程树均在 bubblewrap 中隐藏 injection truth、评价 data 和既有 GT 目录，并由
`strace -f -e trace=%file` 审计。两份 trace 的 forbidden open 均为 0；评价器仅在进程结束后读取 sidecar。
证据在 `walk1_execution.json` 的 `access_audit` 和 `detector_screens/*_file_access.trace`。

## 12. clean detector-only

reference 94 iterations 收敛；1074/1074 planned observations 完整测试；9 chains、1010 covariance windows、
0 significant window、0 merged/retained segment，状态 `SUCCESS / WINDOWED_CONSISTENT_NO_SUPPORT`。
clean gate 通过。空 truth 且空 prediction 下 precision/F1/IoU 分母为零，不把该单次结果写成总体误报保证。

## 13. injected detector-only

reference 53 iterations 收敛；1074/1074 完整测试；同为 9 chains、1010 windows，但仍是 0 significant window、
0 merged/retained segment。相对 30 个 planned injected IDs：TP=0、FP=0、FN=30，recall=0、F1=0、
obs-set IoU=0；precision 为零预测分母下未定义。目标 link 无输出 segment，因此 temporal overlap/IoU 与
start/end error 均不可用，触发 `DETECTOR_GATE_FAIL_MISSED_INJECTION`。

## 14. correctness 结果后审计

审计未改变任何 detector 参数。injected 目标窗口中最大 `statistic/threshold_adj=0.674515`：rank 32、
`49.205998 < 72.950243`，GLS 为 `-0.159349 m`，说明符号已满足但严格 adjusted covariance rejection 未满足。
目标 30 条 post-fit residual 均为负，均值 `-0.195576 m`。全局最大比值也只有 0.751187。clean 全局最大比值
0.664393。完整记录见 `correctness_audit.json`；该结果与固定 Bonferroni policy 一致，不构成调参授权。

## 15. detector 裁决

clean=0 支持，但 injected production support 与 truth 无 overlap，故 detector 唯一裁决为 `FAIL`。这不是
covariance、mapping、GLS sign 或 artifact 缺失导致的运行失败，而是锁定 policy 在该输入上没有 reject 窗口。

## 16. Stage2 与六方法

按预登记停止门，未启动 injected E2E producer。因此 production support 的 Stage2 接收一致性、Stage2 status、
每段 `c_hat`、navigation gradient、Rc、local sigma、`delta_full`、`delta_lcb`、fallback、final graph/Values
identity，以及 `all_range`、`robust_cauchy`、`suppress_all`、`structured_debias`、`lcb_fixed_full`、
`lcb_partial` 六方法均为 `NOT_RUN_DETECTOR_GATE_FAIL_MISSED_INJECTION`。

## 17. localization 裁决

所有所需 final/指标均未运行，aligned ATE RMSE、P95、horizontal/vertical RMSE、coverage，LCB/full 相对
suppress 和 injected raw 的四个严格差值均不可用。localization=`NOT_EVALUABLE`；不产生
`LCB_VS_SUPPRESSION_BENEFIT_OBSERVED`。

## 18. 后续范围与 claim

Walk2/3、低冗余和完整精度矩阵均 `NOT_RUN`；当前结果不允许扩大下一轮。未根据结果修改 window、probability、
temporal 参数、kappa、区间或阈值。T10=C2-C、T11=C、C1--C3 限制和旧结果/默认均不变。
最终 `sealed_hashes.json` 文件 SHA-256 为
`0f878cd29b91515e982da166be4f4d70e14c03e1f29f6d71d5ac9fca18a136dc`；其规范内容 seal 为
`sha256:4a317e59fd1262d5d2ea4d6e3b43b952e0814adaf8fe58dd01b35d04fd97f795`。
