# Canonical persistent positive range injection

预登记：ISAS Walk1 / tag27956 / anchor20276；absolute range +1.0m，
[1664959684.9745398,1664959694.9745398)，恰好10s（recording origin后8s开始）。
每个目标raw observation修改 observed_range_m，其余字段、obs_id、时间戳和source validity原样；
包括协议invalid行，原invalid仍不用于有效误差统计。IMU软链接clean父输入，GT不修改。
父/子SHA、affected obs IDs、精确注入值、区间和变换身份由evaluator私有provenance记录。
Estimator只挂载不含truth的measurement manifest/payload，truth不进入config/cache。
clean/corrupted独立fresh四方法和各一个Stage2 producer，M3/M4仅在同条件内共享producer。
沿用锁定CUSUM/solver/recovery，单案例无sweep，不按结果重试或调参。
ATE沿用统一nearest0.02s与全区间SE3 scale1。NLOS-window RMSE取同一全区间alignment下
位于注入半开窗口的误差，不单独窗口对齐；严格核对GT配对集合，不删除失败方法。
Recover−Reject定义M4−M3，负值表示降低误差。
Range分开报告measured_child−measured_clean（精确注入分量）及compensated_child−measured_clean，
和继承单位GT外参/共同clock假设的几何range error；前者不是未知原始误差/total NLOS truth。

## 实际 canonical smoke 结果

状态 CANONICAL_PIPELINE_PASS；clean/corrupted共8方法+2独立producer，10/10成功，0失败，0fallback。
各条件独立fresh前端，source_hash/discovery_snapshot_hash不同；run_id无复用，实际10个挂载清单
均无GT/truth/跨条件cache路径。IMU hash与父输入相同，GT hash不变。
162条raw observation受到注入，其中144条source-valid，36条valid+planned；协议invalid18条保留且不计指标。
两条件planned IDs均1074条且完全相同。clean support为空；corrupted一个segment，TP36/FP0/FN0，
这些检测数只在valid+planned集合中计算；不把未计划的raw观测算漏检，也不把clean潜在NLOS当已知标签。
Stage2 estimated segment bias=1.0583880618576771 m，M4接受该段，对36个planned observation各应用一次同值固定补偿；
M3 ZERO_ACCEPTED是成功拒绝策略，非失败/fallback。未对未计划的108条有效受影响观测外推补偿。

所有8方法同229个GT匹配样本，注入窗口内39个。ATE保持原tracker代理指标；NLOS-window使用
相同full-interval SE3对齐后的窗口误差，未对窗口单独拟合。单位m：

| condition | method | ATE RMSE | NLOS-window RMSE |
|---|---|---:|---:|
| clean | M0 | 0.163853268 | 0.144806453 |
| clean | M1 | 0.163847231 | 0.145214098 |
| clean | M3 | 0.163853268 | 0.144806453 |
| clean | M4 | 0.163853268 | 0.144806453 |
| corrupted | M0 | 0.310080254 | 0.566835577 |
| corrupted | M1 | 0.281516860 | 0.503333901 |
| corrupted | M3 | 0.185094827 | 0.249326011 |
| corrupted | M4 | 0.170312603 | 0.182449138 |

Recover−Reject（M4−M3，负值表示改善）：corrupted ATE=-0.014782224m，
NLOS-window=-0.066876872m；clean两项均0。

Range error必须区分定义（corrupted，M4）：

| 集合 | 样本数 | clean配对注入分量 RMSE before→after | 假设几何 range RMSE before→after |
|---|---:|---:|---:|
| all_valid | 4266 | 0.183726085 → 0.159201837 | 1.673507351 → 1.679934502 |
| affected_valid | 144 | 1.000000000 → 0.866517335 | 0.715127670 → 1.072342064 |
| affected_valid_planned | 36 | 1.000000000 → 0.058388062 | 0.732991583 → 1.758213961 |

几何range使用此前用户授权的单位外参/同一时基，标ASSUMED_GEOMETRY_AND_CLOCK。
其误差变大也如实报告；不能用注入分量改善替代真实绝对range误差改善。未按结果调整任何参数。

## 使用与交付

一次完整复现（会重新运行clean和corrupted各四方法）：

```bash
python3 experiments/scripts/run_controlled_smoke.py
```

本次实际以 --estimate-only 先封存estimator，再运行独立evaluate_controlled_smoke.py。
为了将窗口指标同步到统一trajectory_metrics.csv，evaluator额外运行一次；没有重复运行estimator。
两次evaluator日志保留。脚本没有amplitude/duration/anchor可选参数，固定单案例。
依赖既有编译runner、bwrap、Python numpy/yaml和既有Walk1 measurement cache/私有calibration。

[runs.csv](/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie/experiments/results/controlled-20260912T100525Z-89b5c73afbb2/runs.csv)、[统一trajectory_metrics.csv](/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie/experiments/results/controlled-20260912T100525Z-89b5c73afbb2/trajectory_metrics.csv)、
[注入truth](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/injection_truth.json)、[所有affected IDs和前后range](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/affected_observations.csv)、
[detection support](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/detection_support.csv)、[检测统计](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/detection_metrics.csv)、
[estimated segment bias](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/estimated_segment_bias.csv)、[range汇总](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/range_summary.csv)、
[逐观测clean配对range](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/corrupted_paired_range.csv)、[Recover−Reject](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/paired_difference.csv)、
[完整结果与隔离审计](/home/mint/ws_fusion_uwb/evaluator_private/icra/controlled/controlled-20260912T100525Z-89b5c73afbb2/evaluation-9dc25a2dcf214d18b6d926ab2803e90c/summary.json)。每条件的原final_masks/fixed_compensations在其各自batch/runs下保留。
GT/oracle派生明细只写evaluator_private，experiment目录仅保留统计/指针，均不在estimator挂载范围。

3/3 injection/window tests、5/5 harness、15/15 range/isolation tests exit0；
[命令/退出码/产物与代码SHA](evidence/controlled_injection_20260912/execution.json)。
当前未发生算法失败/fallback；没有真实fallback执行案例，工程fixture覆盖保留。
核心build/CTest、其他recording、sweep、正式held-out/论文收益claim、独立外参标定均NOT_RUN。
不修改核心算法/参数、不提交/push；历史失败与其他用户改动保留。
