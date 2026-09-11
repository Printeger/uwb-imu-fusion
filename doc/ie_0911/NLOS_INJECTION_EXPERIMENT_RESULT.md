# 冻结算法真实数据 NLOS 注入 E2E 结果

实际基线 `46d37f6709aeedb4958e484683fd51e5fd8ec0cc`。本轮仅修改实验输入、调度和评价接口；算法和科学参数冻结。semi-synthetic persistent NLOS-on-real-data，B=0.5 m、目标8 s、seed=911、闭区间；truth只覆盖新增分量，天然总bias未知。

完整证据目录：`/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01`。机器长表：[Table A](/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01/report/table_a.csv)、[Table B](/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01/report/table_b.csv)、[完整JSON](/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01/report/results.json)、[锁定manifest](/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01/locked_manifest.json)。

## 选择与准入

- sfuise_walk1: normal link/window `[[27956, 20276], 1664959678.3077347, 1664959686.3077347]`；low subset `[9524, 15155, 20276]`。所有排序候选、失败及排除原因见锁定manifest。
- sfuise_walk2: normal link/window `[[27956, 10548], 1664959758.3045385, 1664959766.3045385]`；low subset `[7475, 9524, 10548, 20276]`。所有排序候选、失败及排除原因见锁定manifest。
- sfuise_walk3: normal link/window `[[27956, 10548], 1664959849.2025266, 1664959857.2025266]`；low subset `[7475, 9524, 10548]`。所有排序候选、失败及排除原因见锁定manifest。
- sfuise_walk1_normal_clean: clean gate `True`；状态 `SELECTED`。
- sfuise_walk1_low_clean: clean gate `False`；状态 `SELECTED`。
- sfuise_walk2_normal_clean: clean gate `False`；状态 `SELECTED`。
- sfuise_walk2_low_clean: clean gate `True`；状态 `SELECTED`。
- sfuise_walk3_normal_clean: clean gate `True`；状态 `SELECTED`。
- sfuise_walk3_low_clean: clean gate `True`；状态 `SELECTED`。

## Table A：逐段 Detection / Recovery

每个scene与候选方法保留零段、失败和跳过行。P/R为正 excess candidate的全planned检测指标；完整双边、temporal、overlap、分母、bias误差、Rc与fallback见长表。FP只相对新增注入分量。

| scenario | method | segment | Stage2 | P / R | c_hat (m) | sigma local (m) | proposed offset (m) | final-use | status |
|---|---|---|---|---|---:|---:|---:|---|---|
| sfuise_walk1_normal_clean | suppress_all | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_clean | structured_debias | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_clean | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_clean | lcb_partial | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_injected | suppress_all | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_injected | structured_debias | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_injected | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_normal_injected | lcb_partial | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_low_clean | suppress_all | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_low_clean | structured_debias | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_low_clean | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_low_clean | lcb_partial | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk1_low_injected | suppress_all | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk1_low_injected | structured_debias | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk1_low_injected | lcb_fixed_full | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk1_low_injected | lcb_partial | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk2_normal_clean | suppress_all | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_normal_clean | structured_debias | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_normal_clean | lcb_fixed_full | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_normal_clean | lcb_partial | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_normal_injected | suppress_all | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk2_normal_injected | structured_debias | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk2_normal_injected | lcb_fixed_full | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk2_normal_injected | lcb_partial | NONE | UNAVAILABLE | NA / NA | NA | NA | NA | NA | SKIPPED_CLEAN_GATE |
| sfuise_walk2_low_clean | suppress_all | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_clean | structured_debias | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_clean | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_clean | lcb_partial | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_injected | suppress_all | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_injected | structured_debias | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_injected | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk2_low_injected | lcb_partial | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_clean | suppress_all | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_clean | structured_debias | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_clean | lcb_fixed_full | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_clean | lcb_partial | NONE | SUCCESS_EMPTY | 0.000000 / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_injected | suppress_all | NONE | SUCCESS_EMPTY | 0.000000 / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_injected | structured_debias | NONE | SUCCESS_EMPTY | 0.000000 / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_injected | lcb_fixed_full | NONE | SUCCESS_EMPTY | 0.000000 / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_normal_injected | lcb_partial | NONE | SUCCESS_EMPTY | 0.000000 / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_clean | suppress_all | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_clean | structured_debias | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_clean | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_clean | lcb_partial | NONE | SUCCESS_EMPTY | NA / NA | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_injected | suppress_all | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_injected | structured_debias | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_injected | lcb_fixed_full | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |
| sfuise_walk3_low_injected | lcb_partial | NONE | SUCCESS_EMPTY | NA / 0.000000 | NA | NA | NA | NA | COMPLETE |

## Table B：六方法 aligned RMSE (m)

| dataset / regime / condition | all_range | robust_cauchy | suppress_all | structured_debias | lcb_fixed_full | lcb_partial | ΔRMSE LCB−suppress | improvement % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| sfuise_walk1_normal_clean | 0.163853 | 0.163847 | 0.163853 | 0.163853 | 0.163853 | 0.163853 | 0.000000 | 0.000000 |
| sfuise_walk1_normal_injected | 0.198923 | 0.196904 | 0.198923 | 0.198923 | 0.198923 | 0.198923 | 0.000000 | 0.000000 |
| sfuise_walk1_low_clean | 0.307300 | NA (COMPLETE_WITH_RUN_FAILURE) | 0.307300 | 0.307300 | 0.307300 | 0.307300 | 0.000000 | 0.000000 |
| sfuise_walk1_low_injected | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA | NA |
| sfuise_walk2_normal_clean | 0.228473 | NA (COMPLETE_WITH_RUN_FAILURE) | 0.228473 | 0.228473 | 0.228473 | 0.228473 | 0.000000 | 0.000000 |
| sfuise_walk2_normal_injected | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA (SKIPPED_CLEAN_GATE) | NA | NA |
| sfuise_walk2_low_clean | 0.257227 | 0.222910 | 0.257227 | 0.257227 | 0.257227 | 0.257227 | 0.000000 | 0.000000 |
| sfuise_walk2_low_injected | 0.543552 | NA (COMPLETE_WITH_RUN_FAILURE) | 0.543552 | 0.543552 | 0.543552 | 0.543552 | 0.000000 | 0.000000 |
| sfuise_walk3_normal_clean | 0.178023 | 0.175867 | 0.178023 | 0.178023 | 0.178023 | 0.178023 | 0.000000 | 0.000000 |
| sfuise_walk3_normal_injected | 0.260858 | 0.253056 | 0.260858 | 0.260858 | 0.260858 | 0.260858 | 0.000000 | 0.000000 |
| sfuise_walk3_low_clean | 0.833609 | 0.830300 | 0.833609 | 0.833609 | 0.833609 | 0.833609 | 0.000000 | 0.000000 |
| sfuise_walk3_low_injected | 0.864092 | 0.863016 | 0.864092 | 0.864092 | 0.864092 | 0.864092 | 0.000000 | 0.000000 |

定位采用原20ms最近邻、scale=1 SE(3)；窗口指标使用全轨迹alignment。P95、horizontal/vertical、coverage、observations retained、failure/fallback及窗口数值均在Table B长表。

## 裁决与限制

“同一persistent bias被检测后，LCB是否优于suppression”：证据不足；没有同时满足实际非空temporal support与有效两方法定位输出的配对。
“降低冗余是否放大收益”：逐recording差值（low收益−normal收益，m）为 sfuise_walk3=0.000000；失败和负收益见完整配对JSON，不从三个recordings推断统计显著性。
本轮所有10个已执行scenario的temporal support均为空；4个injected的正excess检测TP均为0。LCB与suppression的相同轨迹来自原有空support行为，没有实际恢复补偿。唯一有效normal/low配对Walk3的收益均为0，未观察到降低冗余放大收益；检测后恢复优势仍证据不足。

local sigma只是同Stage2共同Rc的局部诊断，不是校准置信保证或最终bias后验。suppressed/fallback观测不被记为实际补偿；混合段单列support加权新增分量。Range error因原frame/anchor/lever/time provenance缺口为UNAVAILABLE，不以post-fit residual或aligned GT补齐。T10=C2-C、T11=C、C1–C3不升级。sensitivity、新算法、新phase、自动commit/push均NOT_RUN。

## 实现、测试与命令证据

新增SFUISE原loader导出、闭区间step生成器、v2输入envelope、FDE-only筛选、1800s进程树限制、确定性选择和独立评价/表格生成。原T07半开语义及默认路径保持。

修改前baseline.tar.gz/baseline_hashes.json保存源码、配置、runner及已链接依赖；engineering/frozen_algorithm_audit.json核对算法和科学参数。构建与完整CTest（30/30）日志见engineering/。禁用注入与原loader、clean/injected初值/计划/噪声、C++往返、truth字段拒绝、跨scenario replay拒绝与共享上游已实际检查。bwrap隐藏源bag/生成truth目录后，strace筛选通过且未读取truth、未执行Stage2。

所有实际命令、退出码、墙钟和日志汇总到report/commands.json；各进程原始.command.json、batch中command.json和run_status均保留。工程首次导出因YAML origin精度损失拒绝，修为17位往返；首次批处理provenance白名单拒绝，未运行estimator；首次新target直接build失败，随后catkin重新配置通过。旧失败目录均保留，不属于算法重试。

## 暂停恢复与最终计数

用户低电量暂停的producer记录为INTERRUPTED_USER_PAUSE；随后按明确续跑指令从原始输入新建producer，旧尝试保留，不作为算法失败或超时。PAUSED.json、RESUME.json与interrupted_command.json保存完整记录。最终12个scenario条目，10个执行；Table B共72行，状态计数为{'COMPLETE': 57, 'COMPLETE_WITH_RUN_FAILURE': 3, 'SKIPPED_CLEAN_GATE': 12}。另有1次用户中断；方法TIMEOUT计数为0。所有已完成与算法失败项未重跑。

最新独立指标工程测试7/7通过；完整CTest30/30已通过。后补精确planned时间覆盖检查与窗口边界审计均未改变锁定选择、准入或缓存；见engineering/final_results_audit.json、endpoint_selection_audit.json、endpoint_low_audit.json和locked_manifest_regeneration.json。
