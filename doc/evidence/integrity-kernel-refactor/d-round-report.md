# D 轮（最终验收）报告 2026-09-22

本报告是路线图工作包 D 的收官证据：§10 测试矩阵（order=1/2）、§11 性能与资源
实测、清理审计、§12 合并门逐条自查、6 个 NOT_RUN 终局处置。命令与产物清单见
§D-6；面向评审的收束回答见 `final-review.md`。

起点：`HEAD=3550a38`（W2 证据提交；代码 `09a71e4`）；工作树干净（仅未跟踪
路线图文档）。本轮的代码提交为 `wip(D)`（SHA 见 `validation-report.json` 的
`run_sha` 与 `hashes-C1C2.txt`）。

## D-0 Stage 0：起点、事件 #8、manifest 对齐

**D-0.1 第八次外部格式化事件（补记；非开发方自查）**

| 项 | 记录 |
|---|---|
| 时间 | 2026-09-22 13:06 |
| 范围 | 9 个文件（`publication_identity.{hpp,cpp}`、`types.hpp`、`integrity_monitor.cpp`、`fde_manager.{hpp,cpp}`、`run_logger.cpp`、`candidate_replay.cpp`、`test_integrity_v2.cpp`） |
| 判定 | 指挥方 token 多重集复核：**8/9 严格等价**，`candidate_replay.cpp` 一处 `case '\\'` 展开 artifact（零语义内容） |
| 处置 | 指挥方已 `git restore` 回退；本轮开始时复核 `git status` 仅剩未跟踪路线图文档（工作树 = `3550a38`） |
| 归属 | 非开发方自查（检测/判定/回退均由指挥方完成） |

防复发规则沿用 runbook §20 S0-3（无任何启用的自动格式化路径；工具式 diff 先做
`--ignore-all-space` + token 多重集判别；本 agent 收尾不做任何格式化）。

**D-0.1b 事件 #8 的第二波（2026-09-22 14:22，补记）**

| 项 | 记录 |
|---|---|
| 时间 | 2026-09-22 14:22（`wip(D)` amend 14:04 之后） |
| 范围 | `wip(D)` 提交涉及的 9 个文件：`apps/` 5 个、`candidate_replay.cpp`、`test_integrity_reference.cpp`、`test_history_fault_parameterization.cpp`、`test_integrity_v2.cpp` |
| 判定 | 收尾审计（本 agent）token 多重集复核：**8/9 严格等价**；`advisor_paired_benchmark.cpp` 一处 CSV 表头长字符串换行 artifact（邻接字面量拼接，产物字符串逐字节不变，零语义） |
| 处置 | 本 agent `git restore -- apps src test` 回退；回退后 `git status` 仅剩证据文档修改与未跟踪文件（代码树 = `8584396`） |
| 归属 | 格式化非本 agent 动作（本轮无任何开发侧格式化命令）；检测/判定/回退由本 agent 在收尾审计完成 |

**D-0.2 validation-manifest 对齐**：W2 新增 pipeline 用例按 OUT-01..03 的同一机制
（`test_publication_wiring` + gtest filter）登记：OUT-04 `PipelineAttemptCarriesProductionIdentity`、
OUT-05 `FrozenDataIsUnavailableBeforeHeavyFde` + `WatchdogClauseSemanticsAreExplicit` +
`BackwardsClockIsRefusedLikeInputValidation`、OUT-06 `ControllerTriStateAndAtomicChain`、
OUT-07 `StatelessMonitorPathIsExplicitlyUnprotected`。同时把 CFG-03/GEO-03/MOD-01/MOD-03
从 NOT_STARTED 补测为 MAPPED（新用例，见 §D-1.3），MOD-04/MOD-05 终局为**文档化
NOT_IMPLEMENTED**（见 §D-5）。item 计数 56 → 60（57 MAPPED + 1 MAPPED_PARTIAL +
2 NOT_STARTED）。

**D-0.3 起点基线**：全量套件 **416/0/0**（`raw/run_tests_d_stage0.log`；与 W2 同口径）。

## D-1 §10 矩阵（order=1）

**D-1.1 runner 与结果**：`UWB_IMU_PL_VALIDATION_SHA=<wip(D)> python3
tools/integrity/run_validation.py --all` → **58 PASS / 0 FAIL / 2 NOT_RUN**
（60 项；日志 `raw/validation_d_final.log`，报告 `validation-report.json`）。
唯二 NOT_RUN = MOD-04/MOD-05（文档化 NOT_IMPLEMENTED，见 §D-5）。
终值另附全量套件 **424 tests / 0 errors / 0 failures**（`raw/run_tests_d_final.log`；
424 = 416 + 4 个新用例 × gtest 双计数口径）。

**D-1.2 矩阵逐条状态**（`validation-report.json.items` 为准；下表为家族汇总，
全部条目在本轮 `--all` 中实际执行）：

| 家族 | 条目 | 结果 |
|---|---|---|
| CFG | CFG-01, CFG-02, CFG-03 | PASS（CFG-03 本轮补测） |
| GEO | GEO-01..05 | PASS（GEO-03 本轮补测） |
| NUM | NUM-01..04 | PASS |
| MOD | MOD-01..03 | PASS（MOD-01/MOD-03 本轮补测）；MOD-04, MOD-05 = NOT_RUN（文档化 NOT_IMPLEMENTED） |
| COV | COV-01..04 | PASS |
| HIS | HIS-01..06, HIS-M1/C1/V1/X1/X2 | PASS |
| DET | DET-01..04, DET-CH | PASS（DET-01 为 MAPPED_PARTIAL，按原口径） |
| RSK | RSK-01..03 | PASS |
| FDE | FDE-01..05 | PASS |
| OUT | OUT-01..07 | PASS（OUT-04..07 本轮登记） |
| PRV | PRV-01..06 | PASS |
| GAT/DOM | GAT-01, DOM-01 | PASS |

**D-1.3 本轮补测的四个用例**（roadmap §10 缺口收口，全部新增测试而非改写）：

| ID | 用例（文件） | 断言摘要 |
|---|---|---|
| CFG-03 | `ReferenceFixture.StartupPassDoesNotMaskRuntimeGeometryDegradation`（`test_integrity_reference.cpp`） | 同一固定四锚几何在 (0,0,1) rank-3 有效、在 (0,0,0) rank-2 亏秩 ⇒ 运行时窗口 `model_valid=false`（`window state is rank deficient`）；每个窗口由 `finalizeIntegrityWindow` 重新验证，启动检查不被继承 |
| GEO-03 | `ReferenceFixture.Geo03KGreaterThanNuIsClassifiedByTheKernelNotByCounts` | m=4/rank=2（ν=2）、K=4>ν 字典：含危险盲方向（`G·d4≠0`）时核增益 >0.1 被识别；移除危险模式后 K=3>ν 仅含无害盲方向（`G|kerZ=0`）⇒ 有限 bound；K/列数不构成拒绝依据 |
| MOD-01 | `HistoryFaultParameterization.Mod01UwbDropoutAndNonUniformTimestampsKeepTheDeclaredSupport` | epoch 8 删除一个锚的测量 ⇒ 该 (epoch,anchor) 出现消失且 `material_gap=0`（不臆造模板）；epoch 9 采样 +3ms（< `max_time_skew_s`）⇒ time-linear 列与真实时刻 oracle 逐位一致；支持到达窗口边界 |
| MOD-03 | `HistoryFaultParameterization.Mod03IntervalInteriorOnsetCannotReuseTheWholeIntervalTemplate` | 整区间 FD oracle 与解析模板一致（≤1e-4 阳性对照）；后半区间 onset 与整区间模板相对差 >20%、幅值比 ∈(0.2,0.8) ⇒ 非整区间不得复用完整 bias Jacobian |

**D-1.4 order 维度说明**：矩阵条目为组件级自配置测试；双 order 覆盖由三处共同
给出——(a) 条目内部显式覆盖（CFG-02 `FaultManifest.*`、GEO-05、FDE-05、RSK-02、
DET-02/03 等以合成 order=2 字典构造）；(b) `IntegrityV2FaultModel.GeneratesCompactUwbModesAndEveryImuAxis`
直接断言 `double_faults_enabled` 下的 9×Na 双故障假设族；(c) 端到端 order=2
场景批次（§D-2）。runner 的命令不消费仓库配置，故不为两个 order 各造一份
"假报告"——差异证据来自 (a)(b)(c) 的实际执行记录。

## D-2 §10 矩阵（order=2：双故障族配置开启）

**D-2.1 配置输入**：`configs/research-order2-dev.yaml`（证据目录内，sha256
`18e8f055d6568a6512a6ec856d8039c76868c34e76d65594254a320803735069`）。与
`config/realtime_uwb_imu_pl_research.yaml`（sha256 `aa38f4a6…`）逐行等价，仅：
`double_faults_enabled: false→true`、`max_fault_order: 1→2`、`manifest_path` 相对
路径按目录层级重写。产品唯一配置入口不变。

**D-2.2 批次结果**（7 场景，`/tmp/uwb_imu_pl_d_20260922/order2/`；全部
`validate_run_schema.py` = PASS `uwb-imu-pl/v5`）：

| 场景 | attempts | `effective_fault_cardinality` | 双故障假设峰值（accel/gyro） | 有限 PL | `publication_protected` |
|---|---|---|---|---|---|
| A_nominal | 30 | 2 | 5280 / 5280 | 30/30 | 0 |
| C_uwb_fde | 30 | 2 | 5280 / 5280 | 24/30 | 0 |
| D_imu_bridge | 30 | 2 | 5280 / 5280 | 24/30 | 0 |
| E_union | 30 | 2 | 5280 / 5280 | 24/30 | 0 |
| F_ramp_unmonitorable | 30 | 2 | 7920 / 7920 | 30/30 | 0 |
| G_continuous_rejection | 45 | 2 | 3888 / 3888 | 5/45 | 0 |
| HIP_60 | 60 | 2 | 1728 / 1728 | 5/60 | 0 |

`max_hypotheses=10796`（E_union 汇总行）对比 order=1 的 236/帧：双故障族真实
展开；`mode_dense_allocations=0`、`all_mode_gram_columns=0`（compact 路径在
order=2 仍成立）。

**D-2.3 发布门对照（order=1 vs order=2）**：

| 场景 | order=1 `ADMISSIBLE/REFUSED` | order=2 `ADMISSIBLE/REFUSED` |
|---|---|---|
| A_nominal | 30 / 0 | 30 / 0 |
| C_uwb_fde | 24 / 6 | 24 / 6 |
| D_imu_bridge | 24 / 6 | 24 / 6 |
| E_union | 24 / 6 | 24 / 6 |
| F_ramp_unmonitorable | 30 / 0 | 30 / 0 |
| G_continuous_rejection | 5 / 40 | 5 / 40 |
| HIP_60 | 17 / 43 | 5 / 55 |

唯一的 order 差异：HIP_60 的早期尝试在 order=2 下多 12 次 REFUSED（缺失清单
相同：`risk_proof_id;protection_level_m`）；这是身份门的保守侧（fail-closed）
扩大，不是覆盖缩水——两档合计发布保护输出仍为 0（`publication_protected=0`，
与 W2 口径一致：`formal_eligible=false` 决定"显式未保护"）。

## D-3 §11 性能与资源实测（不换口径）

**D-3.1 口径与环境**：

* 被保护量/轨迹：`realtime_performance_benchmark` 内部 figure-eight（320 epochs
  请求）+ dev-runner 7 场景（30–60 epochs）；配置 = research 配置（order=1）或
  §D-2.1 的 order=2 变体；20 Hz replay，IMU 200 Hz。
* 原始逐帧 trace：`timing.csv`（stage × wall_ms，逐帧取样，**不用组件中位数求和**）；
  汇总工具 `tools/perf_trace_summary.py`（nearest-rank p50/p95/p99/max，
  `cold`=epoch≤100 启动段 / `warm`=其后，拒绝/边缘化/报警帧由 events/integrity
  表联立）。
* 环境声明：18 核工作站；测量期间并行运行 2 个 226-epoch H 运行与 1–3 个基准
  变体（合计 3–6 个单线程负载进程）。所有数字为该负载下的实测；40 ms 结论对
  此量级的负载噪声稳健（相差 2–3 个数量级）。
* 归因：D 轮代码提交 `e66d245` 只改 `apps/`（离线 wall 政策）、`test/`（四个补测
  用例）与 `doc/adr/`；`src/`/`include/` 未动 ⇒ 场景批次与基准的数值行为归属
  `e66d245` 的库层，与 W2 库（`09a71e4`）逐字节一致。
* 40 ms = roadmap §11.2 待验证目标（"正常受保护输出期限"）。

**D-3.2 dev-runner 场景轨迹（order=1 / order=2，core_total，单位 ms）**：

| 场景 (n) | order=1 p50 / p95 / p99 / max | miss>40ms | order=2 p50 / p95 / p99 / max | miss>40ms |
|---|---|---|---|---|
| A_nominal (30) | 39.7 / 140.5 / 152.7 / 152.7 | 50% | 244 / 475 / 497 / 497 | 87% |
| C_uwb_fde (30) | 40.8 / 132.1 / 158.8 / 158.8 | 53% | 224 / 391 / 442 / 442 | 83% |
| D_imu_bridge (30) | 41.9 / 136.7 / 141.9 / 141.9 | 53% | 221 / 439 / 737 / 737 | 83% |
| E_union (30) | 40.9 / 117.6 / 126.5 / 126.5 | 53% | 250 / 406 / 406 / 406 | 87% |
| F_ramp_unmonitorable (30) | 44.3 / 150.2 / 163.7 / 163.7 | 57% | 342 / 568 / 576 / 576 | 90% |
| G_continuous_rejection (45) | 9.4 / 13.3 / 21.0 / 21.0 | 0% | 37.1 / 92.1 / 117.4 / 117.4 | 24% |
| HIP_60 (60) | 62.4 / 78.3 / 91.5 / 91.5 | 87% | 55.9 / 67.7 / 71.0 / 71.0 | 93% |

结论：即使在 30–60 epoch 的短轨迹上，p99 也只在 G（大量拒绝、无候选执行）下
低于 40 ms；order=2 使 p50 上升约 5–6×。

**D-3.3 长轨迹基准（figure-eight；alarm 与 order=2 已完成全程，nominal 至提交时
仍在运行，覆盖至 e199）**：

| 变体（请求长度） | 启动段 p50/p95/p99/max (ms) | 成熟段 p50/p95/p99/max (ms) | miss>40ms（成熟段） |
|---|---|---|---|
| nominal (320；运行中，含至 e199) | 752.87 / 8369.98 / 10129.10 / 10211.10 | 50618.30 / 109147.00 / 131536.00 / 131536.00 | 100% |
| alarm（强制候选压力，260；**完成**） | 1304.06 / 2865.60 / 3029.79 / 3547.09 | 2536.13 / 3164.11 / 6046.89 / 6169.55 | 100% |
| order=2 (150；**完成**) | 1473.29 / 13117.80 / 18767.80 / 21765.50 | 28802.70 / 50324.70 / 52330.60 / 52330.60 | 100% |

`core_total` 在 **epoch 13** 即越过 40 ms（nominal）。

**D-3.3b 资源与队列（同一 trace）**：

* 峰值内存（`/proc/<pid>/status` VmHWM 实测）：alarm 完成时 **122 MB**；order=2
  完成时 **297 MB**；nominal 运行中 **493 MB**（仍在增长）；H_mature_union
  两侧 391/437 MB（e163/e188）。
* alarm 完成时：`processed=260, committed=60, rejected=200`，FDE 状态
  `AMBIGUOUS_UNAVAILABLE 188` / `NO_VALID_CANDIDATE 72`；candidate 逐候选 wall
  p50/p95/p99/max = 0 / 22.83 / 28.31 / 70.91 ms（**候选路径不是瓶颈**）。
* nominal 队列/时效（attempts 表）：`pending_duration_s` p50=max=0.05 s（20 Hz
  节拍，无排队积压）；`state_age_s` = 0；离线回放下 `watchdog_sensor_lag_ns`
  p50≈27 s（回放慢于实时，由构造决定，不代表在线指标）。
* **提交 vs 拒绝的对照（新发现）**：相同 260-epoch 轨迹上，alarm 变体持续拒绝
  （200/260 未提交）时单帧维持 ~2.5 s；nominal 变体持续提交时边界随 epoch 累积、
  单帧增长至 >50 s。即 §D-3.4 的热点增长由**成功提交的边界内容**驱动，而非
  候选/FDE 计算。这条对照同时说明：以拒绝换取性能会缩小可用输出，本轮明确
  **不作为**优化手段。

**D-3.3c 帧类清单（逐帧可判定字段，order=1 场景）**：

| 类 | 判定字段 | 实测 |
|---|---|---|
| 启动 | `cold_warm=cold`（epoch ≤100） | 全部短场景（30–60 ep）整体处于启动段；长轨迹见 §D-3.3 |
| 成熟 | `cold_warm=warm` | 基准 e101+（见 §D-3.3） |
| 报警/拒绝 | `UWB_REJECT` 事件 / `batch_committed=0` | D 3/30、E 5/30、G 36/45、HIP 42/60；A/C/F 0（`fde_status`：`NO_VALID_CANDIDATE`/`AMBIGUOUS_UNAVAILABLE`） |
| 边缘化 | `marginalization_count>0` / `FIXED_LAG_MARGINALIZE` 事件 | 短场景 0（`fixed_lag=200` 未达）；由 H/基准长轨迹覆盖（完成后更新） |
| 重线性化/缓存失效 | `factor_block_cache_invalidations>0` | 几乎每帧（窗口滑动导致的版本失效），如 A 29/30、HIP 59/60 |
| 回退 | `generic_fault_gram_fallback>0` | 0（compact 路径全程成立） |
| 恢复/重初始化 | `controlled_reinitialization_required` 非零 / phase 事件 | 0（机制存在但短场景未触发；相位字段常显 `RUNNING` 为状态机常态） |

**D-3.4 热点（实测，非预设）**：成熟帧中 `window_boundary_provenance` 占
99.5%（e106：11.54 s / 11.62 s；其余全部阶段合计 ~80 ms）。该阶段 =
C1 历史参数化 + 摘要提取（`incremental_estimator.cpp:1770` 的
`preparation_start→factor_start` 区段，含 `planHistoryFaultParameterization` 与
`buildHistoryFaultSummary`）。增长量实测：

* `boundary_rows` 每 epoch +≈23（e20 444 → e115 2652；H 运行 e188 达 4331），
  `emitted_rows`/`nu_perp` 同步增长；`fault_columns`（q_hist）恒为 220（horizon
  稳定在 10 epochs）——增长来自 fixed-lag 边缘化尚未启动（`fixed_lag_epochs: 200`）
  时边界因子逐 epoch 累积。
* 时间对 `boundary_rows` 的实测幂律 ≈ r^4（e30 97.7 ms/697 行 → e60 1500 ms/1387
  行 → e90 6959 ms/2077 行 → e114 26293 ms/2629 行）。
* 结论：**40 ms 目标在本配置下不达标**（超出 2–3 个数量级；目标仅在轨迹开始
  约 12 个 epoch 内成立）。这属于设计级修复（增量摘要更新/边界容量控制），
  涉及历史覆盖与风险账本语义；本轮不做任何语义冒险，记录为 blocker。热点
  不属于 symbol/index/hash、PIM 映射、概率反演或临时分配类别（实测占比可忽略，
  见 e106 分解），故未实施投机优化。

**D-3.5 缺口记录：history 容量键部分未执行**。`history.max_summary_rows` 与
`history.max_perp_rows` 被严格解析并导出，但 `evaluateHistoryFaultCapacity()`
只接收 `max_fault_columns`（调用点：`incremental_estimator.cpp:1405-1409`）。
实测成熟运行 `capacity_ok=1` 而 `emitted_rows=1069`、`nu_perp=1054`（均 >
配置 512）。影响：声明的行数/perp 容量不产生 REFUSE 动作；运行保持可用且
数值正确（保守侧不变），但配置承诺的 fail-closed 边界未生效。本轮按"语义零
改动"原则只记录，不修；修复需 Q/D 决策（执行 REFUSE 会改变成熟期可用性）。

**D-3.6 离线驱动 wall 政策显式声明（W2 §7.2 续跑项收口）**：五个离线应用
（`realtime_performance_benchmark`、`advisor_paired_benchmark`、
`sequential_detector_history`、`gate_d_development`、`integrity_round2_scenario`）
改为显式传 `offlineReplayPublicationLimits()`（`apps/r0_r1_development.cpp` 已于
W2 传入）；生产默认（2 s 陈旧 / 4 s wall 超时）与配置合同不变。基准输出中
`watchdog_valid=1`、`sensor_delta=50 ms`、`sensor_lag≈0.5 s`（离线回放慢于实时
为构造使然）。

**D-3.7 有限有效 PL 比例（成熟段）**：dev-runner 场景（order=1）：A 30/30、
C/D/E 24/30、F 30/30、G 5/45、HIP 17/60。benchmark nominal 全段 0（其合成流
在 FDE 上持续 `NO_VALID_CANDIDATE`——保守不可用，非静默降级；该运行仅用于
计时口径，完整性结论以场景矩阵为准）。**性能 PASS 不写成完整性认证 PASS**：
`formal_eligible=false` 全轮保持。

## D-4 清理审计与 §12 合并门自查

**D-4.1 清理审计（结论：无可安全删除项；被替代路径已处受控状态）**：

| 候选 | 审计结论 | 处置 |
|---|---|---|
| dense oracle（`dense_oracle.cpp`） | 在役：`rank_update_kernel`/`candidate_replay`/`integrity_monitor` 引用；同时是独立测试对照 | 保留 |
| method B（`methodBCandidatePrior`） | 在线被 loader 明确拒绝（`enable_method_b=true` → 启动失败）；离线 API 由 `test_realtime_incremental` 覆盖 | 保留（诊断能力），文档化 |
| `evaluateSnapshot/evaluateConditional` 无状态路径 | 仅 `snapshot_integrity_sweep`（开发 app）与测试使用；W2 已使其显式未保护 | 保留 |
| eager FDE 动作 | B4 已惰性化（`action_entities_constructed` vs `deferred` 计数器持续为 0/正） | 无需清理 |
| LLT/spectral 分解 | 已降级为计数回退（`base_llt`/counters 导出） | 保留（安全网） |
| all-mode 稠密展开 | compact 路径默认（`all_mode_gram_columns=0`）；dense 展开为计数回退 | 保留 |

**D-4.2 ADR 更新**：`doc/adr/0001/0002/0003` 各追加 "D round status (2026-09-22)"
小节（范围/数值合同/边界基无改动；记录性能 blocker 与容量键缺口；Gate J 未完成，
`formal_eligible=false`）。

**D-4.3 唯一配置入口**：产品入口仍为 `config/realtime_uwb_imu_pl_research.yaml`
（+ `config/integrity_fault_manifest.yaml` 严格清单）。order=2 变体位于证据目录
（§D-2.1），仅作验收输入，digest 已记录。

**D-4.4 §12 合并门逐条自查**（以 §12 定稿的 8 门为准）：

| 门 | 判定 | 证据 / 缺口 |
|---|---|---|
| 代码与配置门 | 通过 | base/run SHA 绑定（`validation-report.json`：`base_sha`、`run_sha=wip(D)`）；有效配置/清单可复现（`resolved_config.yaml`、`fault-manifest-resolved.yaml`、config digest） |
| 代数正确性门 | 通过 | 解析/独立参考（GEO-01..05、A4 fixtures）、NUM-01..04、PRV-04/05（oracle JSON 以同代材料重生成，见 §D-4.6）；分类正确性（GEO-02 危险零空间、MOD-03 模板边界） |
| 覆盖门 | 通过 | COV-01..04（exact/envelope/拒绝分类）+ 场景 `coverage_*` 列；组合交叉项由 order=2 批次实测（双故障假设展开、`fault_cross_blocks` 全量计数） |
| 风险门 | 通过 | RSK-01..03、`risk_ledger_*` 列（无隐藏零、重叠无红利、先验拼接） |
| 历史门 | 通过（含记录缺口） | HIS-01..06/M1/C1/V1/X1/X2（等价、lineage、容量 REFUSE 语义、重线性化失效）；缺口：`max_summary_rows`/`max_perp_rows` 未执行（§D-3.5，已记录未修） |
| FDE/发布门 | 通过 | FDE-01..05、OUT-01..07（三态、原子链、看门狗、无证明不复用）；`publication_protected=0` 的显式未保护口径全轮一致 |
| 实时与资源门 | **不通过（blocker）** | 实测见 §D-3.2/3.3/3.4：40 ms 目标在成熟期差 2–3 个数量级；热点 = `window_boundary_provenance`。未缩覆盖、未改阈值；修复属设计级（增量摘要/边界容量），列为后续工作 |
| 模型证据门 | 未完成（如实） | noise/先验/桥接包络仍为 `IMPLEMENTED_UNVERIFIED`；Gate J（独立公式评审、真实数据校准）未做；`formal_eligible=false` 保持 |

**D-4.5 未通过的部署模型证据清单**：真实数据校准（无数据集）、独立公式评审、
动态桥接包络的部署级统计证据、IMU 原始样本级故障的真实噪声模型验证。

**D-4.6 oracle 产物重生成（W2 §12/§13 缺口收口）**：

* 方法：以"同代材料"重生成——A/C/G 的冻结 bin 由 D 轮代码重导出
  （`UWB_IMU_PL_REPLAY_EXPORT_DIR` + `UWB_IMU_PL_REPLAY_ATTEMPTS`，此机制首次
  用于证据刷新，写入 `raw/replay_p2/<scenario>/exports/`），CSV 侧取同代运行
  （`/tmp/uwb_imu_pl_d_20260922/export/`）；H 的 CSV 侧取 226-epoch 运行
  （`/tmp/uwb_imu_pl_w2_20260922/new/`）。运行根经符号链接合并，两个工具以
  `UWB_IMU_PL_ORACLE_RUNS` 指定。
* 发现并修复一个真实缺陷（**W2 引入**）：`candidate_replay` v6 写入器把
  `factor_inventory` 段落成 v4/v5 专属（v6 被排除），与"v6 = v5 布局 + 3 个
  removal provenance 字段"的文档契约不符，导致所有按 v5 布局实现的 v6 读取器
  错位（实测：python 读取器 `trailing bytes: 4273915 at 1070891`；修复前后
  A#30 导出 5,344,806 → 5,348,690 字节，差额即该段；注意 D 代导出整体
  5.3 MB > P2 代 1.3 MB，属 C1 边界内容增长）。修复：v6 也写该段
  （`wip(D)` 提交 `8584396`），并把往返测试强化为断言 factor_inventory 逐字段
  往返（旧代码下必失败）。
* **新增客观陈旧门（本轮工具改动）**：保留的 P2 代冻结 bin 与 D 代运行不属
  同代材料，跨代比较数字无定义。两个 oracle 工具现以**身份绑定**判据判定：
  设 bin 内 `identity_digest` ≠ 生产运行 `diagnostic_snapshot_identity.csv`
  同 attempt 行的 `identity_digest`，即判 `stale bin`，相关跨边界检查
  （O0/O1/O2/O5、O8a–d）转 **NOT_RUN** 并附精确重导出配方
  （`UWB_IMU_PL_REPLAY_EXPORT_DIR` + `UWB_IMU_PL_REPLAY_ATTEMPTS`，§D-6）；
  bin 内部自洽检查（O3/O4/O6/O7、O8e–i）不依赖生产行，保持实算。判据客观、
  不循环（不以"数字是否对得上"反推门是否该开）。**未对陈旧 bin 做任何数字
  比较，也未把陈旧 bin 的失败改记为通过。**
* 结果（最终）：`square-root-oracle.json` = **41 PASS / 0 FAIL / 9 NOT_RUN**
  （9 = 5×O8j 既有内部未覆盖项 + H#201 的 O8a–d 陈旧门；H#201 的 O8e–i
  实算 PASS）；`oracle-results.json` = **35 PASS / 0 FAIL / 8 NOT_RUN**
  （8 = 3 个 P2 期已剪枝 bin 整案 + G#10 的 O5「生产行无
  onset_epoch==attempt」+ H#201 的 O0/O1/O2/O5 陈旧门；H#201 的
  O3/O4（bin 内部）、O6/O7（CSV 侧）实算 PASS）。H#201 的 bin 无法低成本
  重导出（需以导出环境变量重跑 226-epoch 全场景 ≈2.5 h），故按门记
  NOT_RUN 并保留配方，**未以陈旧材料充数**。
  `source_revision` 绑 `8584396`。
* 纪律：两个 JSON 由同代材料生成后即备份固化；验证 runner 的 PRV-05/06 执行会以
  默认 `/tmp` 源覆盖它们（§12/§13 既知副作用）。D 轮最终版本由（含门的）工具
  直接重跑生成，哈希基于该版本；如需再跑验证，之后必须重生成。`prune-log.md`
  中记录的旧 bin 哈希描述的是被替代的 P2 代导出；当前哈希记录在重生成的两个
  JSON 内。
* 存储：D 代冻结 bin 单枚 0.4–5.3 MB，按「大产物 /tmp」纪律存放于
  `/tmp/uwb_imu_pl_d_20260922/replay_p2_store/`，`raw/replay_p2` 为其符号链接
  （raw/ 合计 4.9 MB ≤15 MB；工具经符号链接读写无差别，已重跑两工具复验）。

## D-5 6 个 NOT_RUN 的终局处置

| ID | 终局 | 记录位置 |
|---|---|---|
| CFG-03 | ✅ 补测（新用例，MAPPED，PASS） | manifest 注记 + §D-1.3 |
| GEO-03 | ✅ 补测（新用例，MAPPED，PASS） | 同上 |
| MOD-01 | ✅ 补测（新用例，MAPPED，PASS） | 同上 |
| MOD-03 | ✅ 补测（新用例，MAPPED，PASS） | 同上 |
| MOD-04 | 📄 NOT_IMPLEMENTED：`same_device_multiaxis`/`two_uwb` 为 `unsupported_families`，启用即启动拒绝（fail-closed）；多轴共因交叉项无实现。复现：`test_fault_manifest --gtest_filter=ConfigMigration.StartupRejectsManifestWithEnabledUnsupportedFamily` 或加载启用该族的清单 | manifest note |
| MOD-05 | 📄 NOT_IMPLEMENTED：`amplitude_model=bounded_set` 一律标记 NOT_IMPLEMENTED 并启动拒绝（集合支撑界与"集合失效"风险账本无实现）。复现：`test_fault_manifest --gtest_filter=FaultManifest.BoundedAmplitudeSetMustBeMarkedNotImplemented` | manifest note |

## D-6 复现命令与产物清单

```bash
B=/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl      # 仓库根
E=$B/doc/evidence/integrity-kernel-refactor
WS=/home/mint/ws_fusion_uwb

# 套件与验收调度（run_sha = wip(D) 代码提交）
cd $WS && source /opt/ros/noetic/setup.bash && source devel/setup.bash
catkin build uwb_imu_pl -j2 && catkin run_tests uwb_imu_pl > $E/raw/run_tests_d_final.log 2>&1
cd $B && UWB_IMU_PL_VALIDATION_SHA=<wip(D)> python3 tools/integrity/run_validation.py --all \
    > $E/raw/validation_d_final.log 2>&1

# order=2 批次（7 场景；配置 digest 见 §D-2.1）
CFG2=$E/configs/research-order2-dev.yaml
SCN=$B/config/r0_r1_development_scenarios.yaml
O=/tmp/uwb_imu_pl_d_20260922/order2
for spec in "A_nominal 30" "C_uwb_fde 30" "D_imu_bridge 30" "E_union 30" \
            "F_ramp_unmonitorable 30" "G_continuous_rejection 45"; do
  set -- $spec; mkdir -p $O/$1
  UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 $WS/devel/lib/uwb_imu_pl/r0_r1_development \
      $CFG2 $O/$1 $2 $1 $SCN > $O/$1/stdout.log 2>&1
done
UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 $WS/devel/lib/uwb_imu_pl/r0_r1_development \
    $CFG2 $O/HIP_60 60 HIP_history_crossing_fault $SCN > $O/HIP_60/stdout.log 2>&1

# §11 基准（三变体）与 trace 汇总
P=/tmp/uwb_imu_pl_d_20260922/perf
/usr/bin/time -v $WS/devel/lib/uwb_imu_pl/realtime_performance_benchmark \
    $B/config/realtime_uwb_imu_pl_research.yaml $P/nominal 320
UWB_IMU_PL_BENCHMARK_FORCE_ALARM=1 /usr/bin/time -v $WS/devel/lib/uwb_imu_pl/realtime_performance_benchmark \
    $B/config/realtime_uwb_imu_pl_research.yaml $P/alarm 260
/usr/bin/time -v $WS/devel/lib/uwb_imu_pl/realtime_performance_benchmark \
    $CFG2 $P/order2 150
python3 $E/tools/perf_trace_summary.py $P/nominal   # 及各场景目录

# 运行 schema 校验
python3 $B/tools/validate_run_schema.py <run_dir>
```

```bash
# 冻结 bin 重导出（同代材料；按 attempt 过滤）+ oracle 重生成
E=$B/doc/evidence/integrity-kernel-refactor
CFG=$B/config/realtime_uwb_imu_pl_research.yaml   # order=2 时替换为 $E/configs/research-order2-dev.yaml
SCN=$B/config/r0_r1_development_scenarios.yaml
O=/tmp/uwb_imu_pl_d_20260922/export
for spec in "A_nominal 30 30" "C_uwb_fde 30 25,26" "G_continuous_rejection 45 10"; do
  set -- $spec; mkdir -p $O/$1 $E/raw/replay_p2/$1/exports
  UWB_IMU_PL_REPLAY_EXPORT_DIR=$E/raw/replay_p2/$1/exports   UWB_IMU_PL_REPLAY_ATTEMPTS=$3 UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1       $WS/devel/lib/uwb_imu_pl/r0_r1_development $CFG $O/$1 $2 $1 $SCN       > $O/$1/stdout.log 2>&1
done
R=/tmp/uwb_imu_pl_d_20260922/oracle_runs       # 合并根：A/C/G -> export；H -> 226-epoch 运行
UWB_IMU_PL_ORACLE_RUNS=$R python3 $E/tools/context_oracle.py   # square-root-oracle.json
UWB_IMU_PL_ORACLE_RUNS=$R python3 $E/tools/oracle_compare.py   # oracle-results.json
```

产物：`validation-report.json`、`raw/validation_d_final.log`、
`raw/run_tests_d_final.log`、`configs/research-order2-dev.yaml`、
`tools/perf_trace_summary.py`、重生成的 `square-root-oracle.json` /
`oracle-results.json`（含 bin sha）、场景与基准 trace
（`/tmp/uwb_imu_pl_d_20260922/`，大产物不入库）。H_mature_union 两侧 226-epoch
对照已完成：`publish_diff.py` = **exit 0，零已发布量差异**（226 行；身份门
224 `ADMISSIBLE` / 2 `REFUSED`、`protected=0`、`refused=0`；两侧汇总计数器
逐项相同）；详情与日志见 `fde-publication.md §7.4` 与 `runbook §22` 的 D 节。
