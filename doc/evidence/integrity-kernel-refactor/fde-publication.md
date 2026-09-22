# C4（M3）：原子提交与发布身份

范围：roadmap §8.6。合同：现有阈值不动；`formal_eligible=false`；不 push。
选择风险/保证组（§8.4/§8.5）见 `fde-post-selection.md`（M2）；本文件覆盖 M3（§8.6）。

---

## 1. 提交绑定（identity binding）

实现：`publication_identity.{hpp,cpp}` — `PublicationIdentity` 携带 §8.6 要求的全部身份量：

| 字段 | 含义 | 检查方式 |
|---|---|---|
| `snapshot_id` / `state_solution_id` | 快照与状态解身份 | 必须匹配证书；解变化需**同时间点中心偏移界**重新绑定 |
| `history_summary_id` | 历史摘要身份（C1 的 `version_digest`） | 必须匹配（摘要变化 ⇒ 证书不可复用） |
| `manifest_digest` | 配置/清单摘要 | 必须匹配 |
| `health_state` | 健康状态 | 必须匹配 |
| `detector_ids` | 检测器集合（C2 的 `detector_id`） | 集合逐项匹配 |
| `risk_proof_id` | 风险证明身份 | 必须匹配 |
| `protection_level_m` | 已认证的 PL | 随身份发布 |
| `position_reference` | 参考点（body 原点 vs 天线相位中心等） | **必须匹配**，不得由偏移界修补 |
| `timestamp_ns` | 时刻 | 不同时刻 ⇒ 需**时间传播证明** |
| `frame_id` | 坐标系 | **必须匹配**，不得由偏移界修补 |
| `certificate_id` | 证书身份 | 0 ⇒ 直接拒绝 |

判定（`checkPublicationIdentity`）三态：`admissible` / `same_time_rebind`（有中心偏移界）/
`requires_propagation`（不同时刻，需 `PropagationProof`）；**两估计值之差不是证明**。

## 2. 状态机（atomic commit）

合法迁移（其余一律拒绝，`advancePublicationState`）：

```
READY -> EVALUATING -> PROTECTED | UNAVAILABLE
UNAVAILABLE -> FDE_EVALUATING -> VERIFIED_CANDIDATE -> ATOMIC_COMMIT -> PROTECTED
PROTECTED -> EVALUATING | UNAVAILABLE        （随时可降级）
```

拒绝的例子（测试断言）：`UNAVAILABLE -> PROTECTED`（跳级）、`VERIFIED_CANDIDATE -> PROTECTED`
（绕过原子提交）。完整恢复链在测试中逐步走通。

## 3. 时间与输出契约

| 要求 | 实现/断言 | 证据 |
|---|---|---|
| 受保护/未保护在契约中区分 | `protected_output` 是身份字段；`PublicationIdentity` 差异可检 | OUT-02 |
| watchdog 用单调壁钟、新鲜度用 sensor timestamp | `updateWatchdog`：`wall_elapsed_ns` 只在单调且未跳变的样本上累加；`sensor_lag_ns`（wall 前进 − sensor 前进）判新鲜度 | OUT-03 |
| 回放 `/clock` 跳变/暂停不得使 wall-time 统计失真 | 跳变样本**不累加** wall；跳变不触发 wall 超时；滞后量不因跳变而增长 | OUT-03（`wall_elapsed_after_jump=420000000` 与真实推进一致） |
| 队列拥塞 ⇒ 及时不可用 | 冻结数据（wall 前进、sensor 不动）⇒ `sensor_stale`（滞后 400 ms > 100 ms 门限） | OUT-03 |
| 时钟倒退（无回放标记）⇒ 直接拒绝 | `valid=false` + `moved backwards` | OUT-03 |
| 证书不得跨时间/坐标系复用 | 时间 ⇒ 需传播证明；frame/参考点/detector/manifest ⇒ 不可修补 | OUT-01/02 |

## 4. 决策差异表（对 M2 基线）

| 决策点 | M2（C3 后） | M3（C4 后） | 保守方向 |
|---|---|---|---|
| 提交前提 | 候选通过 kernel/PL/证明 | 追加**身份匹配**（快照/解/摘要/清单/健康/检测器/风险证明/参考点/时刻/坐标系） | 更保守：身份不符即不发布 |
| 后端同时间点改解 | 无规则 | 需**已证明的中心偏移界**才可重绑定（否则拒绝） | 更保守 |
| 不同时间点 | 无规则 | 需时间传播证明；**不得**用两估计之差修补 | 更保守 |
| 状态迁移 | 无显式状态机 | 白名单迁移；禁止跳级/绕过原子提交 | 更保守 |
| 过期/拥塞/时钟跳变 | 无规则 | wall 与 sensor 分离；跳变不影响 wall 统计；滞后量判新鲜 | 更保守 |

**无覆盖缩水**：新增规则只会把"原本可能发布"的路径改为"身份/证明满足才发布"，未放宽任何既有判定。

## 5. 计数

| 量 | 值 |
|---|---|
| 新增模块 | `publication_identity.{hpp,cpp}`（≈250 行） |
| 新增测试 | `test_publication_identity.cpp`：OUT-01/02/03 = **3 用例** |
| 全量测试 | **398 / 0 errors / 0 failures**（M2 后 392 + 3×2）；日志 `raw/run_tests_c3c4_baseline.log` |
| validation | `UWB_IMU_PL_VALIDATION_SHA=8c5330f --all` → **50 PASS / 0 FAIL / 6 NOT_RUN**（OUT-01..03 全 PASS）；日志 `raw/validation_c3c4.log` |
| 提交 | M3 checkpoint `wip(M3)` = `8c5330f`；证据提交见 `hashes-C1C2.txt` 头部 |

## 6. 里程碑状态

| 里程碑 | 状态 | 说明 |
|---|---|---|
| M1 = C2 | 完成（前一轮） | `c2-dual-channel.md` |
| M2 = C3 | 完成（模块 + 测试 + 证据） | `fde-post-selection.md`；未接线三项如实记录 |
| M3 = C4 | **完成（模块 + 测试 + 证据）** | 本文件；同样如实记录：模块→`integrity_monitor` 发布路径的**接入**未做（当前为模块级 API + 测试），诊断列/CSV 未加 OUT 字段 |
| W2 = C4 生产接线 | **完成（本轮）** | 见 §7：接入点/状态持有者、看门狗判定规则、诊断 v16、场景重基线差异表 |

## 7. W2：C4 接入生产发布链（2026-09-22）

### 7.1 接入点与状态持有者（显式对象，无第二状态源）

| 面 | 精确位置 | 说明 |
|---|---|---|
| 状态持有者 | `RealtimeIntegrityPipeline::publication_`（类型 `PublicationController`，`include/uwb_imu_pl/integrity/publication_identity.hpp`） | 唯一持有：发布状态机状态、最近被准入的证书、看门狗累计量；管线内没有第二处发布状态 |
| 尝试入口 | `RealtimeIntegrityPipeline::processUwbBatch(const UwbBatch&, const ClockSample&)` | 在任何重 FDE 之前调用 `publication_.beginAttempt(clock_sample)`；`processUwbBatch(batch)` 从 `steady_clock::now()` + `batch.timestamp` 构造样本（生产无回放标记 ⇒ 倒退即拒绝） |
| 发布产出 | `RealtimeIntegrityPipeline::applyPublicationGate(IntegrityOutput*, const UwbBatch&)` | 在 `processUwbBatchImpl` 返回后组装身份并调用 `publication_.finalizeAttempt(...)`，把判定写入 `IntegrityOutput::publication` |
| const 快照路径 | `IntegrityMonitor::evaluateSnapshot(..., PublicationController*)`、`evaluateConditional(..., PublicationController*)`（`*Impl` 为原实现体） | 状态机**不藏在 const 路径**：由调用方显式传入持有者；`nullptr` ⇒ `gate_executed=false`、`identity_check=NOT_EVALUATED`、显式未保护（不暗示检查通过） |
| 判定纯函数 | `checkPublicationIdentity` / `propagateToTime` / `advancePublicationState` / `updateWatchdog`（同文件） | 无状态；三态的语义与 M3 模块一致 |

**状态机**：白名单迁移，只允许 `READY→EVALUATING→PROTECTED|UNAVAILABLE`、
`UNAVAILABLE→FDE_EVALUATING→VERIFIED_CANDIDATE→ATOMIC_COMMIT→PROTECTED`、
`PROTECTED→EVALUATING|UNAVAILABLE`。`PublicationController::dropToUnavailable()` 只走这些合法路径；
`VerifiedCandidate/AtomicCommit` 在两次尝试之间不可达（提交链同步执行），观察到即保持原状态并拒绝。

**身份取材（生产现场真实值；缺一即失败封闭）**：`window:<window_id>:transaction:<transaction_id>`（FNV-1a 64）、
`linearization_version`、C1 `history_summary.version_digest`（仅 `present && valid` 时）、
实际运行的检测器 `detector_type` 标签集、W1 `selection_risk_proof_id`、`cfg.fault_manifest->digest`、
健康快照（`source:state` 序列）、`protection_level.pl_xyz_m`、`position_reference`、`timestamp`、`cfg.realtime.world_frame`。
证书 id = 上述完整身份的 FNV-1a 64；任一缺失 ⇒ `certificate_id=0` ⇒ `REFUSED` 且
`missing_identity_fields` 逐项列出（**不编造**）。字符串→数值只经 `identityHash64`（`common/integrity_identity`）这一文档化映射。

**三态处置（生产）**：

| 情形 | 处置 | 依据 |
|---|---|---|
| `certificate_id == 0` 或结构身份不匹配 | `REFUSED`：拒绝提交（显式未保护 + 缺失清单） | 失败封闭 |
| 同一时刻、解身份变化 | `REFUSED`：生产**没有**已证明的中心偏移界（"两估计之差"不是证明） | 任务合同 §8.6 |
| 不同时刻 | `REQUIRES_PROPAGATION`：**显式未保护输出**，不复用旧证书（`propagateToTime` 无证明路径） | 同上 |
| 身份可准入但平台未认证（`formal_eligible=false`，Gate J 未完成） | 状态置 `UNAVAILABLE`，未保护输出 + 原因记录 | 平台现状（不伪称受保护） |
| 身份可准入且平台可认证 | 走 `VERIFIED_CANDIDATE→ATOMIC_COMMIT→PROTECTED`（唯一受保护路径） | 本生产链当前不可达（Gate J），测试覆盖 |

### 7.2 看门狗：判定规则与"重 FDE 之前"

* 尝试入口即更新看门狗；**在进入重 FDE 之前**即可判 UNAVAILABLE 的正是两条规则：
  1. **冻结数据**：sensor 时间戳完全不动（`sensor_delta_ns == 0`）而 wall 前进、累计发散超过门限；
  2. **wall 超时**。
* 反向/非法样本（wall 或 sensor 倒退且无回放标记）⇒ `watchdog.valid=false` ⇒ 与平台既有输入校验一致地
  `throw std::invalid_argument`（不进入 FDE、不动状态）。
* **刻意不拒绝**"只是落后但仍在前进"的流：它每次都消费最新数据，拒绝它等于丢弃新鲜输入。
  实测依据：`H_mature_union`（226 epochs，离线）在"原始 OR 判定"下被拒 169/226 次
  （wall 每 epoch 推进 ~100 ms > sensor 50 ms，累计滞后 2.8 s）；改成冻结规则后 0 次拒绝。
  模块原始判定（`sensor_stale = lag>限 || delta>限`）仍原样导出，供消费方区分"没有新数据"与"数据超前"。
* 门限为新政策默认（构造参数可覆盖，未改动任何既有阈值）：`sensor_stale_limit=2 s`（=2×平台设计内
  桥接航程 `bridge.max_duration_s=1 s`）、`wall_timeout_limit=4 s`（=2×陈旧限）；场景实测见 §7.5。
* **离线回放口径**：`H_mature_union`（226 epochs，本机 ~2.3 s/epoch）在默认 wall 限下仍会因
  "相邻尝试间隔 > 4 s"触发 `wall_timeout`（实测 attempt 99/151 行），而离线回放的 wall 时钟不是实时
  期限来源 ⇒ 场景 harness（`apps/r0_r1_development.cpp`）显式传入
  `offlineReplayPublicationLimits()`（wall 超时判定按构造关闭，冻结/陈旧判定仍全量生效，策略在调用点可见）。
  **续跑项（D 包）**：其余离线驱动（`realtime_performance_benchmark` / round2 / sequential 等）仍用
  默认政策；若其单次尝试超过 4 s，会按设计拒绝该尝试——D 包做性能实测前应显式声明离线口径或收紧/放宽
  wall 政策。
* 回放 `/clock` 跳变（显式标记）不污染 wall 统计、不制造陈旧判定（模块语义；生产当前无回放标记输入，
  该项为"已接线、场景不可触发"）。

### 7.3 诊断 v15→v16（只加列）与工具同步

| 组 | 列（新增） | 位置 |
|---|---|---|
| ① 身份检查三态 + 理由 | `publication_identity_check`、`publication_identity_reason`、`publication_refusal`、`publication_missing_identity_fields`、`publication_*`（身份值 8 列）+ `transaction_opened` | `diagnostic_attempts.csv` |
| ② 发布状态机 | `publication_state_before/after`、`publication_transition`、`publication_transition_accepted`、`publication_protected`、`publication_unprotected`、`publication_gate_executed` | 同上 |
| ③ watchdog | `watchdog_valid`、`watchdog_wall_elapsed_ns`、`watchdog_sensor_elapsed_ns`、`watchdog_sensor_delta_ns`、`watchdog_sensor_lag_ns`、`watchdog_sensor_stale`、`watchdog_wall_timeout`、`watchdog_replay_jump`、`watchdog_clock_refused`、`watchdog_reason` | 同上 |
| ④ 保证组/事件类 id（C3/W1） | `selection_event_class_ids`（已计费类 id 排序串） | 同上 |
| ⑤ 选择风险证明 id（W1 并集计费） | `selection_risk_proof_id`（`eventClassId` 类 id 集 + 计费额 的 FNV 绑定；`fde_manager.cpp`） | 同上 |
| ⑥ §7.3 三个 provenance 列 | `removal_data_source`、`model_error_record`、`model_error_validated` | `diagnostic_candidates.csv` 与 `candidates.csv` |
| ⑦ §7.1 四字段 | `unit_kind`、`profile_j`、`profile_valid`（`parameter_dimension` 已有） | `hypotheses.csv` |

* 版本戳：manifest `diagnostics_schema_version`、全部 `diagnostic_*.csv` 行戳（含 square-root、history-summary）统一为 `uwb-imu-pl/gate-d-diagnostics/v16`。
* 工具同步（兼容策略**二选一，取 (a)**）：(a) 平级接受已知版本——`tools/gate_d_diagnostics.py`
  接受 v1..v16，`tools/validate_run_schema.py` 以版本化表头接受 v5/B1/v13/**v16** 变体（旧报告继续可校验）；
  (b)（未采用）只接受最新版本并迁移旧报告。
* 被看门狗拒绝的尝试是新形状：`status=WATCHDOG_REFUSED`、`transaction_opened=0`、零事务/窗口、
  恰好一个 `publication_watchdog` 阶段、不写 `transactions.csv` 行；`gate_d_diagnostics` 对该形状单独校验。
* `candidate_replay` codec：**先加显式版本标记再扩展**——写出 `frozen-candidates/v6`，v6 = v5 + 三个
  removal provenance 字段（仅在 v6 分支读写）；v1..v5 字节布局不变（旧工件的哈希不受影响），未知版本拒绝解析；
  独立 Python 读取器 `tools/replay_io.py` 接受 v4/v5/v6。**replay 哈希变化**：v6 写出的文件哈希相对 v5 必然变化
  （新增字段），归因见差异表，属格式版本升级而非数值变化。

### 7.4 发布差异表（对 `b7feb9c`；逐行实测）

命令与产物见 `runbook.md §21`（`/tmp/uwb_imu_pl_w2_20260922/{base,new}/<scenario>`；
逐行比较工具 `tools/publish_diff.py`，比较面 = `integrity.csv`/`transactions.csv`/`candidates.csv`/
`hypotheses.csv` 的**共有列** + `diagnostic_attempts.csv` 的覆盖/决策计数列）。

| 场景（epochs） | 已发布动作/状态/PL/事务 | 覆盖证书与决策计数 | 结论 |
|---|---|---|---|
| `A_nominal` (30) | **无差异**（30 行逐列相同） | **无差异** | 不变量化 |
| `C_uwb_fde` (30) | **无差异** | **无差异** | 不变量化 |
| `D_imu_bridge` (30) | **无差异** | **无差异** | 不变量化 |
| `E_union` (30) | **无差异** | **无差异** | 不变量化 |
| `F_ramp_unmonitorable` (30) | **无差异** | **无差异** | 不变量化 |
| `G_continuous_rejection` (45) | **无差异** | **无差异** | 不变量化 |
| `HIP_history_crossing_fault` (60) | **无差异** | **无差异** | 不变量化 |
| `H_mature_union` (226) | **本轮未逐行对照**（单侧 25-40 min，harness 强制 226；见下） | 未对照 | D 包续跑 |

* **无覆盖缩水（实测）**：覆盖证书与决策计数列（`coverage_*`、`kernel/post/pl_evaluated_actions`、
  `selected_actions`、`risk_ledger_*`、`primary_failure`、`all_failures`、`not_evaluated_checks`）
  在全部 7 个场景逐行相同 ⇒ 无覆盖缩水、无新增未保护输出（所有场景 `publication_protected=0`、
  `WATCHDOG_REFUSED=0`；未保护标记本就是平台现状）。
* **身份门记录（新列，加性）**：成熟尝试 `ADMISSIBLE`（如 `A_nominal` 30/30）；无证书尝试
  `REFUSED` 且缺失清单为 `risk_proof_id;protection_level_m`（决策未产出风险证明/PL 的丢弃或早退路径），
  例如 `G_continuous_rejection` 40/45、`HIP_60` 43/60、`C_uwb_fde` 6/30 —— 均为 fail-closed 记录，
  不改变任何已发布量。
* **replay 哈希**：v6 写出的二进制与 v5 必然不同（新增 3 个 provenance 字段），归因 = 格式版本升级；
  本批场景未开启 `UWB_IMU_PL_REPLAY_EXPORT_DIR`，盘上无新 replay 工件。
* `H_mature_union` 续跑点：`bash /tmp/w2_baseline_run.sh` 中把 H 行恢复为 226 epochs 后重跑两侧，
  再执行 `python3 tools/publish_diff.py <base>/H_mature_union <new>/H_mature_union`；本轮新侧已跑到
  epoch ~87 的探测运行显示身份/看门狗记录工作正常（§7.5 样例），成本 ~25-40 min/侧。

### 7.5 计数与场景实测

**接线实测（`H_mature_union` 探测运行，逐 attempt 记录；该运行在 epoch ~87 被手动终止）**

* 到达成熟窗口后每条尝试的身份均**从生产现场取齐**（样例 attempt 58）：
  `snapshot_id=16556290447411530335`（window/transaction 哈希）、
  `state_solution_id=58`（linpoint 版本）、`history_summary_id=13577579993673201648`（C1 `version_digest`）、
  `manifest_digest=9056656343633310150`、`health_state=13276243065478317808`、
  `detector_ids=joint_window_whitened_parity_squared_norm;all_in_graph_residual_diagnostic`、
  `risk_proof_id=5690718879068745039`（W1 §8.5 事件类绑定）、
  `certificate_id=11642378353154719094`；`publication_identity_check=ADMISSIBLE`，
  `state_after=UNAVAILABLE`（平台未认证 ⇒ 显式未保护 + 原因）。
* 看门狗通道在同一场景给出**不同**的两类读数：早期尝试 `sensor_lag≈0`；
  离线（慢于实时）运行下 `sensor_lag` 单调增长（尝试 58 时 10.995 s、`sensor_stale=1`），
  但**不被拒绝**（`sensor_delta=50 ms>0`，不是冻结）；`status=EXECUTED`。
* 冻结/超时路径的 pipeline 级证据见测试 `FrozenDataIsUnavailableBeforeHeavyFde`
  （记录 `sensor_delta=0`、`sensor_lag>限`、`WATCHDOG_REFUSED`、`transaction_opened=0`）。

**计数（本轮，代码提交 `wip(W2)` = `09a71e4`）**

| 量 | 值 |
|---|---|
| 全量套件 | **416 tests / 0 errors / 0 failures**（W1 基线 404 + 6 新用例×2；日志 `raw/run_tests_cwire_w2_stage3.log`；起点 404/0 = `raw/run_tests_cwire_w2base.log`） |
| validation | `UWB_IMU_PL_VALIDATION_SHA=09a71e4 --all` → **50 PASS / 0 FAIL / 6 NOT_RUN**（与 M3/W1 相同；`validation-report.json` 已入库；日志 `raw/validation_w2.log`） |
| 新增用例 | `test_publication_wiring.cpp`：`PipelineAttemptCarriesProductionIdentity`（OUT-04）、`FrozenDataIsUnavailableBeforeHeavyFde`/`WatchdogClauseSemanticsAreExplicit`/`BackwardsClockIsRefusedLikeInputValidation`（OUT-05）、`ControllerTriStateAndAtomicChain`（OUT-06）、`StatelessMonitorPathIsExplicitlyUnprotected`（OUT-07） |
| replay v6 | `GateDReplay.LosslessRoundTripAllBlocksAndActions` 追加三字段往返断言 |
| 运行 schema | `tools/validate_run_schema.py` 场景输出 **PASS uwb-imu-pl/v5**（v16 表头）；`test_gate_d_tools.py` 随套件运行 |

**工具副作用处置（沿用 runbook §12/§13 政策）**：validation 的自动发现把两个 oracle JSON 指向旧
`/tmp/uwb_imu_pl_b1_20260921/b1_runs_v12` dumps（产生虚假 O8g FAIL）⇒ 已 `git restore` 回提交态；
新一版 oracle 产物需完整 226-epoch H 运行（D 包口径），本轮不伪称已重生成。

**差异表**：见 §7.4（7 场景 + HIP 全部"无已发布量变化"、无覆盖缩水；H 列为 D 包续跑）。

### 7.6 OUT-01..03 的 pipeline 级注记（照 W1 对 FDE-03..05 的写法）

| 用例 | pipeline 级触发路径 | 场景名 | 测试名 | 观测 |
|---|---|---|---|---|
| OUT-01（同时间点改解 / 证书不得复用） | `RealtimeIntegrityPipeline::processUwbBatch → applyPublicationGate → PublicationController::finalizeAttempt → checkPublicationIdentity`；生产传 `centre_shift_bound=-1`（无证明路径） | `A_nominal`(30)/`C_uwb_fde`(30) 的逐 attempt 记录（证书 id 随尝试变化；无解变化 ⇒ 不触发 rebind 分支） | `PublicationWiring.ControllerTriStateAndAtomicChain`（三态 + 无证明拒绝） | `identity_check=ADMISSIBLE`；`REFUSED` 时缺失清单入列 |
| OUT-02（受保护/未保护显式区分） | 同一发布门写 `IntegrityOutput::publication.{protected_output,unprotected_output,refusal}`（v16 列） | `C_uwb_fde`(30) 6/30、`G_continuous_rejection`(45) 40/45、`HIP_60` 43/60 的 `REFUSED` 记录；全部场景 `publication_protected=0` | `PublicationWiring.PipelineAttemptCarriesProductionIdentity`、`StatelessMonitorPathIsExplicitlyUnprotected` | 未保护原因逐条可读（"no publication certificate …" / "platform certification is incomplete …"） |
| OUT-03（wall/sensor 分离 + 冻结及时不可用） | `processUwbBatch(batch, ClockSample) → beginAttempt`（重 FDE 之前）；冻结/超时 ⇒ `WATCHDOG_REFUSED` 短路（不进入 `processUwbBatchImpl`） | **无冻结场景**（现有场景 sensor 恒前进 ⇒ 不可触发，按 W1 §7.1 方式标注）；H 的"落后但前进"实测为**不拒绝** | `PublicationWiring.FrozenDataIsUnavailableBeforeHeavyFde`、`WatchdogClauseSemanticsAreExplicit`、`BackwardsClockIsRefusedLikeInputValidation` | `sensor_delta=0 ∧ lag>限 ⇒ REFUSED`；`transaction_opened=0`；倒退 ⇒ `std::invalid_argument` |

**其余"已接线、场景不可触发"项（诚实标注）**：回放 `/clock` 跳变不污染 wall 统计（模块语义 + 测试，
生产无回放标记输入）；`PROTECTED` 路径（需 `formal_eligible=true`，Gate J 未完成，仅测试可达）；
被看门狗拒绝尝试的日志形状（生产者有 pipeline 测试，场景无冻结样本）。

