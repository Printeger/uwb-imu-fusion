# 配置迁移与公共契约冻结（A3-5 / A3-6）

本文件记录工作组 A 第二批（P2）在**不改变任何阈值与算法语义**前提下完成的
配置层迁移规则，以及 B/C 阶段必须遵守的冻结接口。所有新键的解析都在
`IntegrityConfigLoader::load` 内联完成，未新增运行期分支开关。

## 1. 迁移规则（`config/realtime_uwb_imu_pl_research.yaml`）

### 1.1 窗口规模：`epochs` → `intervals`（规范键）

| 输入 | 行为 |
|---|---|
| 只有 `epochs`（v4 旧配置） | 接受；写入迁移警告 `epochs is a legacy alias, prefer intervals`；规范键 `intervals` 一并序列化 |
| 只有 `intervals` | 接受；无警告 |
| 两者同时给出且相等 | 接受；提示 `epochs duplicated intervals; remove the legacy alias` |
| 两者同时给出且不相等 | **拒绝加载**：`integrity_window.epochs conflicts with integrity_window.intervals` |

语义保持不变：`epochs`（即区间数）= 窗口包含的测量区间数；H 的列数、历元数口径不变
（P1 基线：10 区间 → 253×165；20 区间 → 483×315）。

### 1.2 故障阶数：`single_faults_enabled` / `double_faults_enabled` → `max_fault_order`（规范键）

| 输入 | 行为 |
|---|---|
| 只有 legacy 标志 | 接受；由 `double_faults_enabled` 推导 `max_fault_order ∈ {1,2}`；写入迁移警告与规范键 |
| 只有 `max_fault_order` | 接受；反推 `single_faults_enabled=true`、`double_faults_enabled=(order==2)` |
| 两者同时给出且一致 | 接受（当前研究配置即此形态：`max_fault_order: 1` + `single=true/double=false`） |
| 两者同时给出且冲突 | **拒绝加载**：`fault_models.max_fault_order conflicts with legacy single_faults_enabled/double_faults_enabled` |

**不做静默双向写**：解析结果始终以两个规范键同时出现在 `resolved_yaml` 中，冲突一律
报错，绝不“后者覆盖前者”。旧行为（`double_faults_enabled=true` 需要至少一个
`combinations.uwb_plus_accel/uwb_plus_gyro`）保持不变。

### 1.3 `max_linearization_step_norm`：保留 + 警告

值（0.25）与判定逻辑**未改动**。仅追加提示性迁移注释：该混合单位欧氏步长门限是
候选可采纳性检查，物理尺度线性化诊断延后到 B/C（见 `proof-obligations.md` 第 10 项，
G 帧证据 `raw/runs_p2/G_continuous_rejection`）。

## 2. 故障清单（fault manifest）契约

* 新文件 `config/integrity_fault_manifest.yaml`，由 `fault_models.manifest_path`
  引入，相对路径以**配置文件所在目录**为基准解析。
* 严格加载：未知键、未知 family、未声明列表一律拒绝；`evidence_status` 词表为
  `IMPLEMENTED_UNVERIFIED | NOT_IMPLEMENTED`；`amplitude_model` 词表为
  `unbounded_subspace | validated_bounded_set`。
* 能力声明不得超出实现（D3）：有界幅值集合只能声明 `NOT_IMPLEMENTED`；
  `projection_eligibility` 是**结构字段**（实现族必须 `algebraic`，未实现族必须
  `unsupported`），动作标签（`allowed_actions`）不得改变数值可取性。
* 与配置的交叉检查（启动期拒绝）：
  * `manifest.max_fault_order < fault_models.max_fault_order` → 拒绝；
  * 已启用族所引用事件必须在 manifest 中存在；
  * `double_faults_enabled=true` 时必须有已启用的 pair family（当前为 `uwb_imu`）；
  * `protected_quantity`/`position_reference` 必须等于 `position_xyz`/`body_origin`。
* 解析后的 manifest 副本以注释形式写入 `resolved_config.yaml`
  （`# --- resolved fault manifest (read-only copy) ---`），并附带
  `resolved_fault_manifest_id` 与 `resolved_fault_manifest_digest`
  （当前：`uwb-imu-pl/fault-manifest/2026-09-21-1` / `d2a01e7343f5c357`）。
  独立可读副本见 `fault-manifest-resolved.yaml`。

## 3. 哈希纪律

* `config_hash` 在**全部**迁移注释与 manifest 副本追加完成后重算，
  覆盖：规范键、迁移警告、manifest id/digest（即 `resolved_yaml` 全文）。
* 不变量（已由 `IntegrityConfig.HashCoversResolvedConfigurationAfterOverride` 锁定）：
  已迁移的 `resolved_yaml` 再加载一次应得到**相同** hash 与文本（幂等）；
  原始含 legacy 别名的输入与迁移后 dump 的 hash 允许不同（注释与规范键属实质内容）。
* 运行期 hash 复核链见 `effective-config.yaml` 与 `hashes.txt`。

## 4. 证据与回归测试

| 规则 | 测试 |
|---|---|
| 冲突拒绝（order、epochs/intervals） | `ConfigMigration.OrderAndLegacyFlagConflictRejected`、`ConfigMigration.EpochsIntervalsConflictRejected` |
| legacy 映射 + 警告 + 规范键 | `ConfigMigration.LegacyOnlyFieldsGainWarningsAndCanonicalKeys` |
| manifest 词表/结构/未实现声明 | `FaultManifest.*`（5 用例） |
| 启动期拒绝未实现族 | `ConfigMigration.StartupRejectsManifestWithEnabledUnsupportedFamily` |
| v4 配置读兼容 | `IntegrityConfig.RetainsV4ConfigurationReadCompatibility` |
| 生效值与 hash | `IntegrityConfig.HashCoversResolvedConfigurationAfterOverride`、`UnifiedOverridesAreResolvedAndHashed` |

* 命令：`catkin run_tests uwb_imu_pl`（见 `raw/run_tests_p2.log`，240 tests / 0 failures）。
* 迁移后的实跑导出：`raw/runs_p2/*/resolved_config.yaml`（含注释块）。
* census 夹具（P1 生成，P2 补齐规范键）
  `configs/research-double-faults-census.yaml`（order 2）与
  `configs/research-epochs20-census.yaml`（order 1）已加入 `manifest_path`，
  并在 P2 重新运行通过（`raw/census_p2/sd`、`raw/replay_p2/epochs20-census`）。

## 5. 明确不做的事（本轮）

* 不新增算法开关；不修改任何阈值（0.25 gate、`p_fa/p_md`、alert limits、风险预算）。
* 不改变检测器/PL/证据/故障模型的数学。
* 不实现平方根内核、历史摘要、分组包络、惰性 FDE（B/C/D 阶段）。
* 不删除 legacy 键的读取支持；旧配置在给出警告后继续可用。

## 6. history 容量键（C1-c/C2 新增，2026-09-21）

| 键 | 类型 | 默认 | 语义 |
|---|---|---|---|
| `history.max_summary_rows` | u64 | 512 | 摘要行数容量 |
| `history.max_fault_columns` | u64 | 512 | 摘要故障列容量 |
| `history.max_perp_rows` | u64 | 512 | 检测（`ν_⊥`）行容量 |
| `history.capacity_action` | 枚举 | `REFUSE` | 超限动作：`REFUSE` / `RESET` / `STOP_PROTECTED` |

* 默认值来源（D-2 实测）：可恢复跨度 ≤ `epochs + recovery_margin_epochs` = 20 历元，
  每历元 ≤ 22 列（8 anchors × {常数,时间线性} + 6 IMU 轴）；实测 q_hist 在半个跨度
  （epoch 26，跨度 10）为 **220** ⇒ 全跨度界 440，默认 **512** 含余量。
  实测记录见 `history-fault-parameterization.md`。
* 严格加载：节缺省 = 上表默认；节存在时**四个键全部必需**；未知键、未知动作、
  缺键一律硬错误（测试 `IntegrityConfig.HistoryCapacityKeysAreStrictlyLoaded`）。
* resolved dump 规范化补齐该节，config hash 覆盖。
* **执行语义（超限 → 不可用并计数）尚未接入**：摘要路径（Part C 接线）未落地；
  "绝不静默丢最旧故障"为冻结语义，不存在 `DROP_OLDEST` 一类动作（测试断言其被拒绝）。
