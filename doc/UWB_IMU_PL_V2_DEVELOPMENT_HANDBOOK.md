# UWB–IMU Integrity V2 Development Handbook

**目标分支：** `feature/realtime-uwb-imu-pl`  
**目标系统：** UWB–IMU 增量平滑紧耦合定位，残差式 / slope-based RAIM，统一 FD/FDE，IMU 故障排除后的完整性感知运动桥接  
**文档状态：** V2 架构与开发基线  
**基线观察日期：** 2026-09-04  
**适用对象：** 算法设计、C++/GTSAM 开发、仿真、实验、完整性验证  
**安全声明：** 本文档是研究软件设计规范，不构成适航、功能安全或产品认证结论。

> V2 只有一个目标架构，不拆成 “FDE Engineering V0 / FDE Formal V1”。  
> 但实现、数学验证和实验证据必须通过逐项 verification gates。Gate 是同一个 V2 的证据积累顺序，不是多个产品版本。

---

## 0. 如何使用本 handbook

开发时把本文件放入仓库：

```text
doc/UWB_IMU_PL_V2_DEVELOPMENT_HANDBOOK.md
```

每个 pull request 必须回答四个问题：

1. 修改对应本 handbook 的哪一个设计条款、接口或 verification gate？
2. 是否改变 fault model、risk budget、protected state、factor provenance 或 PL 语义？
3. 是否新增了可审计日志与测试证据？
4. 是否仍满足“当前 epoch 在完整性决策前不修改 backend”的事务约束？

本文中的关键词含义：

- **MUST / 必须**：违反即不符合 V2 设计。
- **SHOULD / 应**：默认遵守；偏离时必须写 ADR（Architecture Decision Record）。
- **MAY / 可**：可选扩展，不得改变正式路径语义。
- **formal path / 正式路径**：用于 detector、FDE、PL、availability 与完整性标签的路径。
- **shadow path / 影子路径**：只做诊断、比较或研究，不参与正式决策。

---

## 1. V2 的最终目标

V2 应在每个 UWB keyframe epoch \(k\rightarrow k+1\) 内完成：

1. 从尚未提交的 IMU 样本构造 current preintegration；
2. 从当前 UWB group 构造测距因子；
3. 从 committed history 提取一致的 Bayes-tree / fixed-lag 边界信息；
4. 构造 whitened joint integrity window；
5. 执行一个正式的联合残差 / parity detector；
6. 对 UWB、IMU 及受支持的跨传感器组合故障生成 fault-mode evidence；
7. 在 factor-group 粒度生成 FDE actions；
8. 用固定线性化点和低秩更新评估所有 actions；
9. 处理 exclusion ambiguity 与 wrong-exclusion risk；
10. 对最终 action 重新检测并计算 post-FDE slope-based PL；
11. 仅执行一次 backend transaction；
12. 更新 sensor health、factor provenance、日志与 availability。

目标链路为：

```text
raw IMU + UWB
      │
      ▼
EpochTransaction（尚未提交）
      │
      ▼
Bayes-tree boundary + active integrity window
      │
      ▼
joint whitening / parity FD
      │
      ├── no alarm ──> all-in PL ──> one commit
      │
      └── alarm
              │
              ▼
      fault-hypothesis evidence pool
              │
              ▼
      factor-group exclusion-action pool
              │
              ▼
      rank-update candidate evaluation
              │
              ▼
      ambiguity-safe action selection
              │
              ▼
      post-FDE FD + slope PL
              │
              ▼
           one commit
```

---

## 2. 设计结论总表

| 设计问题 | V2 决定 |
|---|---|
| 正式 detector 数量 | 一个 **joint whitened window detector** |
| UWB / IMU 各自 FD | 作为 fault-mode evidence 与诊断，不单独消耗正式 continuity budget |
| “池子”是什么 | detection 层是 whitened joint residual/parity pool；FDE 层是 `FaultUnit` / `ExclusionAction` pool |
| UWB exclusion unit | 同一物理 anchor 在受监测时间段内的全部相关 range rows / factors |
| IMU exclusion unit | 一个有明确时间区间的 preintegration factor group |
| IMU fault model | 结构化 accel / gyro / bias-jump fault subspace；禁止“任意 15D fault”作为默认可监测模型 |
| 当前 IMU 何时进入图 | FD/FDE/PL 后 |
| 当前 UWB 何时进入图 | FD/FDE/PL 后 |
| candidate prior | 所有 candidate 共享同一 committed-history boundary 与同一线性化版本 |
| candidate posterior | 基础信息矩阵只构造一次；用 Sherman–Morrison/Woodbury 类低秩更新/降阶求解 |
| 精确性 oracle | dense local refactorization；只用于测试和数值 fallback，不作为默认在线枚举方式 |
| IMU 排除后 | 删除 suspect preintegration，加入 `KinematicBridgeFactor` 与 bias continuity |
| bridge 模型 | 通用 CV/constant-attitude + maneuver envelope；可选 quadrotor dynamics bridge |
| 缓变 fault | detector 使用滑窗 / 多历元证据，不限于两 keyframe 快照 |
| 多个 plausible exclusions | 排除其 fault-unit 并集，或声明 ambiguity/unavailable；不得仅按最大 residual 选一个 |
| GNC/robust weighting | shadow 或显式建模；禁止 estimator 与 formal PL 使用不一致的隐含权重 |
| persistent fault | health quarantine + factor ledger + active-window recovery |
| 已边缘化污染 | 若无法从可信 checkpoint 恢复，则 `HISTORY_PRIOR_CONTAMINATED`，不得发布 formal available |
| factor acceptance vs availability | 分离；PL 超 AL 不自动表示 factor fault |
| backend mutation | 每个 epoch 至多一次事务提交 |

---

## 3. 当前代码基线审计

### 3.1 已有且应保留的能力

当前分支已经具备以下基础：

- GTSAM 4.2 iSAM2 / incremental fixed-lag backend；
- 15D keyframe state：`Pose3 + velocity + ConstantBias`；
- `CombinedImuFactor`；
- UWB batch factor；
- 强类型 ID：`AnchorId`、`MeasurementId`、`FactorId`、`StateId`、`HypothesisId`、`BatchId`；
- `LinearizationVersion`；
- whitened row block、current marginal 与 `solveInformation()` 抽象；
- 当前 UWB group 的 conditional \(\chi^2\) detector；
- anchor-level slope 与 PL；
- Method B leave-current-out information downdate 的雏形；
- strict config、resolved YAML、hash、manifest、timing 与 formal gate 文化；
- fixed-lag 与 full-history 两种 backend；
- UWB fault 仿真模式和现有测试基线。

这些不是重写对象。V2 应复用其 ID、versioning、strict config、logger、test oracle 和 Bayes-tree selected-solve 思路。

### 3.2 当前实现的关键阻塞点

当前 `IncrementalUwbImuEstimator::predictTo()` 会：

```text
preintegrate IMU
→ create CombinedImuFactor
→ backendUpdate()
→ increment epoch
→ set pending UWB epoch
```

因此现有 “pre-measurement prior” 只排除了 current UWB，却已经吸收 current IMU。该语义无法支持 current IMU FDE。V2 的首要重构是：

```text
predictTo() commits IMU
```

改为：

```text
prepareEpoch() builds pending factors only
evaluateIntegrity()
commitEpoch(plan) exactly once
```

### 3.3 当前类型系统的缺口

现有 `FaultHypothesis` 是 anchor-specific：

```cpp
AnchorId anchor_id;
std::vector<MeasurementId> affected_measurements;
std::string physical_fault_type;
```

V2 需要表达：

- sensor-independent `FaultUnitId`；
- 一个 hypothesis 包含多个 fault units；
- affected factor groups / row blocks / epoch intervals；
- fault-parameter dimension；
- fault subspace generator；
- exclusion action；
- prior probability 与 risk allocation；
- history provenance；
- monitorability status。

不得继续通过 `anchor_id == 0` 或字符串约定模拟 IMU fault。

### 3.4 当前 snapshot 的缺口

现有 `CurrentStatePrior` 与 `currentMarginal()` 只面向当前 15D state。V2 正式 detector 需要一个 active window：

```text
boundary prior
+ retained historical factor blocks
+ current pending IMU block
+ current pending UWB block
```

所以必须新增 window snapshot，而不是强行扩展 `CurrentStatePrior` 的含义。

### 3.5 当前风险模型的缺口

现有 risk config 主要是：

```text
p_fa
p_hmi_total
nominal_axis_tail
p_nm
single_anchor_p_md
single_anchor_prior_bound
HAL / VAL
```

V2 还需要：

- monitored UWB/IMU fault priors；
- fault combination policy；
- outcome-conditioned allocations；
- bridge model escape risk或确定性 bound；
- history contamination allocation；
- linearization/model-overbound allocation；
- operation-level continuity horizon；
- isolation / wrong-exclusion accounting。

---

## 4. 对现有讨论与 Claude 批判意见的整合结论

### 4.1 接受的批判

以下批判应写入 V2 核心：

1. **不能一边禁止 candidate re-solve，一边含糊地要求每个 candidate “重新算 posterior”。**  
   V2 明确采用：一次 base factorization + block downdate/update；dense refactorization 作为 oracle。

2. **两 keyframe 检测不足以稳定检测缓变 IMU fault。**  
   V2 正式 detector 使用 active sliding window。当前-epoch action 与 PL 仍可使用低维 touched-key Schur / rank-update kernel。

3. **导师的池子想法可行。**  
   whitening 后异质 residual 可以在 joint parity space 统一检测；但 exclusion 必须按物理 fault unit / factor group 执行。

4. **一个 joint detector 比三个独立正式 detector 更干净。**  
   UWB/IMU-specific tests保留为 evidence，不再简单用 \(D_U\lor D_I\lor D_J\) 并做三项 union bound。

5. **CV bridge 不是唯一选择。**  
   通用 kinematic bridge 是安全基线；若有与 IMU 独立的控制输入，可提供 quadrotor dynamics bridge 作为更紧模型。

6. **架构一次设计到位，证据按 gate 累积。**  
   这与仓库已有 formal discipline 一致。

### 4.2 需要修正的批判

Claude 意见中两处不能直接照搬：

#### A. 低秩更新文献的时间状态

“Extending ARAIM ... via Rank Updates” 是 ION GNSS+ 2026 的会议摘要，会议日期晚于本文档基线日期。它可以作为设计启发，但不能当作已经完成同行评审和独立复现的既定结论。V2 必须用自己的 dense oracle、Monte Carlo 和 timing 证据验证。

#### B. “estimator 用 GNC，integrity 用离散 hypothesis”不能无条件分离

如果 estimator 实际提交的是数据依赖的连续权重，而 PL 仍按 full-weight / removed-factor 离散模型计算，估计状态、协方差和 fault map 不一致，formal PL 语义会失效。

V2 规定：

- 正式路径默认使用明确的 inclusion/exclusion action；
- GNC 可作为 shadow evidence、candidate ordering 或研究对照；
- 若未来正式使用连续权重，权重必须冻结、记录，并作为 effective covariance 的一部分进入 detector、rank update、fault hypothesis 和 PL；还必须单独验证数据依赖权重对 overbound 的影响。

---

## 5. V2 完整性范围与标签

### 5.1 Protected state

默认 protected state：

\[
p^W_{B,k}
\]

即 world frame 下 body origin position。

输出：

\[
PL_x,\; PL_y,\; PL_z
\]

\[
HPL=\sqrt{PL_x^2+PL_y^2},\qquad VPL=PL_z.
\]

UWB antenna lever arm只属于 measurement model，不改变 protected point。若以后保护 antenna position 或 full pose，必须增加新的 protected selector 与 alert limits，不能复用当前标签。

### 5.2 目标 monitored fault set

V2 的 hypothesis generator 应支持：

\[
H_0
\]

\[
H_{U_i}: \text{physical anchor }i\text{ range fault}
\]

\[
H_{I,a_x},H_{I,a_y},H_{I,a_z}
\]

\[
H_{I,g_x},H_{I,g_y},H_{I,g_z}
\]

可选 block hypotheses：

\[
H_{I,a_{xyz}},\qquad H_{I,g_{xyz}}
\]

受配置和 monitorability gate 允许时：

\[
H_{U_i+I,a_q},\qquad H_{U_i+I,g_q}
\]

以及可选：

\[
H_{U_i+U_j}.
\]

“支持生成”不等于“任何 geometry 都能 monitor”。运行时必须检查 rank、smallest singular value、condition number 和 finite slope。

### 5.3 硬件健康事件

以下事件优先由 deterministic barrier 处理，而不是伪装成 Gaussian RAIM fault：

- IMU packet dropout；
- timestamp reversal；
- gap 超限；
- saturation flag；
- non-finite value；
- UWB invalid quality flag；
- anchor map/version mismatch；
- covariance non-SPD；
- clock/timestamp contract violation。

这些事件直接产生受约束的 exclusion action。若要把它们纳入概率 risk tree，必须另有 calibrated occurrence rate。

### 5.4 Proposed label

完成全部 V2 gates 后，可考虑新标签：

```text
FORMAL_WINDOWED_UWB_IMU_FDE
```

标签必须携带 machine-readable scope，例如：

```json
{
  "protected_state": "body_origin_position_world",
  "detector": "joint_window_residual_chi_square",
  "pl_method": "residual_failure_mode_slope",
  "window_epochs": 20,
  "monitored_fault_cardinality": 2,
  "imu_fault_models": [
    "interval_accel_bias_axis",
    "interval_gyro_bias_axis"
  ],
  "bridge_model": "kinematic_cv_bounded",
  "history_recovery": "active_window_only"
}
```

---

## 6. 核心完整性不变量

### INV-01 — Detect before commit

current epoch 的 IMU、UWB 和 bridge candidate 在完整性决策前不得永久进入 backend。

### INV-02 — Single transaction

每个 epoch 的 backend mutation 次数必须为 0 或 1。一个 transaction 可以同时：

- remove active factors；
- add selected IMU/UWB/bridge factors；
- add new state values；
- update timestamps。

### INV-03 — Common base

所有 candidates 使用完全相同的：

- committed graph version；
- factor ledger version；
- ordering version；
- noise model version；
- linearization point；
- boundary prior；
- raw measurement set。

### INV-04 — Physical grouping

FDE 不得按 arbitrary residual row 删除。删除单位必须映射到物理 fault source。

### INV-05 — Candidate-specific recomputation

排除后必须更新：

- rows；
- rank / dof；
- state increment；
- covariance；
- residual statistic；
- detector threshold；
- remaining fault hypotheses；
- slope；
- PL；
- availability。

“沿用 all-in PL”不允许。

### INV-06 — No hidden robust weights

formal snapshot 中每个 row block 的 effective covariance、robust weight 和 whitening model 必须显式可审计。

### INV-07 — History provenance

任何 active factor 必须能追溯到：

- sensor；
- raw IDs；
- epoch interval；
- factor slot；
- health state；
- fault units；
- marginalization status。

### INV-08 — Ambiguity is not confidence

两个 candidate 都 pass 不代表 winner 正确。必须执行 ambiguity policy。

### INV-09 — Bridge is a model, not a free edge

bridge 必须有独立输入、明确 residual、optimization covariance、integrity envelope、最大持续时间和 model ID。

### INV-10 — Fail closed

model invalid、unmonitorable、history contaminated、risk budget invalid、rank-update SPD failure或 bridge timeout 时不得发布 formal available。

---

## 7. 事务化 estimator

### 7.1 `EpochTransaction`

建议新增：

```cpp
struct EpochTransaction {
  TransactionId id;
  std::uint64_t base_graph_version;
  LinearizationVersion base_version;

  std::size_t previous_epoch;
  std::size_t proposed_epoch;
  TimestampNs begin;
  TimestampNs end;

  NavigationState previous_state;
  NavigationState nominal_predicted_state;

  std::vector<ImuMeasurement> raw_imu_slice;
  std::shared_ptr<const gtsam::PreintegratedCombinedMeasurements> preintegration;
  UwbBatch uwb_batch;

  PendingFactorGroup imu_group;
  std::vector<PendingFactorGroup> uwb_groups;
  PendingFactorGroup generic_bridge_group;
  std::optional<PendingFactorGroup> dynamics_bridge_group;

  ImuQueueCursor imu_cursor;
  bool backend_mutated = false;
};
```

transaction 必须 immutable 或逻辑只读。candidate evaluation 不得修改它。

### 7.2 API

将当前 lifecycle：

```cpp
predictTo(timestamp);
preMeasurementSnapshot(batch);
commitUwbBatch(batch);
rejectUwbBatch(batch, reason);
```

替换为：

```cpp
EpochTransaction prepareEpoch(const UwbBatch& batch);

LinearizedIntegrityWindow buildIntegrityWindow(
    const EpochTransaction& transaction,
    const IntegrityWindowRequest& request) const;

CommitReceipt commitEpoch(
    EpochTransaction&& transaction,
    const EpochCommitPlan& plan);

DiscardReceipt discardEpoch(
    EpochTransaction&& transaction,
    const DiscardReason& reason);
```

### 7.3 `prepareEpoch()` 的职责

它可以：

- 验证 UWB batch；
- 查找 causal IMU slice；
- 执行 preintegration；
- 生成 nominal predicted state；
- 构造 pending factors；
- 分配 factor IDs；
- 构造 generic bridge；
- 从 backend 读取 boundary / factor ledger；
- 生成 transaction。

它不得：

- 调用 backend `update()`；
- increment committed epoch；
- pop IMU queue；
-修改 health state；
-写“committed”日志；
-改变 graph/ordering/linpoint version。

### 7.4 `commitEpoch()` 的职责

在 version check 通过后：

1. 构造 selected new factors；
2. 构造 factors-to-remove；
3. 构造 new values；
4. 在一个 backend call 中 add/remove；
5. 更新 epoch 和 timestamp；
6. 消费 IMU queue；
7. 更新 factor ledger；
8. 更新 health manager；
9. 查询 current state；
10. 写 commit receipt。

GTSAM iSAM2 和 fixed-lag API 都支持 update 时传入 factor indices to remove。V2 的 backend wrapper 必须暴露这一能力。

### 7.5 `discardEpoch()` 不是 rollback

transaction 尚未提交，所以 discard 不做图回滚。它只：

- 消费已过期 raw data 或按策略保留 replay buffer；
-保持 previous committed state；
-写 fail-closed event；
-决定是否启动 bridge-only / controlled reinitialization transaction。

---

## 8. Factor Ledger 与历史恢复

### 8.1 目的

新增 `FactorLedger`，记录 backend factor slot 与物理 provenance。

```cpp
struct FactorLedgerEntry {
  FactorId factor_id;
  FactorGroupId group_id;
  SensorType sensor;
  FactorKind kind;

  std::vector<gtsam::Key> keys;
  std::vector<MeasurementId> source_measurements;
  std::vector<FaultUnitId> associated_fault_units;

  std::size_t epoch_begin;
  std::size_t epoch_end;
  TimestampNs time_begin;
  TimestampNs time_end;

  std::optional<gtsam::FactorIndex> backend_slot;
  FactorLifecycle lifecycle;
  HealthState health_at_commit;
  std::string noise_model_id;
  std::string model_id;
  LinearizationVersion commit_version;
};
```

`FactorLifecycle` 至少包括：

```text
PENDING
ACTIVE
REMOVED_BY_FDE
MARGINALIZED
SUPERSEDED_BY_BRIDGE
QUARANTINED_SOURCE
```

### 8.2 Late detection

滑窗 detector 可能在 \(k+d\) 才发现从 \(k\) 开始的 ramp fault。若 fault factors 仍 active：

```text
remove affected factor slots
+ add replacement bridge factors
+ re-eliminate affected cliques
```

必须在一个 recovery transaction 中完成。

### 8.3 Marginalization boundary

若 suspect factor 已进入 marginalized prior，则普通 factor removal 已经不能恢复其影响。

V2 必须满足至少一个策略：

**策略 A — Maturity delay**

```text
fixed_lag_epochs > fd_window_epochs + recovery_margin_epochs
```

且在 detector 能确认 epoch “mature” 之前不允许其信息不可逆边缘化。

**策略 B — Replay checkpoint**

保存可信边界 checkpoint、raw IMU/UWB 与 factor configs；检测到 late fault 时从 checkpoint 重建 active window。

若二者都不可用：

```text
HISTORY_PRIOR_CONTAMINATED
formal_eligible = false
availability = UNAVAILABLE
```

不得把 `historical_fault_provenance=false` 默认为安全。

---

## 9. Joint Integrity Window

### 9.1 为什么不是只用当前 15D marginal

UWB step fault通常单历元可见，但 IMU ramp / small bias需要时间积累；gyro fault还可能通过运动、重力和位置轨迹的跨历元耦合才变得可观测。因此 detector 必须利用窗口。

### 9.2 Window state

令窗口为 \(k-L+1,\ldots,k+1\)，状态增量：

\[
\delta X =
[\delta x_{k-L+1}^{\top},\ldots,\delta x_{k+1}^{\top}]^\top .
\]

每个 \(x_i\) 为 15D tangent state：

\[
x_i=(R_i,p_i,v_i,b_{a,i},b_{g,i}).
\]

### 9.3 Window rows

窗口包含：

1. boundary prior；
2. active historical IMU factors；
3. active historical UWB factors；
4. current pending IMU factor；
5. current pending UWB factors；
6. candidate-specific bridge rows；
7. 不作为 fault source 的 trusted regularization rows；
8. provenance 与 health metadata。

### 9.4 Boundary prior

full-history backend 必须把窗口以前的变量 Schur-compress 成 boundary information；fixed-lag backend可使用其 active boundary priors，但必须验证没有重复计入 active factors。

统一表示：

\[
\frac12\delta X_b^\top\Lambda_b\delta X_b-\eta_b^\top\delta X_b.
\]

可转成 square-root rows：

\[
\|R_b\delta X_b-d_b\|^2,
\qquad R_b^\top R_b=\Lambda_b.
\]

### 9.5 Snapshot 类型

```cpp
struct LinearizedFactorBlock {
  FactorGroupId group_id;
  FactorKind kind;
  SensorType sensor;
  RowRole role;

  Eigen::MatrixXd jacobian_raw;
  Eigen::VectorXd residual_raw;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd whitener;

  Eigen::MatrixXd jacobian_whitened;
  Eigen::VectorXd residual_whitened;

  std::vector<int> window_column_indices;
  std::vector<FaultUnitId> fault_units;
  double effective_weight;
  std::string whitening_model_id;
  LinearizationVersion version;
};

struct LinearizedIntegrityWindow {
  WindowId id;
  LinearizationVersion version;
  std::vector<StateLayoutEntry> state_layout;
  std::vector<LinearizedFactorBlock> blocks;

  Eigen::MatrixXd H;
  Eigen::VectorXd z;
  int rank;
  int dof;

  Eigen::MatrixXd base_information;
  Eigen::VectorXd base_information_rhs;
  WindowCapabilities capabilities;
};
```

### 9.6 Freeze rule

一个 FD/FDE cycle 内：

- 不允许 iSAM2 relinearize；
- 不允许 covariance model change；
- 不允许 robust weight change；
- 所有 candidate 使用同一 `LinearizationVersion`。

winner commit 后 backend 可进行其正常 nonlinear update。若实际 post-commit step 超过 linearization gate，输出必须降级并触发下一 epoch 的 model validity alarm。

---

## 10. 正式联合 detector

### 10.1 Canonical model

窗口 whitening 后：

\[
z = H\delta X + A_h f_h + v,\qquad v\sim\mathcal N(0,I).
\]

使用 QR/SVD 构造：

\[
P_\perp=I-HH^\dagger .
\]

不得显式形成大规模 \(P_\perp\)；实现使用 null-space QR 或 residual projection solve。

### 10.2 Statistic

\[
T_0 = \|P_\perp z\|^2.
\]

在 nominal linear Gaussian overbound 成立时：

\[
T_0\sim\chi^2_\nu,
\qquad
\nu=n_{\mathrm{rows}}-\operatorname{rank}(H).
\]

threshold：

\[
\tau^2=F^{-1}_{\chi^2_\nu}(1-P_{\mathrm{FA,test}}).
\]

通过条件：

\[
T_0\le \tau^2.
\]

代码和日志必须明确 statistic 是 squared norm；避免同时把 \(\tau\) 和 \(\tau^2\) 命名为 threshold。

### 10.3 Fault distribution

在 \(H_h\) 下：

\[
T_0\sim\chi^2_{\nu,\lambda_h^2},
\]

\[
\lambda_h^2=\|P_\perp A_h f_h\|^2.
\]

### 10.4 Overlapping windows 与 continuity

单个 window 的 central \(\chi^2\) 结论不自动给出整个任务期间的 false alarm probability。连续滑窗高度相关。

配置必须同时给出：

```text
p_fa_per_test
continuity_horizon_tests
continuity_accounting = union_bound | calibrated_sequential
```

保守 union bound：

\[
P_{\mathrm{FA,operation}}
\le
N_{\mathrm{tests}}P_{\mathrm{FA,test}}.
\]

若使用 calibrated sequential threshold，必须保存 Monte Carlo / replay 校准证据和配置 hash。

### 10.5 Modality-specific evidence

不再配置三个正式 alarm thresholds。为诊断和 candidate ordering输出：

```cpp
struct SensorEvidence {
  SensorType sensor;
  double explained_parity_energy;
  double normalized_score;
  bool hardware_barrier_triggered;
};
```

这些 evidence 不可用于 formal pruning，除非其漏检风险被纳入 budget。

---

## 11. Fault Hypothesis Pool

### 11.1 `FaultUnit`

```cpp
struct FaultUnit {
  FaultUnitId id;
  SensorType sensor;
  FaultKind kind;

  std::size_t epoch_begin;
  std::size_t epoch_end;
  TimestampNs time_begin;
  TimestampNs time_end;

  std::vector<FactorGroupId> affected_groups;
  std::vector<MeasurementId> affected_measurements;

  int parameter_dimension;
  FaultSubspaceModel subspace_model;

  double prior_probability_bound;
  HealthState source_health;
  bool persistent;
};
```

### 11.2 `FaultHypothesis`

```cpp
struct FaultHypothesisV2 {
  HypothesisId id;
  std::vector<FaultUnitId> units;
  Eigen::MatrixXd A;   // whitened fault map in current window

  double prior_probability_bound;
  double p_md_allocation;
  double hmi_allocation;

  MonitorabilityResult monitorability;
  bool monitored;
  std::string pruning_reason;
};
```

### 11.3 `FaultHypothesis != ExclusionAction`

例如：

```text
H_I_ax
H_I_ay
H_I_az
```

可能共享：

```text
E_REMOVE_CURRENT_IMU_USE_BRIDGE
```

因为它们是不同 fault directions，但实际恢复动作相同。

### 11.4 UWB fault map

对 anchor \(i\) 的 shared bias：

\[
f_{U_i}=A_{U_i}\delta_i.
\]

若一个 epoch 每 anchor 一条 range，\(A_{U_i}\) 是一列 incidence vector。若 window 内假设 persistent anchor bias，同一 fault parameter 可作用于多个 epoch rows：

\[
A_{U_i}^{\mathrm{persistent}}
=
[e_{r_{i,k-L+1}},\ldots,e_{r_{i,k+1}}]^\top
\]

在对应 rows 填 1 或经过 whitening 后的系数。

需要区分：

- `anchor_bias_epoch_independent`；
- `anchor_bias_persistent_constant`；
- `anchor_bias_ramp`；
- `anchor_outage` deterministic event。

### 11.5 IMU measurement fault 与 bias-state fault

必须区分：

1. **IMU measurement corruption**：污染 preintegrated motion residual；
2. **bias-state jump**：真实 bias dynamics发生跳变，可能同时影响 bias transition residual；
3. **packet/dropout/saturation**：硬件 barrier。

不能把三者合成“IMU factor任意坏”。

---

## 12. IMU fault subspace

### 12.1 Fault parameterization

对 preintegration interval \([t_i,t_j]\)：

\[
\tilde a(t)=a(t)+B_a(t)f_a,
\]

\[
\tilde\omega(t)=\omega(t)+B_g(t)f_g.
\]

基础模型先支持 interval-constant additive faults：

\[
f_a\in\mathbb R^3,\qquad f_g\in\mathbb R^3.
\]

axis-wise hypothesis使用 \(f_{a_x}\in\mathbb R\) 等。

### 12.2 Preintegration sensitivity

preintegrated motion error可写成：

\[
\delta r_I =
G_a f_a + G_g f_g.
\]

因此：

\[
A_{I,a}=W_I G_a,\qquad
A_{I,g}=W_I G_g.
\]

GTSAM preintegration内部已有 delta rotation/velocity/position 对 bias 的 Jacobians。对一个 interval-constant additive measurement bias，它们可作为局部 sensitivity 的起点，但必须：

- 确认符号；
- 确认 frame；
- 确认 residual ordering；
- 区分 9D motion residual 与 6D bias evolution residual；
- 用 raw-IMU finite difference 验证。

### 12.3 推荐实现

新增：

```cpp
class ImuFaultSubspaceBuilder {
 public:
  ImuFaultSubspaces build(
      const EpochTransaction& transaction,
      const LinearizedFactorBlock& combined_imu_block) const;
};
```

输出：

```cpp
struct ImuFaultSubspaces {
  Eigen::MatrixXd accel_xyz;
  Eigen::MatrixXd gyro_xyz;
  std::array<Eigen::VectorXd, 3> accel_axis;
  std::array<Eigen::VectorXd, 3> gyro_axis;
  Eigen::MatrixXd bias_jump_accel;
  Eigen::MatrixXd bias_jump_gyro;
};
```

### 12.4 Analytic + finite-difference contract

正式路径使用 analytic sensitivity。测试 oracle 对 raw IMU samples 注入：

\[
\epsilon e_q
\]

重新 preintegrate，比较：

\[
\frac{r_I(f+\epsilon e_q)-r_I(f-\epsilon e_q)}{2\epsilon}
\]

与 analytic column。

验收至少覆盖：

- stationary；
- constant velocity；
- constant acceleration；
- constant yaw rate；
- 3D rotation；
- 不同 \(\Delta t\)；
- nonzero initial bias。

### 12.5 Ramp model

若监测 ramp：

\[
f(t)=f_0+\dot f(t-t_i),
\]

fault parameter dimension 变为 2/axis 或 6/block。`A_h` 必须通过时间加权 sensitivity integration产生。不得用 constant-bias column直接声称覆盖 ramp。

---

## 13. Monitorability 与 redundancy gate

对每个 hypothesis：

\[
M_h=A_h^\top P_\perp A_h.
\]

计算：

- rank；
- \(\sigma_{\min}\)；
- \(\sigma_{\max}\)；
- condition number；
- protected-axis slope；
- residual DOF。

要求：

```text
rank(M_h) == parameter_dimension
sigma_min >= configured threshold
condition <= max_fault_gram_condition
all protected slopes finite
```

否则：

```text
UNMONITORABLE
```

运行语义：

- 若 hypothesis prior 已计入 `p_nm` 且总 unmonitored risk 合法，可继续但 PL 标明 unmonitored risk；
- 否则 availability 必须为 `UNAVAILABLE`。

尤其要观察 gyro fault。UWB 主要约束 position；纯 gyro fault 的可观测性依赖：

- lever arm；
- gravity/acceleration coupling；
- motion excitation；
- window length；
- anchor geometry；
- bias prior。

V2 不允许硬编码“8 anchors => gyro fault可监测”。

---

## 14. Fault-mode evidence 与 isolation

### 14.1 不按 raw residual 排名

whitening 解决量纲问题，但 row-wise最大值仍忽略：

- shared state coupling；
- preintegration correlation；
- fault subspace dimension；
-同一物理 fault影响多行。

### 14.2 Profile residual / GLRT evidence

令：

\[
y=P_\perp z,\qquad B_h=P_\perp A_h.
\]

fault amplitude estimate：

\[
\hat f_h=(B_h^\top B_h)^\dagger B_h^\top y.
\]

hypothesis-conditioned residual：

\[
T_h=\|y-B_h\hat f_h\|^2.
\]

explained parity energy：

\[
\Delta T_h=T_0-T_h.
\]

输出：

```cpp
struct FaultModeEvidence {
  HypothesisId hypothesis;
  Eigen::VectorXd estimated_fault;
  double all_in_statistic;
  double conditioned_statistic;
  double explained_energy;
  double log_evidence;
  MonitorabilityResult monitorability;
};
```

### 14.3 Formal pruning rule

fault-mode evidence可以：

- 排序 candidate；
- 构造 plausible set；
- 提供 fault amplitude；
- 触发 health suspicion。

不得仅凭低 score 删除一个已分配风险的 hypothesis。若要剪枝，剪枝概率必须进入 `p_nm` 或独立 risk allocation。

---

## 15. Exclusion Action Pool

### 15.1 类型

```cpp
struct ExclusionAction {
  ExclusionActionId id;
  std::vector<FaultUnitId> covered_units;

  std::vector<FactorGroupId> groups_to_remove;
  std::vector<FactorGroupId> groups_to_add;

  BridgeMode bridge_mode;
  int exclusion_cardinality;
  std::string action_model_id;
};
```

### 15.2 基础 actions

```text
KEEP_ALL
REMOVE_UWB_ANCHOR_i
REMOVE_CURRENT_IMU_ADD_GENERIC_BRIDGE
REMOVE_CURRENT_IMU_ADD_DYNAMICS_BRIDGE
REMOVE_UWB_i_AND_CURRENT_IMU_ADD_BRIDGE
REMOVE_UWB_i_AND_UWB_j
REMOVE_HISTORICAL_GROUPS_WITHIN_ACTIVE_WINDOW
```

生成器只生成配置允许且 factor ledger可执行的 actions。

### 15.3 Action coverage

`covers(e,h)` 表示 action \(e\) 排除了 hypothesis \(h\) 中全部受影响因子，或用独立 bridge替换了 suspect IMU transition。

若只排除 \(H_{U_i+I_a}\) 中的 UWB unit，而保留 faulty IMU，action不覆盖该 hypothesis。

### 15.4 Cardinality

max cardinality必须同时受：

- prior risk；
- geometry；
- residual DOF；
- runtime；
- bridge availability；
- active factor provenance

约束。仅配置 `max_faults=2` 不构成可监测证明。

---

## 16. Candidate evaluation：一次 base、低秩更新

### 16.1 Frozen linear normal equations

全量 pending/all-in window：

\[
\Lambda_0=H_0^\top H_0,\qquad
\eta_0=H_0^\top z_0.
\]

\[
\Sigma_0=\Lambda_0^{-1},\qquad
\delta X_0=\Sigma_0\eta_0.
\]

实现使用 Cholesky/QR solves，不显式求逆。

### 16.2 删除一个 whitened block

待删除 block：

\[
J_r\delta X\simeq b_r.
\]

则：

\[
\Lambda_-=\Lambda_0-J_r^\top J_r,
\]

\[
\eta_-=\eta_0-J_r^\top b_r.
\]

downdate covariance：

\[
\Sigma_-=
\Sigma_0+
\Sigma_0J_r^\top
(I-J_r\Sigma_0J_r^\top)^{-1}
J_r\Sigma_0.
\]

必要条件：

\[
I-J_r\Sigma_0J_r^\top \succ 0.
\]

### 16.3 加入 bridge block

bridge：

\[
J_b\delta X\simeq b_b.
\]

\[
\Lambda_e=\Lambda_-+J_b^\top J_b,
\]

\[
\eta_e=\eta_-+J_b^\top b_b.
\]

\[
\Sigma_e=
\Sigma_--
\Sigma_-J_b^\top
(I+J_b\Sigma_-J_b^\top)^{-1}
J_b\Sigma_-.
\]

\[
\delta X_e=\Sigma_e\eta_e.
\]

### 16.4 Candidate statistic

必须在 candidate solution上重新计算 retained residual：

\[
T_e=
\sum_{\ell\in\mathcal R_e}
\|b_\ell-J_\ell\delta X_e\|^2.
\]

\[
\nu_e=n_{\mathrm{rows},e}-\operatorname{rank}(H_e).
\]

\[
\tau_e^2=F^{-1}_{\chi^2_{\nu_e}}
(1-P_{\mathrm{FA,test},e}).
\]

### 16.5 Touched-key kernel

不必形成整个 fixed-lag covariance。每个 action只接触少量 keys：

- UWB anchor：current pose；
- current IMU：pose/vel/bias at \(k,k+1\)；
- bridge：同一组 transition keys；
- historical action：其相邻 states。

`RankUpdateEvaluator` 应提取：

```text
protected keys
∪ removed-factor keys
∪ added-factor keys
```

的 joint covariance / Schur complement，或在完整 local window上维护一次 sparse factorization。

### 16.6 Dense oracle

新增 `DenseCandidateOracle`：

- 从同一 frozen window直接删除/添加 rows；
- dense QR/LLT refactorize；
-输出 state、covariance、statistic、PL。

unit tests要求 rank-update 与 oracle比较。

### 16.7 数值 gates

candidate无效条件：

```text
base version mismatch
inner downdate matrix not SPD
candidate information not SPD
rank loss
condition number exceeded
non-finite covariance
linearization step exceeded
candidate detector invalid
fault gram singular
risk allocation invalid
```

任何 invalid candidate不得被 winner selector使用。

### 16.8 关于即将发表的 rank-update工作

ION GNSS+ 2026 摘要报告了 Bayes-tree marginal + Sherman–Morrison/Woodbury 的高效多 hypothesis评估。V2 可借鉴其结构，但不得直接把摘要中的运行时间或 overbound结论当作本项目证据。必须自建 oracle 和 benchmark。

---

## 17. Ambiguity-safe FDE

### 17.1 Plausible hypothesis set

根据 profile evidence、fault prior 和 isolation threshold构造：

\[
\mathcal A(z)=
\{h:\text{data不能在分配风险下排除 }H_h\}.
\]

该集合不是“top-1 hypothesis”。

### 17.2 Action selection原则

首先寻找覆盖 plausible set 的 actions：

\[
\mathcal E_{\mathrm{cover}}=
\{e:\forall h\in\mathcal A(z),\;covers(e,h)\}.
\]

然后在满足：

- post-FDE detector pass；
- PL finite；
- monitorability；
- bridge validity；
- cardinality；
- runtime / health constraints

的 actions中选择最小信息损失或最小 PL 的 action。

推荐序：

1. 最小 exclusion cardinality；
2. 最小 post-FDE HPL/VPL；
3. 最大 retained information log-det；
4. deterministic tie-break by action ID。

### 17.3 无覆盖 action

若：

\[
\mathcal E_{\mathrm{cover}}=\varnothing,
\]

则：

```text
FDE_AMBIGUOUS
availability = UNAVAILABLE
```

可进入安全降级 plan，但不得宣称 fault attribution成功。

### 17.4 Union exclusion

例如 UWB_i 和 IMU 都 plausible：

```text
REMOVE_UWB_i_AND_CURRENT_IMU_ADD_BRIDGE
```

若该 combined action可监测且 PL合格，优先于任意 top-1硬猜。

### 17.5 Wrong exclusion risk

最终 risk tree必须覆盖：

\[
P_{\mathrm{WE}}
=
\sum_h\sum_{e\not\supseteq h}
P(H_h)P(E=e\mid H_h)
P(\mathrm{HMI}\mid h,e).
\]

V2 通过 plausible-set coverage将主要 wrong-exclusion event转化为：

- combined exclusion；
- ambiguity unavailable；
- 或显式 outcome risk allocation。

不得在日志中把 `selected_action` 等同于 `true_fault`.

---

## 18. Kinematic Bridge

### 18.1 作用

bridge同时服务于：

1. IMU exclusion后的 graph connectivity；
2. IMU fault isolation的 independent reference；
3. UWB/IMU joint candidate的可解性；
4. degraded navigation期间的显式 uncertainty growth。

### 18.2 独立性

generic bridge不得使用 current suspect IMU。可使用：

- previous trusted state；
- \(\Delta t\)；
- bounded maneuver assumptions。

dynamics bridge可使用：

- time-aligned control commands；
- thrust / attitude setpoint；
- mass / drag model；
- independently calibrated model envelope。

若 bridge读取 current IMU，它不能作为 IMU fault的独立替代。

### 18.3 Generic residual

建议拆成 pose/velocity bridge与 bias continuity：

\[
r_R=\Log(R_k^\top R_{k+1}),
\]

\[
r_p=p_{k+1}-p_k-v_k\Delta t,
\]

\[
r_v=v_{k+1}-v_k,
\]

\[
r_{b_a}=b_{a,k+1}-b_{a,k},
\]

\[
r_{b_g}=b_{g,k+1}-b_{g,k}.
\]

Pose3 translation tangent frame必须在实现文档中固定。不要混用 local/world translation Jacobian。

### 18.4 Optimization covariance 与 integrity model

```cpp
struct BridgeUncertainty {
  Eigen::MatrixXd optimization_covariance;
  BridgeIntegrityModel integrity_model;
  std::string calibration_id;
};
```

支持：

```text
GAUSSIAN_OVERBOUND
DETERMINISTIC_BOX
DETERMINISTIC_ELLIPSOID
```

### 18.5 Maneuver envelope

若：

\[
\|a\|_\infty\le a_{\max},
\]

则每轴 conservative bounds：

\[
|\Delta v|\le a_{\max}\Delta t,
\]

\[
|\Delta p|\le \frac12a_{\max}\Delta t^2.
\]

姿态可按：

\[
|\Delta\theta|
\le
\omega_{\max}\Delta t+
\frac12\alpha_{\max}\Delta t^2.
\]

这些是模型余项 bound，不是随意设置的小 sigma。

### 18.6 Bridge margin传播

若 bridge model error \(\epsilon_b\) 满足 box：

\[
|\epsilon_b|\le q_b,
\]

而 protected error map为 \(K_{e,b}\)，则：

\[
B_{e,j}
=
|c_j^\top K_{e,b}|q_b.
\]

若 ellipsoid：

\[
\epsilon_b^\top Q_b^{-1}\epsilon_b\le1,
\]

则：

\[
B_{e,j}
=
\sqrt{
c_j^\top K_{e,b}Q_bK_{e,b}^\top c_j
}.
\]

不得只把 raw \(0.5a_{\max}\Delta t^2\) 直接加到 HPL，而忽略状态耦合。

### 18.7 Quadrotor dynamics bridge

仿真器为 quadrotor 时可实现：

```text
QuadrotorDynamicsBridge
```

作为 optional model。它必须：

- 不使用 suspect IMU；
-使用控制输入与独立 timestamp；
-有 mass/drag/thrust uncertainty；
-输出自己的 model ID 与 integrity bound；
-与 generic bridge做 ablation。

真机缺少可靠控制输入时回退 generic bridge。

### 18.8 最大持续时间

配置：

```text
max_consecutive_bridge_epochs
max_bridge_duration_s
```

超限：

```text
BRIDGE_TIMEOUT
availability = UNAVAILABLE
controlled_reinitialization_required = true
```

禁止无限 CV dead reckoning或静默 reset。

---

## 19. Post-FDE slope-based PL

### 19.1 Candidate model

对 selected action \(e\)：

\[
z_e=H_e\delta X+A_{e,h}f_h+v_e.
\]

\[
K_e=H_e^\dagger,
\qquad
P_{\perp,e}=I-H_eH_e^\dagger.
\]

protected-axis selector \(c_j\)从 window state tangent映射到 current body-origin world position。

### 19.2 Worst-case slope

\[
g_{e,h,j}^2
=
c_j^\top K_eA_{e,h}
(A_{e,h}^\top P_{\perp,e}A_{e,h})^\dagger
A_{e,h}^\top K_e^\top c_j.
\]

仅在 fault gram通过 monitorability gate 时有限。

### 19.3 Detection-boundary noncentrality

\[
F_{\chi^2_{\nu_e,\bar\lambda_{e,h}^2}}
(\tau_e^2)
=
P_{\mathrm{MD},e,h}.
\]

数值求解必须：

- 单调 bracket；
- status return；
- 不允许 silent clamp；
- 与 SciPy/Boost oracle交叉验证；
- 记录 iterations 和 residual。

### 19.4 Nominal component

\[
\sigma_{e,j}^2
=
c_j^\top\Sigma_ec_j.
\]

\[
PL_{0,e,j}=k_{0,e,j}\sigma_{e,j}.
\]

### 19.5 Hypothesis component

\[
PL_{h,e,j}
=
g_{e,h,j}\bar\lambda_{e,h}
+
k_{h,e,j}\sigma_{e,j}
+
B_{e,h,j}.
\]

### 19.6 Final per-axis PL

\[
PL_{e,j}
=
\max\left(
PL_{0,e,j},
\max_{h\in\mathcal H_e}
PL_{h,e,j}
\right).
\]

其中 \(\mathcal H_e\) 必须包括：

- selected action后仍可能存在的 monitored faults；
- plausible但未被 action完全覆盖的 outcome；
- bridge model fault / escape；
- active history hypotheses。

### 19.7 Risk closure

必须验证：

\[
I_0
+
\sum_{e}\sum_h I_{e,h}
+
P_{NM}
+
P_{bridge}
+
P_{history}
+
P_{model}
\le
P_{HMI,REQ}.
\]

risk budget不能只检查 `sum(prior*p_md)`；还要检查 exclusion outcomes 与 model escape。

### 19.8 PL 与 availability

```text
candidate detector pass
AND risk budget valid
AND model valid
AND HPL <= HAL
AND VPL <= VAL
```

才是 `AVAILABLE`。

factor可被 commit但 navigation仍可 `UNAVAILABLE`。

---

## 20. Health Manager

### 20.1 状态

```text
HEALTHY
SUSPECT
QUARANTINED
RECOVERY_TEST
FAILED
```

适用于：

- each UWB anchor；
- accelerometer subsystem；
- gyroscope subsystem；
- generic bridge model；
- optional dynamics bridge。

### 20.2 转移

```text
HEALTHY
  └─ evidence/alarm → SUSPECT

SUSPECT
  ├─ selected/covered by FDE → QUARANTINED
  └─ cleared for N tests → HEALTHY

QUARANTINED
  └─ shadow residual passes recovery preconditions → RECOVERY_TEST

RECOVERY_TEST
  ├─ M consecutive pass → HEALTHY
  └─ fail → QUARANTINED
```

### 20.3 Quarantine semantics

quarantined source：

- 不进入正式 estimator；
-仍可产生 shadow residual；
-不从 hypothesis pool消失；
-恢复前不得自动重新提交；
-其 prior fault probability应按 health state调整，并记录 model ID。

### 20.4 IMU quarantine

IMU quarantine期间：

- bridge保持 transition；
-raw IMU继续缓存用于 recovery test；
-不得用 quarantined IMU产生 bridge；
-达到 timeout触发 controlled reinitialization或 no trusted solution。

---

## 21. GNC / robust estimation 的边界

### 21.1 正式路径

正式 V2 action是可审计的：

```text
KEEP
REMOVE
REPLACE_WITH_BRIDGE
```

每个 retained factor有固定 effective covariance。

### 21.2 Shadow GNC

可新增：

```text
GncShadowEvaluator
```

输出 continuous weights用于：

- candidate ordering；
-NLOS diagnosis；
-研究对比；
-恢复判定辅助。

不得直接改变 formal posterior。

### 21.3 正式采用 GNC 的条件

若未来需要 GNC进入正式 estimator：

1. freeze final weight；
2. log weight；
3.把 \(R_i/w_i\) 作为 effective covariance；
4.用该 covariance重建 joint detector；
5.用相同权重重算 fault map、rank update和 PL；
6.验证 data-dependent weighting的 tail overbound；
7.更新 formal scope。

否则不能发布 formal label。

---

## 22. 在线算法伪代码

```text
function processUwbEpoch(batch):

    tx = estimator.prepareEpoch(batch)
    assert backend.version == tx.base_version
    assert backend was not mutated

    window = snapshot_builder.build(tx, health, factor_ledger)
    all_in = joint_monitor.evaluate(window)

    evidence = hypothesis_evaluator.evaluate_all(
        window,
        hypothesis_generator.generate(window, health)
    )

    if hardware_barrier_triggered:
        candidate_actions = barrier_actions(...)
    else if all_in.detector_passed:
        candidate_actions = {KEEP_ALL}
    else:
        plausible = ambiguity_manager.plausible_set(evidence, risk)
        candidate_actions = action_generator.covering_actions(
            plausible, factor_ledger, bridge_factory
        )

    base_kernel = rank_update_evaluator.factorize_once(window)

    candidate_results = []
    for action in candidate_actions ordered deterministically:
        result = rank_update_evaluator.evaluate(base_kernel, action)
        result.post_detector = joint_monitor.evaluate_candidate(result)
        result.remaining_hypotheses =
            hypothesis_generator.after_action(action, window, health)
        result.pl = pl_monitor.compute(result, risk)
        candidate_results.push(result)

    decision = ambiguity_manager.select(candidate_results, evidence, risk)

    if not decision.commit_allowed:
        estimator.discardEpoch(move(tx), decision.reason)
        health.update(decision)
        logger.write(...)
        return unavailable_output

    receipt = estimator.commitEpoch(move(tx), decision.commit_plan)
    assert receipt.backend_updates == 1

    health.update(decision, receipt)
    ledger.update(receipt)
    logger.write(all_in, evidence, candidates, decision, receipt)

    return decision.integrity_output
```

### 22.1 Determinism

同一：

- raw input；
- resolved config；
- seed；
- platform floating-point contract；
- factor ordering

必须产生相同：

- hypothesis IDs；
- action ordering；
- winner；
- status；
- config hash；
- output schema。

---

## 23. 软件模块与文件布局

建议新增：

```text
include/uwb_imu_pl/
├── common/
│   ├── types.hpp                         # 保留通用 sensor/state types
│   └── integrity_ids.hpp                 # Transaction/Window/FaultUnit/Action IDs
├── estimation/
│   ├── incremental_estimator.hpp         # 重构 prepare/commit/discard
│   ├── epoch_transaction.hpp
│   ├── factor_ledger.hpp
│   ├── integrity_window_snapshot.hpp
│   └── rank_update_kernel.hpp
├── factors/
│   ├── realtime_factors.hpp
│   ├── kinematic_bridge_factor.hpp
│   └── quadrotor_dynamics_bridge_factor.hpp
├── integrity/
│   ├── integrity_monitor.hpp             # facade
│   ├── joint_window_detector.hpp
│   ├── fault_model.hpp
│   ├── imu_fault_subspace.hpp
│   ├── hypothesis_generator.hpp
│   ├── hypothesis_evidence.hpp
│   ├── exclusion_action.hpp
│   ├── fde_manager.hpp
│   ├── protection_level_v2.hpp
│   ├── risk_allocator_v2.hpp
│   ├── ambiguity_manager.hpp
│   └── health_manager.hpp
└── io/
    ├── run_logger.hpp
    └── schema_v4.hpp
```

source files对应创建在 `src/uwb_imu_pl/...`。

### 23.1 现有文件修改表

| 现有文件 | 修改 |
|---|---|
| `common/types.hpp` | 保留通用类型；移出 anchor-only hypothesis；扩展 availability/status，不无限膨胀 |
| `config/integrity_config.hpp` | 增加 window、FDE、IMU fault、bridge、health、risk outcome configs |
| `estimation/incremental_estimator.hpp/.cpp` | 事务化；backend add/remove；factor slots；不再在 `predictTo()` 提交 IMU |
| `integrity/estimation_snapshot.hpp` | 保持 V1兼容；新增独立 window snapshot类型 |
| `integrity/integrity_monitor.hpp/.cpp` | V1 snapshot保留；V2 facade调用 joint detector/FDE/PL |
| `factors/realtime_factors.hpp/.cpp` | 新增 bridge factors；公开可审计 Jacobians |
| ROS node | pipeline改为 transaction；发布 FDE/health/status |
| config loader | strict keys、resolved serialization、hash |
| logger | schema升级，新增 hypotheses/actions/health/bridge tables |
| simulator | raw IMU fault injection、cross-sensor faults、control input输出 |
| tests | 新增 transaction、rank update、IMU sensitivity、bridge、joint FDE suites |

### 23.2 兼容策略

保留旧 API 仅可作为：

```text
deprecated V1 adapter
```

其内部必须调用新 transaction API，不允许维护两套 backend mutation逻辑。

---

## 24. 关键接口草图

### 24.1 `JointWindowDetector`

```cpp
class JointWindowDetector {
 public:
  DetectorResultV2 evaluate(
      const LinearizedIntegrityWindow& window,
      const DetectorRiskContext& risk) const;

  CandidateDetectorResult evaluateCandidate(
      const CandidateLinearSolution& candidate,
      const DetectorRiskContext& risk) const;
};
```

### 24.2 `HypothesisGenerator`

```cpp
class HypothesisGenerator {
 public:
  std::vector<FaultHypothesisV2> generate(
      const LinearizedIntegrityWindow& window,
      const FactorLedger& ledger,
      const HealthSnapshot& health,
      const RiskBudgetV2& risk) const;

  std::vector<FaultHypothesisV2> afterAction(
      const ExclusionAction& action,
      const LinearizedIntegrityWindow& window,
      const HealthSnapshot& health,
      const RiskBudgetV2& risk) const;
};
```

### 24.3 `RankUpdateEvaluator`

```cpp
class RankUpdateEvaluator {
 public:
  BaseCandidateKernel factorizeOnce(
      const LinearizedIntegrityWindow& window) const;

  CandidateEvaluation evaluate(
      const BaseCandidateKernel& base,
      const ExclusionAction& action) const;
};
```

### 24.4 `FdeManager`

```cpp
class FdeManager {
 public:
  FdeDecision decide(
      const DetectorResultV2& all_in,
      const std::vector<FaultModeEvidence>& evidence,
      const std::vector<CandidateEvaluation>& candidates,
      const RiskBudgetV2& risk) const;
};
```

### 24.5 `BridgeFactory`

```cpp
class BridgeFactory {
 public:
  PendingFactorGroup makeGeneric(
      const EpochTransaction& tx,
      const BridgeConfig& config) const;

  std::optional<PendingFactorGroup> makeDynamics(
      const EpochTransaction& tx,
      const ControlInputBuffer& controls,
      const BridgeConfig& config) const;
};
```

---

## 25. 配置设计

建议把研究配置升级为：

```yaml
schema_version: uwb-imu-pl/v4
seed: 20260901

incremental:
  relinearize_threshold: 0.1
  relinearize_skip: 200
  fixed_lag_epochs: 200
  epoch_bin_s: 0.02
  max_time_skew_s: 0.01
  single_transaction_per_epoch: true

integrity_window:
  epochs: 20
  recovery_margin_epochs: 10
  boundary_method: bayes_tree_schur
  rank_tolerance: 1.0e-10
  max_condition_number: 1.0e10
  max_linearization_step_norm: 0.25
  require_history_provenance: true

detector:
  type: joint_window_residual_chi_square
  p_fa_per_test: 1.0e-6
  continuity_horizon_tests: 1000
  continuity_accounting: union_bound
  use_squared_norm_statistic: true

fault_models:
  max_cardinality: 2

  uwb:
    enabled: true
    epoch_single_anchor_bias: true
    persistent_anchor_bias: true
    ramp_bias: false
    prior_probability_bound: 1.0e-4
    p_md: 1.0e-3

  imu:
    accel_axis_interval_bias: true
    gyro_axis_interval_bias: true
    accel_xyz_interval_bias: false
    gyro_xyz_interval_bias: false
    bias_jump: false
    ramp_fault: false
    accel_prior_probability_bound: 1.0e-5
    gyro_prior_probability_bound: 1.0e-5
    p_md: 1.0e-3
    min_fault_gram_sigma: 1.0e-8
    max_fault_gram_condition: 1.0e10

  combinations:
    uwb_plus_accel: true
    uwb_plus_gyro: true
    two_uwb: false
    assume_independent_priors: false

fde:
  trigger: joint_detector_alarm_or_hardware_barrier
  isolation: profile_parity_glrt
  ambiguity_policy: union_exclusion_else_unavailable
  selection_primary: minimum_cardinality
  selection_secondary: minimum_protection_level
  dense_oracle_online_fallback: false
  max_candidate_count: 128

bridge:
  generic:
    enabled: true
    model: constant_velocity_constant_attitude
    optimization_sigma_position_m: 0.5
    optimization_sigma_velocity_mps: 1.0
    optimization_sigma_rotation_rad: 0.5
    acceleration_bound_mps2: 4.0
    angular_rate_bound_radps: 2.0
    angular_acceleration_bound_radps2: 4.0
    integrity_model: deterministic_box

  quadrotor_dynamics:
    enabled: false
    require_control_input: true
    model_calibration_id: ""
    integrity_model: deterministic_ellipsoid

  max_consecutive_epochs: 20
  max_duration_s: 1.0

health:
  suspect_evidence_count: 2
  quarantine_after_exclusion: true
  recovery_shadow_passes: 20
  recovery_test_passes: 10
  allow_auto_recovery: true

risk:
  p_hmi_total: 4.0e-5
  nominal_axis_tail: 1.0e-5
  p_nm: 1.0e-7
  p_bridge_escape: 0.0       # deterministic bound时为0
  p_history_contamination: 0.0
  p_model_escape: 0.0
  horizontal_alert_limit_m: 2.0
  vertical_alert_limit_m: 3.0
  allocation_policy: prior_weighted_outcome_conditioned

robust_shadow:
  gnc_enabled: false
  never_mutate_formal_weights: true

output:
  schema_version: uwb-imu-pl/v4
  write_factor_ledger: true
  write_window_rows: false
  write_hypothesis_evidence: true
  write_candidates: true
  write_health: true
  write_bridge: true
  write_timing: true
```

上面数值是 schema示例，不是已校准安全参数。loader必须拒绝未知 key、缺失安全字段和不闭合风险预算。

### 25.1 Cross-field validation

必须检查：

```text
fixed_lag_epochs == 0
OR
fixed_lag_epochs >
integrity_window.epochs + integrity_window.recovery_margin_epochs
```

以及：

```text
bridge.max_duration_s > 0
max_candidate_count >= required minimum
risk sum <= p_hmi_total
detector continuity allocation valid
all enabled fault priors present
all model IDs non-empty when required
```

---

## 26. 输出与审计 schema

建议 `RunManifest.schema_version = "uwb-imu-pl/v4"`。

### 26.1 `integrity.csv`

新增字段：

```text
transaction_id
window_id
base_graph_version
linearization_version
joint_statistic
joint_threshold
joint_dof
joint_passed
selected_action_id
selected_action_type
fde_status
pl_x pl_y pl_z hpl vpl
nominal_component_x/y/z
fault_component_x/y/z
bridge_component_x/y/z
availability
formal_eligible
risk_budget_valid
history_provenance_valid
backend_updates
```

### 26.2 `fault_hypotheses.csv`

```text
timestamp
hypothesis_id
fault_unit_ids
sensor_types
parameter_dimension
prior_bound
p_md_allocation
hmi_allocation
monitorability_rank
sigma_min
condition_number
slope_x slope_y slope_z
noncentrality_boundary
monitored
reason
```

### 26.3 `fde_candidates.csv`

```text
timestamp
action_id
removed_group_ids
added_group_ids
bridge_mode
covers_plausible_set
valid
rank
dof
statistic
threshold
post_passed
condition_number
hpl
vpl
information_logdet
selected
reason
evaluation_ms
```

### 26.4 `factor_ledger.csv`

```text
factor_id
group_id
backend_slot
sensor
kind
epoch_begin
epoch_end
source_measurement_ids
fault_unit_ids
lifecycle
health_at_commit
noise_model_id
model_id
commit_version
```

### 26.5 `health.csv`

```text
timestamp
source_id
source_type
previous_state
new_state
trigger
evidence
selected_action
consecutive_bridge_epochs
recovery_counter
```

### 26.6 `bridge.csv`

```text
timestamp
bridge_model_id
mode
dt
optimization_covariance_diag
integrity_bound
control_input_available
calibration_id
active
consecutive_epochs
timeout
```

### 26.7 `fault_truth.csv`

现有 UWB-only truth扩展为：

```text
fault_unit_id
sensor_type
fault_kind
axis
epoch_begin
epoch_end
step/ramp parameters
raw injection point
active
```

---

## 27. 仿真 fault injection

### 27.1 注入层级

IMU fault必须注入 raw IMU stream：

```text
simulated true motion
→ ideal specific force / angular rate
→ nominal sensor noise/bias
→ injected fault
→ published ImuMeasurement
→ real preintegration
```

禁止只修改 final preintegration residual；那会绕过时序、旋转耦合和 covariance传播。

### 27.2 UWB fault

支持：

- anchor step bias；
- ramp；
- intermittent NLOS；
- persistent bias；
- outage；
- simultaneous anchors；
- anchor + IMU fault。

### 27.3 IMU fault

至少：

- accel x/y/z step；
- gyro x/y/z step；
- 3D direction step；
- slow ramp；
- bias jump；
- saturation；
- dropout；
- timestamp gap。

### 27.4 Bridge stress trajectories

- stationary；
- straight constant velocity；
- circle；
- figure-eight；
- aggressive acceleration；
- braking；
- high yaw rate；
- simultaneous translation/rotation；
- control-model mismatch；
- no-control-input fallback。

### 27.5 Fault magnitude sweep

重点覆盖 detection boundary，而不仅是明显大 fault：

```text
0.1 × boundary
0.25 ×
0.5 ×
0.75 ×
1.0 ×
1.25 ×
2 ×
5 ×
```

---

## 28. 测试规范

### 28.1 Transaction tests

建议新建 `test_epoch_transaction.cpp`：

- `PrepareEpochDoesNotMutateBackend`
- `PrepareEpochDoesNotAdvanceCommittedEpoch`
- `EveryCandidateSharesSameVersion`
- `CommitEpochUpdatesBackendExactlyOnce`
- `DiscardEpochDoesNotRollbackBackend`
- `ImuQueueConsumedOnlyAtFinalize`
- `VersionMismatchFailsClosed`
- `AddRemoveFactorsAreAtomic`

### 28.2 Factor ledger tests

`test_factor_ledger.cpp`：

- factor ID/slot映射；
- removal lifecycle；
- marginalization lifecycle；
- provenance completeness；
- replay checkpoint；
- no stale slot reuse；
- deterministic serialization。

### 28.3 Rank update tests

`test_rank_update_evaluator.cpp`：

- remove one UWB row block vs dense oracle；
- remove anchor group；
- remove IMU block；
- remove IMU + add bridge；
- cross-sensor combination；
- random SPD systems；
- near-singular downdate；
- invalid SPD gate；
- covariance symmetry；
- protected covariance；
- state increment；
- candidate statistic；
- deterministic candidate ordering。

建议容差分层：

```text
well-conditioned relative error <= 1e-9
moderate-conditioned <= 1e-7
near-gate <= documented tolerance
```

### 28.4 IMU sensitivity tests

`test_imu_fault_subspace.cpp`：

- analytic vs raw finite difference；
- axis ordering；
- sign；
- frame；
- \(\Delta t\) scaling；
- nonzero bias；
- rotation coupling；
- block vs axis columns；
- ramp model。

### 28.5 Joint detector tests

`test_joint_window_detector.cpp`：

- nominal statistic central \(\chi^2\)；
- empirical false alarm；
- rank-based dof；
- correlated factor covariance；
- window length变化；
- boundary prior；
- no duplicate factor rows；
- overlapping-window operation-level false alarm；
- nonlinear step gate。

### 28.6 Hypothesis / FDE tests

`test_joint_fde.cpp`：

- single UWB correct coverage；
- single accel fault；
- single gyro fault；
- UWB+IMU；
- two plausible hypotheses；
- union exclusion；
- no covering action；
- unmonitorable hypothesis；
- max cardinality；
- wrong-exclusion fail-closed；
- post-FDE re-detection；
- post-FDE PL recomputation。

### 28.7 Bridge tests

`test_kinematic_bridge.cpp`：

- analytic Jacobians；
- CV exact case；
- bounded acceleration；
- rotation bound；
- bias continuity；
- bridge margin propagation；
- generic/dynamics selection；
- no suspect IMU dependency；
- timeout；
- controlled reinit event。

### 28.8 Health tests

`test_health_manager.cpp`：

- state transitions；
- quarantine；
- no immediate re-entry；
- shadow recovery；
- persistent step；
- intermittent fault；
- simultaneous sensor states；
- deterministic reset。

### 28.9 Monte Carlo

必须分别报告：

- \(P_{FA}\)；
- detection probability；
- correct exclusion；
- wrong exclusion；
- ambiguous exclusion；
- no-valid-candidate；
- PL coverage；
- HMI count；
- availability；
- bridge duration；
- p50/p95/p99/max timing。

零观察 HMI 不是零风险证明。报告必须包含样本数与置信上界。

---

## 29. Verification Gates（同一个 V2，不是版本切分）

### Gate A — Baseline freeze

证据：

- pin branch commit SHA；
-现有 tests、P2/P3报告、config hash；
- V1 numerical baseline；
- known failures清单。

### Gate B — Transactional estimator

通过条件：

- prepared epoch无 backend mutation；
- nominal output与旧路径在容差内等价；
-每 epoch exactly one update；
- fixed-lag/full-history均通过；
- timing无不可接受退化。

### Gate C — Window snapshot与factor ledger

通过条件：

- active graph rows无漏计/重复；
- boundary prior与dense graph oracle一致；
- provenance 100%；
- factor removal可追踪；
- marginalization boundary行为正确。

### Gate D — Rank-update kernel

通过条件：

- 所有 candidate类型与dense oracle一致；
- SPD/conditioning gates正确；
- p99候选评估满足时间预算；
- no explicit full inverse。

### Gate E — UWB FDE

通过条件：

- single-anchor step/ramp；
- ambiguity policy；
- post-FDE detector；
- post-FDE PL；
- wrong exclusion统计；
- nominal false exclusion；
- 8-anchor与低冗余几何。

### Gate F — IMU fault model

通过条件：

- analytic sensitivity；
- accel/gyro axis faults；
- window-length observability；
- monitorability maps；
- fault magnitude sweep；
- raw-stream injection。

### Gate G — Bridge

通过条件：

- graph connectivity；
- conservative envelope；
- stress trajectories；
- bridge PL component；
- timeout/reinit；
- optional dynamics bridge ablation。

### Gate H — Joint UWB+IMU FDE

通过条件：

- joint detector calibration；
- hypothesis pool；
- UWB+IMU combinations；
- union exclusion；
- outcome-conditioned risk closure；
- no hidden local-detector pruning。

### Gate I — Persistent fault / history recovery

通过条件：

- quarantine/recovery；
- active-window late factor removal；
- checkpoint/replay或maturity delay；
- marginalized contamination fail-closed。

### Gate J — Formal release gate

通过条件：

- risk budget closed；
- all schemas validated；
- deterministic replay；
- timing budget；
- complete raw inventory/checksums；
- no skipped tests；
- scope label与实际能力一致；
- independent review of equations and code.

---

## 30. 性能预算

UWB epoch period为 \(T_{\mathrm{epoch}}\)。建议总 p99 wall time：

\[
T_{\mathrm{V2,p99}}\le0.8T_{\mathrm{epoch}}.
\]

初始分配比例：

| Stage | p99 budget |
|---|---:|
| prepare/preintegration | 15% |
| window snapshot | 15% |
| base factorization + joint FD | 15% |
| hypothesis evidence | 10% |
| candidate rank updates | 20% |
| PL | 10% |
| one backend commit | 15% |

出现 alarm时 candidate数量增大；日志必须区分 nominal与FDE epochs。

优化顺序：

1.避免 full graph residual；
2. touched-key selected solves；
3. base factorization复用；
4. block rank updates；
5. candidate parallelism仅在 deterministic reduction可保证时开启；
6.不以减少 monitored hypotheses换时间，除非 risk转入 `p_nm`。

---

## 31. 开发工作包

这些工作包共同组成一个 V2，接口从第一天按最终架构设计。

### WP-1：类型、transaction、ledger

交付：

- IDs；
- `EpochTransaction`；
- `FactorLedger`；
- backend add/remove wrapper；
- strict version contracts。

### WP-2：window snapshot

交付：

- active factor linearization；
- boundary prior；
- window layout；
- no-double-count oracle。

### WP-3：joint detector与hypothesis model

交付：

- QR/SVD parity；
- UWB fault map；
- IMU sensitivity；
- monitorability；
- profile evidence。

### WP-4：rank-update FDE

交付：

- base kernel；
- action generator；
- block downdate/update；
- dense oracle；
- ambiguity manager。

### WP-5：bridge与health

交付：

- generic bridge；
- optional dynamics bridge；
- integrity bound propagation；
- quarantine/recovery；
- timeout。

### WP-6：PL与risk closure

交付：

- post-FDE slopes；
- outcome-conditioned hypotheses；
- nominal/fault/bridge components；
- availability；
- risk audit。

### WP-7：simulator、logger、formal evidence

交付：

- raw IMU faults；
- cross-sensor cases；
- schema v4；
- MC；
- timing；
- report generator；
- final gate。

---

## 32. Pull request约束

每个 PR：

- 不得混合数学语义变化与大规模格式化；
-必须包含 tests；
- config schema变化必须更新 strict loader tests；
- log schema变化必须更新 validator；
-新增 fault model必须更新 risk closure；
-新增 action必须更新 ambiguity tests；
-新增 bridge model必须提供 envelope证据；
-不得通过降低阈值/扩大 sigma只让测试“变绿”；
-不得删除旧 formal gate证据；
-不得把 forthcoming abstract中的结果写成已复现事实。

---

## 33. Definition of Done

V2 完成必须同时满足：

### Architecture

- [ ] current IMU/UWB在决策前均 pending；
- [ ] one transaction / epoch；
- [ ] factor ledger完整；
- [ ] joint window detector；
- [ ] structured UWB/IMU hypotheses；
- [ ] low-rank candidate evaluation；
- [ ] bridge replacement；
- [ ] ambiguity-safe FDE；
- [ ] post-FDE slope PL；
- [ ] health quarantine；
- [ ] history contamination handling。

### Mathematics

- [ ] detector dof/rank正确；
- [ ] fault maps verified；
- [ ] finite slope only under monitorability；
- [ ] candidate covariance与oracle一致；
- [ ] outcome risk闭合；
- [ ] bridge bound传播正确；
- [ ] overlapping-window continuity解释完整。

### Evidence

- [ ] all tests 0 failures / 0 skipped；
- [ ] nominal false alarm evidence；
- [ ] UWB, accel, gyro, cross-sensor MC；
- [ ] wrong exclusion与ambiguity报告；
- [ ] PL coverage / HMI上界；
- [ ] bridge stress；
- [ ] p50/p95/p99/max timing；
- [ ] deterministic replay；
- [ ] raw inventory/checksums；
- [ ] commit SHA/config hash；
- [ ] independent review。

---

## 34. 开发前必须与导师确认的问题

1. **IMU fault scope**  
   axis step、3D block、ramp、bias jump、scale factor、time sync分别是否在正式 claim内？

2. **多故障 cardinality**  
   必须覆盖 UWB+IMU吗？是否覆盖 two-UWB？

3. **fault priors来源**  
   来自 datasheet、实验统计还是保守上界？是否允许独立性假设？

4. **window length与alert time**  
   允许多大 detection latency？slow ramp检测和实时告警怎么权衡？

5. **bridge平台模型**  
   真机是否有独立 control input？若没有，是否接受 generic kinematic envelope？

6. **history recovery**  
   采用 maturity delay、factor removal、checkpoint replay中的哪一种？

7. **wrong-exclusion policy**  
   是否接受 ambiguity时 union exclusion和availability下降？

8. **formal protected state**  
   仅 body-origin position，还是还需要 attitude / antenna position？

9. **operation continuity horizon**  
   \(P_{FA}\) 是 per epoch、per trajectory还是per mission？

10. **availability vs estimator continuity**  
    unavailable期间是否仍输出 best-effort state？下游接口如何标识？

这些决定必须形成 ADR，并进入 resolved config或scope manifest。

---

## 35. 已知研究风险

### 35.1 IMU fault可观测性

gyro fault可能在短窗或低机动时不可监测，导致 infinite slope / unavailable。这不是软件 bug。

### 35.2 Bridge conservatism

generic CV bridge若 envelope合理，PL可能快速增长；若 PL不增长，反而应怀疑模型过度自信。

### 35.3 Late fault污染

任何在 marginalization后才识别的 fault都可能污染 boundary prior。必须有 recovery机制或明确 fail-closed。

### 35.4 Nonlinearity

所有 rank updates对 frozen linear model是精确的，但对原 nonlinear graph只是一阶近似。linearization gate和dense/nonlinear oracle不可省略。

### 35.5 Data-dependent robust weighting

GNC可能提高 accuracy，却使 formal distribution与selection event复杂。V2不应偷偷把它当作完整性证明。

### 35.6 Risk calibration

没有 fault priors、noise overbound和bridge model escape证据，就只能是 research integrity estimate，不能提升正式标签。

---

## 36. 文献与来源定位

### 36.1 当前代码与内部设计

- `Printeger/uwb-imu-fusion`, branch `feature/realtime-uwb-imu-pl`, observed 2026-09-04.
- 当前 `incremental_estimator.hpp/.cpp`、`integrity_monitor.hpp/.cpp`、`types.hpp`、`estimation_snapshot.hpp`、research YAML。
- 项目讨论与 Claude 批判意见：`v2.md`。

### 36.2 Slope / residual RAIM

- Bruvik, Valentin, Schlichting, Walker, Kochenderfer, *Protection Levels for Vision-Based Pose Estimation*, 2026.  
  关键用途：arbitrary monitored fault subspace、worst-case failure-mode slope、noncentral \(\chi^2\) boundary、fault + nominal PL；论文明确将 FDE留为后续工作。

- Joerger, Chan, Pervan, *Solution Separation versus Residual-Based RAIM*, NAVIGATION, 2014.

- Joerger, Pervan, *Fault Detection and Exclusion Using Solution Separation and Chi-Squared ARAIM*, IEEE TAES, 2016.

- Blanch, Walter, Enge, *Protection Levels after Fault Exclusion for Advanced RAIM*, NAVIGATION, 2017.

### 36.3 FGO / multi-sensor integrity

- Tian et al., *IM-GIV: an effective integrity monitoring scheme for tightly-coupled GNSS/INS/Vision integration based on factor graph optimization*, arXiv:2410.22672, 2024.  
  用途：FGO residual linearization中同时考虑 pseudorange、IMU preintegration和vision fault。

- Jiang et al., *Innovation-based Kalman filter fault detection and exclusion method against all-source faults for tightly coupled GNSS/INS/Vision integration*, GPS Solutions 28:108, 2024.  
  用途：sliding-window innovations与多传感器 fault separation。

- Zhang et al., *MR-ULINS: A Tightly-Coupled UWB-LiDAR-Inertial Estimator with Multi-Epoch Outlier Rejection*, IEEE RA-L 9(12), 2024.  
  用途：UWB multi-epoch consistency的工程邻近工作；不提供本文所需 formal PL。

### 36.4 Rank update / GNC

- Hu, Wang, Wen, *Extending ARAIM to Incremental Smoothing-Based GNSS/IMU Fusion: Efficient Protection Level Computation via Rank Updates*, ION GNSS+ 2026 scheduled abstract.  
  状态：在本文档日期尚未正式报告；仅作为结构启发。

- Wen et al., *GNSS Outlier Mitigation Via Graduated Non-Convexity Factor Graph Optimization*, IEEE TVT / arXiv:2109.00667.  
  用途：continuous weighting作为robust estimation参考，不等于 formal integrity。

- Wen et al., *Unified Fault Detection, Exclusion, and Integrity Monitoring ... via Graduated Non-Convexity*, ION GNSS+ 2026 scheduled abstract.  
  状态：future conference abstract；不得作为已验证结论。

### 36.5 Bounded fallback / set-membership

- Calafiore, *Reliable Localization Using Set-Valued Nonlinear Filters*, IEEE TSMC-A, 2005.  
  用途：bounded process/measurement uncertainty、incompatible observation rejection、carry-over prediction set。

- Zhu et al., *Safety-critical LiDAR-inertial odometry with on-manifold deterministic protection level*, 2026.  
  用途：fallback/degeneracy时保守传播与 deterministic envelope的设计原则。

### 36.6 GTSAM API

- GTSAM 4.2 `ISAM2::update(... removeFactorIndices ...)`。
- GTSAM fixed-lag smoother update with factors-to-remove。
- `PreintegratedCombinedMeasurements` 与 preintegration bias Jacobians。

---

# Appendix A — 建议状态机

```text
                    ┌────────────┐
                    │  NOMINAL   │
                    └─────┬──────┘
                          │ joint alarm / barrier
                          ▼
                    ┌────────────┐
                    │ EVALUATING │
                    └─────┬──────┘
             ┌────────────┼─────────────┐
             │            │             │
             ▼            ▼             ▼
      FDE_UWB        FDE_IMU       FDE_MULTI
             │            │             │
             └────────────┼─────────────┘
                          ▼
                 post-FDE FD + PL
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
         AVAILABLE              UNAVAILABLE
              │                       │
              ▼                       ▼
           COMMIT             DEGRADED / REINIT
```

另行维护 sensor health，不把 navigation mode和sensor health混为一个枚举。

---

# Appendix B — 推荐 FDE statuses

```text
NOT_TRIGGERED
SUCCESS_KEEP_ALL
SUCCESS_UWB_EXCLUSION
SUCCESS_IMU_EXCLUSION_GENERIC_BRIDGE
SUCCESS_IMU_EXCLUSION_DYNAMICS_BRIDGE
SUCCESS_MULTI_SENSOR_EXCLUSION
AMBIGUOUS_UNION_EXCLUSION
AMBIGUOUS_UNAVAILABLE
NO_VALID_CANDIDATE
FAULT_MODE_UNMONITORABLE
HISTORY_PRIOR_CONTAMINATED
BRIDGE_TIMEOUT
MODEL_INVALID
RISK_BUDGET_INVALID
BACKEND_VERSION_MISMATCH
CONTROLLED_REINITIALIZATION_REQUIRED
```

---

# Appendix C — 最小开发顺序

```text
1. Pin baseline and add handbook/ADR framework
2. Introduce IDs, EpochTransaction, FactorLedger
3. Refactor backend to one add/remove update
4. Prove nominal equivalence
5. Build LinearizedIntegrityWindow and dense oracle
6. Implement joint detector
7. Implement UWB fault maps
8. Implement IMU sensitivity and finite-difference oracle
9. Implement monitorability
10. Implement profile evidence / plausible set
11. Implement rank-update candidate evaluator
12. Implement generic bridge + bound propagation
13. Implement action coverage / ambiguity manager
14. Implement post-FDE slopes / PL / risk audit
15. Implement health/quarantine/history recovery
16. Extend simulator and schemas
17. Run all verification gates
18. Only then change formal label
```

---

# Appendix D — 禁止事项清单

- 不得在 `prepareEpoch()` 调 `backendUpdate()`。
- 不得为每个 candidate clone/re-solve完整 iSAM2。
- 不得从 full-set whitening后的 rows直接删除子行而不重建对应 covariance block。
- 不得按 raw residual最大值直接排 fault。
- 不得把 arbitrary 15D IMU fault声称为可监测。
- 不得用 suspect IMU构造其替代 bridge。
- 不得把 CV factor设置得不合理地精确。
- 不得忽略 multiple valid candidates。
- 不得在 post-FDE沿用 pre-FDE covariance、dof、threshold或slope。
- 不得在 fault进入marginal prior后仍宣称 history trusted。
- 不得让 GNC estimator与formal PL使用不同隐藏权重。
- 不得用“0次HMI”表述“零完整性风险”。
- 不得把 future conference abstract写成已复现结果。
- 不得在 PL超AL时自动把健康 factor当fault删除。
- 不得静默 reinitialize。
