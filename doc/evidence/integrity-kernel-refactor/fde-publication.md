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
