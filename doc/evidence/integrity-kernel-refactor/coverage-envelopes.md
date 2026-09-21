# 覆盖包络与覆盖证书（工作包 B2 / 路线图 §5.7–§5.8）

本轮（P4 = B2）在 **零阈值/零算法语义改动** 的前提下完成三件事：

1. **单双故障同 registry**：order=1 与 order=2 走同一引擎，双故障只在被引用的
   (mode, mode) 对上计算交叉项，并且拼接前必须通过参数独立性守卫（族支持、共享参数、
   共享方向）——失败即 fail-closed。
2. **紧凑模式与容量受限缓冲**：每个 mode 只在其实际写入的 factor group 行上存储
   （紧凑描述符 + 紧凑块 + `HᵀA` + `Aᵀparity`），交叉块由行跨度求交得到；窗口级
   zero-padded 矩阵只在冻结 Qᵀ 审计输入需要时（或容量回退时）物化。
3. **精确遍历 + 分组包络 + 覆盖证书**：在线遍历是精确遍历（无 top-K）；分组包络是
   可选容量路径，必须同时通过**包含性证明**与**支配性义务**才被接受；标签
   `EXACT` / `UPPER_ENVELOPE` / `UNCOVERED` 与 `coverage_envelope_id` 随诊断 v13
   导出。

## 1. 契约与不变量

| 项 | 本轮结论 | 证据 |
|---|---|---|
| 在线遍历 | 精确遍历，registry 顺序，无 top-K、无截断 | `buildExactCoverageCertificate()`；`B2Coverage.ExactTraversalCertifiesEveryTask` |
| 双故障拼接 | 仅当 `pairFamilySupport != Unsupported` 且参数块结构独立时允许；否则 `monitored=false`、`plausible=false`、`z_classification=3` | `B2Registry.*`（4 例） |
| 交叉项 | 按需：只对假设引用的 (mode,mode) 对计算，且在 worker pool 之前单线程预计算（缓存只读） | `B2Registry.CrossBlocksAreRequestedOnlyWhereNeeded`；场景计数 |
| 紧凑存储 | 假设级乘积（Gram 交叉块、score、`HᵀA`）全部来自紧凑块，逐模式 padded 分配 = 0 | `B2Compact.CompactPathAllocatesNoPaddedModeBlocks`；`mode_dense_allocations=0`（7 场景） |
| 容量上限 | 超限回退到 padded 形式并**计数**（`compact_capacity_fallbacks`）；假设维数超限 fail-closed 并计数（`hypothesis_capacity_refusals`）；绝不静默增长 | `B2Compact.CapacityExhaustionFallsBackWithIdenticalResults`、`B2Compact.HypothesisDimensionCapRefusesFailClosed` |
| 存储形式不改结果 | 紧凑与 padded 回退逐位相同（统计量、估计故障、Gram 差 = 0） | 同上（断言 `maxAbsDiff == 0.0`） |
| 包络包含性 | `A_leaf = A_group · T` 必须在容差内成立，proof 记录 `T` 与相对残差 | `B2Coverage.GroupedEnvelopeDischargesProofAndDominance`；`B2Coverage.MismatchedModesAreRejectedByTheInclusionProof` |
| 包络支配性 | 叶参数化下的包络界必须 ≥ 叶自身精确界（否则拒绝，保留 EXACT） | `B2Coverage.DominanceRejectsAnUnderCoveringEnvelope` |
| 覆盖不完备 | `complete=false` ⇒ `reason="coverage incomplete: protected output unavailable"`，叶标 `UNCOVERED`，不许静默使用 | `B2Coverage.IncompleteCoverageMakesProtectedOutputUnavailable` |
| 标签导出 | 诊断 v13：`hypotheses.csv` 追加 `coverage_label,coverage_envelope_id` | 场景 5,930 行全部 `EXACT,0`；oracle O8g–O8i |

## 2. 分组包络的构造与两条义务

**包含性证明**：对候选组 mode `g` 与叶 mode `l`，用紧凑块上的正规方程求
`T = argmin ‖A_l − A_g·T‖`，随后在**窗口实际行**上重算 `A_g·T` 并比较最大绝对残差
（相对 `max|A_l|`）。只有 `relative_residual ≤ inclusion_tolerance` 才接受
（默认 1e-9）。`T` 与残差写入 proof，供 oracle/审计复核。

**支配性义务**：叶自身的精确界
`B_leaf = max_i sqrt(g_i · Γ_l⁺ · g_iᵀ)`（`Γ_l = A_lᵀ M A_l`，`g_i` 为受保护响应
行，求逆用秩揭示伪逆）与"包络在叶参数化下的界"
`B_env = max_i sqrt((g_i T) · (Tᵀ Γ_g T)⁺ · (g_i T)ᵀ)` 比较，要求
`B_env + dominance_tolerance ≥ B_leaf`。拒绝时 `accepted=false`、
`dominance_margin` 记录最坏余量并给出 reason；叶保持 `EXACT`。

> 注：斜率对参数缩放不变（`A → cA` 时 `g → cg`、`Γ → c²Γ`），因此**标量倍数**的
> 组方向总是精确满足两条义务；支配性检查实际拦住的是 **方向不匹配/退化** 的组
> （例如零 map 在宽松包含容差下"通过"证明，但界为 0 < 叶界）。这正是
> `B2Coverage.DominanceRejectsAnUnderCoveringEnvelope` 的构造。

## 3. GEO-05：双故障相消（交叉项不可省）

`test/test_integrity_reference.cpp::ReferenceFixture.GEO05DoubleFaultCancellationNeedsTheCrossTerm`

- 构造：`H`(12×3) 满列秩，`C=[0|R]` 形式的两行受保护映射；`d2 = H·w − d1`
  ⇒ 双故障列加在**同一窗口**上（`A=[d1|d2]`），`A·(1,1) = H·w` 在奇偶空间相消。
- 断言：单故障斜率有限且 `Γ_ii>0`；组合 Gram 奇异（`rank<2`）；盲方向
  `‖Z·v‖<1e-8` 但 `‖G·v‖>1e-3`（危险）；把 `Γ_12` 置零后最小奇异值 > 1e-3
  （**丢交叉项会掩盖危险组合**）；秩截断的精确界仍有限（已知限制，交由包络覆盖，
  已在 `validation-manifest.json` 的 GEO-05 note 中记录）。
- 该用例与 `B2Coverage.DominanceRejectsAnUnderCoveringEnvelope` 互补：前者证明
  交叉项必须保留，后者证明包络必须以支配性证明兜住秩截断之外的组合。

## 4. 场景计数（stage3 最终运行，7 场景）

| 场景 | fault_mode_columns | fault_cross_blocks | cross_block_cache_hits | all_mode_gram_columns | mode_dense_allocations | compact_mode_rows | padded_equivalent_rows | fallback / refusal |
|---|---|---|---|---|---|---|---|---|
| A_nominal | 5,930 | 5,930 | 5,930 | 0 | 0 | 139,110 | 1,407,120 | 0 / 0 |
| C_uwb_fde | 5,920 | 5,920 | 5,920 | 0 | 0 | 138,315 | 1,403,420 | 0 / 0 |
| D_imu_bridge | 5,924 | 5,924 | 5,924 | 0 | 0 | 139,020 | 1,405,602 | 0 / 0 |
| E_union | 5,924 | 5,924 | 5,924 | 0 | 0 | 139,020 | 1,405,602 | 0 / 0 |
| F_ramp_unmonitorable | 10,090 | 8,130 | 8,130 | 0 | 0 | 237,670 | 2,400,760 | 0 / 0 |
| G_continuous_rejection | 4,950 | 4,950 | 4,950 | 0 | 0 | 79,770 | 673,860 | 0 / 0 |
| H_mature_union | 52,186 | 52,186 | 52,186 | 0 | 0 | 1,281,398 | 13,109,888 | 0 / 0 |

读法：

* `fault_cross_blocks == fault_cross_block_cache_hits`：全部交叉块都在预计算阶段
  命中，热循环没有发生任何"未请求"重算（B2 第一版曾因多线程写共享 map 崩溃，见
  runbook §7 与本节 §5）。
* `F_ramp_unmonitorable` 的 `fault_cross_blocks (8,130) < fault_mode_columns
  (10,090)`：未被任何假设引用的 mode 不产生交叉块——**按需**而非全量 Gram。
* `compact_mode_rows` 只统计实际写入的行（模式触及的 group 行数之和），
  `padded_equivalent_rows = 窗口行数 × Σq`。七个场景的整体压缩比约 **10.1×**
  （139,110 → 1,407,120 为 10.1；H 为 10.2）。
* `mode_dense_allocations = 0` 表示紧凑路径没有逐模式 padded 分配；
  `B2Compact.LegacyMappedRouteCountsPaddedAllocations` 证明旧映射路径下该计数为
  正（诚实计量，而不是把开销藏起来）。

## 5. 存储形式等价（紧凑 vs padded 回退）

`B2Compact.CapacityExhaustionFallsBackWithIdenticalResults` 在同一合成窗口上跑
两次：默认（紧凑）与 `max_total_compact_rows=1`（强制回退）。断言包括
`all_in_statistic`、`conditioned_statistic`、`log_evidence`、`plausible`、
`estimated_fault`、`fault_gram` **逐位相等**（差的最大绝对值 = 0.0）。回退路径的
计数：`compact_capacity_fallbacks=1`、`mode_dense_allocations=2`（每个有效模式一次，
行数 = 2 × 窗口行数）。

场景级等价（`equivalence_compare.py`，baseline = B1/P3 运行）：

| 场景 | 文件 | 离散差异 | 路由差异 | 数值失败 | worst_numeric_rel |
|---|---|---|---|---|---|
| A_nominal | 20 | 0 | 0 | 0 | 0.0 |
| C_uwb_fde | 20 | 0 | 0 | 0 | 0.0 |
| D_imu_bridge | 20 | 0 | 0 | 0 | 0.0 |
| E_union | 20 | 0 | 0 | 0 | 0.0 |
| F_ramp_unmonitorable | 20 | 0 | 0 | 0 | 0.0 |
| G_continuous_rejection | 20 | 0 | 0 | 0 | 0.0 |
| H_mature_union | 20 | 0 | 0 | 0 | 0.0 |

`metadata_differences` 仅来自两处**有意的**元数据变化：诊断 schema
`v12 → v13`（每次 attempt 一行）与 `context_bytes`（上下文结构体新增字段）。两者
都被等价工具归类为元数据，不计入离散差异；`hypotheses.csv` 新增两列同样以
`column_mismatch` 记录，共享列仍逐列比较。

## 6. 独立 oracle（coverage 部分）

`tools/context_oracle.py`（`UWB_IMU_PL_ORACLE_RUNS` 指向 stage3 运行）：

| 项 | 内容 | 结果 |
|---|---|---|
| O8a–O8f | 既有 square-root 上下文项（rank/dof、R 对角、statistic、detector-only 行、parity 恒等、受保护协方差 PSD） | 5 个 fixture 全部 PASS |
| O8g | coverage 标签存在且落在 `{EXACT, UPPER_ENVELOPE, UNCOVERED}` 内 | PASS（4 个有假设审计的 bin；H 场景未导出假设审计 → NOT_RUN 并给出原因） |
| O8h | 标签/id 一致性：`EXACT→0`、`UPPER_ENVELOPE→≥1`、`UNCOVERED→0` | PASS |
| O8i | 已认证窗口中不允许出现 `UNCOVERED`（覆盖不完备即不可用） | PASS |
| O8j | 包络包含恒等式与支配性 | **NOT_RUN**：`T` 与支配界是冻结窗口内部量，不在 replay schema 中；由 C++ 测试 `B2Coverage.*` 与 GEO-05 fixture 覆盖 |

汇总：**42 PASS / 0 FAIL / 8 NOT_RUN**（5 个保留 fixture × 10 项；NOT_RUN 均带原因）。

## 7. 明确未做（NOT_RUN，按指令保留）

| 项 | 原因 |
|---|---|
| §5.9 风险并集界（B3） | 本轮不做：包络只做"覆盖 + 支配性"，不做预算级风险并集 |
| 惰性 FDE / 历史摘要 / FDE 选择合同 | C 阶段范围 |
| 性能收口 | D 阶段范围；本轮只报计数与观测 |
| 在线启用分组包络 | 保持精确遍历为在线默认；包络路径已实现并经测试，但启用会改变导出标签，属显式行为变更，留待与 B3 风险账本一起做 |
| GEO-03（K>ν 不统一拒绝） | 仍为 NOT_STARTED，note 记录 B2 输入（K=10→236 假设/帧；交叉块按需） |
| 时间耗尽的在线提前停止 | D 阶段；本轮只做容量上限（内存/维数）与 fail-closed |

## 8. 复现命令

> 注：`validation-report.json` 的 `run_sha = 9d5550f`（P3 提交）——报告是在 B2 改动
> 位于工作树、尚未提交时生成的；`base_sha` 仍为 A 轮基线 `2772b3a`。B2 自身的提交号
> 记录在本批最终报告的 checkpoint 行与 `hashes-B2.txt` 的头部说明中。

```bash
# 构建 + 全量测试（284 tests / 0 fail）
cd /home/mint/ws_fusion_uwb && source /opt/ros/noetic/setup.bash
catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl
catkin_test_results build/uwb_imu_pl/test_results/uwb_imu_pl

# 场景（stage3）
B=/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl
CFG=$B/config/realtime_uwb_imu_pl_research.yaml
SCN=$B/config/r0_r1_development_scenarios.yaml
O=/tmp/uwb_imu_pl_b2_20260921/stage3
for spec in "A_nominal 30" "C_uwb_fde 30" "D_imu_bridge 30" "E_union 30" \
            "F_ramp_unmonitorable 30" "G_continuous_rejection 45"; do
  set -- $spec; mkdir -p $O/$1
  UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \
      $CFG $O/$1 $2 $1 $SCN > $O/$1/stdout.log 2>&1
done
UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \
    $CFG $O/H_mature_union 226 H_mature_union $SCN

# 等价 + oracle + schema
export UWB_IMU_PL_EQUIVALENCE_BASELINE=/tmp/uwb_imu_pl_b2_20260921/baseline_b1
export UWB_IMU_PL_EQUIVALENCE_CURRENT=$O
python3 $B/doc/evidence/integrity-kernel-refactor/tools/equivalence_compare.py
export UWB_IMU_PL_ORACLE_RUNS=$O
python3 $B/doc/evidence/integrity-kernel-refactor/tools/context_oracle.py
python3 $B/tools/validate_run_schema.py $O/A_nominal
python3 $B/tools/gate_d_diagnostics.py $O/A_nominal
```
