# 诊断 fixtures 与 diagnostics v10 字段盘点（A2）

## 1. 五类保留帧（输入 + 影子诊断）

机器可读清单：`fixtures/frames.json`（每帧含 bin 路径/sha256、窗口头、v10 各表检查状态、
首因、全部检查状态、三阶梯代数；bin 二进制本体在 `raw/replay/<run>/exports/attempt-*.bin`）。

| 类 | 帧 | 输入（bin） | 实际后果（实测） |
|---|---|---|---|
| 普通帧 | A_nominal ep30 | `raw/replay/A_nominal/exports/attempt-30.bin` | KEEP_ALL；detector pass（parity=0≤166）；backend 提交；PL 有限（hpl=2.885>2m，vpl=1.999）→ availability UNAVAILABLE（PL 超限 + Gate J 未完成） |
| 报警帧 | C_uwb_fde ep25（anchor 1 +2.25 m） | `raw/replay/C_uwb_fde/exports/attempt-25.bin` | parity=419>166；KEEP_ALL 被判不覆盖 plausible 集；union 排除动作被选中并提交（fde=SUCCESS_UWB_EXCLUSION） |
| 当前 inf 帧 | C_uwb_fde ep26 | `raw/replay/C_uwb_fde/exports/attempt-26.bin` | detector pass（parity=0）但**无任何 eligible 候选覆盖 plausible 集** → best-effort 提交、PL 保持 inf、reason=AMBIGUOUS_UNAVAILABLE；`not_evaluated=[post-FDE detector/PL 全未评估]` |
| 缺测/拒批帧 | G_continuous_rejection ep10（全 anchor +5 m） | `raw/replay/G_continuous_rejection/exports/attempt-10.bin` | 唯一候选被 0.25 步长 gate 拒绝（`certified step exceeds gate; final Jacobian not built`）→ epoch discard、无 UWB 提交（缺测流的可观测等价物） |
| 边缘化帧 | H_mature_union ep201（首个固定滞后边缘化历元） | `raw/replay/H_mature_union/exports/attempt-201.bin` | 边缘化事件 `FIXED_LAG_MARGINALIZE`，窗口照常（253×165，boundary 块承担历史名义信息），KEEP_ALL 提交、PL 有限 |

**关于“缺测帧”的诚实说明**：开发运行器不产生“整批缺失/超时”帧（没有丢包注入），
可执行的等价物是**被拒批、无 UWB 提交的历元**（G 场景 5..40 历元），本轮用它并提供同一窗口输入。
真正的 dropout 帧需要回放/仿真侧输入（simulator 或新测试），列入 A4 输入清单。

## 2. 影子诊断（离线，本轮新增工具）

- 工具：`tools/replay_io.py`（独立重实现 `uwb-imu-pl/frozen-candidates/v4` 二进制编解码，不调用生产代码）
  + `tools/analyze_frames.py`（普查 + 首因/全检查状态 + 三阶梯）。
- 首因判定顺序：stage 级 FAIL → candidate 数值拒绝（skip/fallback reason）→ coverage 非覆盖结果
  → 发布态 reason。**未评估的检查显式写 `NOT_EVALUATED`**（如 C ep26 的 post-FDE 全链路）。
- 全部检查状态：24 个阶段逐一 PASS/SKIP_WITH_REASON/FAIL/NOT_EVALUATED + 每候选 5 个布尔 + 每动作覆盖结论。
- 三阶梯（固定 H/C/噪声/风险，不改变声明）：
  `one_dim_instant`（当前历元单测量行）／`template_epoch_independent`、`template_persistent`、
  `template_family_union`（生产模板族，含 2 参数并集）／`source_free_profile`（窗口内每历元独立参数）。
  本轮结果：**五帧的三个层级在冻结 H 下全部可监测（rank(Z)=q，σ_min≥7.4，cond≤12），
  失败都发生在下游**（候选 eligibility/覆盖或 0.25 步长 gate），不是故障方向代数本身。
  这只是“哪一步先失败”的定位，不构成原因证明。
- 交叉校验：C#25 one_dim σ_min=9.100/模板族 8.687 与生产 `hypotheses.csv`
  （epoch-independent onset30=9.10271 等）在同一量级一致性检查通过；ladder 与生产对
  **同一行集**的 Γ 结果一致（见 `fixtures/frames.json.production_hypothesis_crosscheck`）。

## 3. diagnostics v10 已覆盖字段（盘点）

来源：`src/uwb_imu_pl/io/run_logger.cpp`（`uwb-imu-pl/gate-d-diagnostics/v10`）+ `AttemptDiagnostics` 等结构。

| 表 | 已覆盖 |
|---|---|
| `diagnostic_attempts.csv` | 身份（attempt/transaction/window/4 版本/时间戳）、`frozen_group_ids`、backend epoch 前后、pending/state_age、raw_imu_samples、连续拒批、marginalization、四类 cache 计数与容量、`base_step_norm/selected_step_norm`、risk 分解（nominal/p_nm/bridge/history/model/hypotheses/total/margin）、五类假设计数、`generated_actions/kernel_evaluated/post_passed/pl_evaluated/selected`、全部数值工作计数（svd/llt/ldlt/qr/candidate/oracle）、`analytic_*`、`oracle_*`、`status/reason` |
| `diagnostic_stages.csv` | 每阶段 status（EXECUTED/SKIPPED/EXCEPTION）+ wall_ms + reason（**全部检查状态的来源**） |
| `diagnostic_candidates.csv` | 每动作 kernel/post/pl 布尔、coverage_rejected、selected、slow_path、near_gate、numerical_path、fallback_reason、skip_reason、cache_hits、certificate 界、matrix_free_step_rejected、各段耗时 |
| `diagnostic_coverage.csv` | 每动作 plausible/mandatory/covered/uncovered 集合与 outcome/reason |
| `diagnostic_state_steps.csv` | BASE/SELECTED 状态增量的分段范数（rotation/position/velocity/bias） |
| `hypotheses.csv` | 每假设 dim/rank/sigma/cond/slopes*/onset/先验/分配/monitorable/plausible/conditioned 统计；**slope 列有已知缺陷（恒 inf，见下）** |
| `integrity.csv` | 检测器统计/阈值/dof/通过、PL、availability、fde_status、selected action、bridge PL、recovery/reinit 字段 |

**真正缺失、本轮以“离线影子”补齐的**：
1. 首因（primary failure）——v10 只有单条 `status/reason`；离线工具按层定序给出 `first_limitation`。
2. 全部检查状态汇总——v10 分散在 stages/candidates/coverage；离线工具合成逐检查台账。
3. 离线 shadow diagnostic（gate 提前返回后继续列举后续缺陷）——离线工具对同快照继续做
   纯代数与已导出检查的继续评估，并把无法继续的部分标注 `NOT_EVALUATED`。
   **未把这三个字段写回生产 schema**：A2 明确“如需改代码最小化”，而 v10 已含底层数据；
   变更 CSV schema/结构应随 A3 的接口冻结一次完成（见 `baseline-report.md` 下一批输入）。

## 4. 已发现但未修复的缺陷（如实记录）

1. `hypotheses.csv.slope_x/y/z` 恒 `inf`：`integrity_monitor.cpp:1793` 读取
   `evidence[i].monitorability.protected_slopes`，批量路径从未给该副本赋值（默认 +inf）。
   仅诊断导出受影响；PL 使用 `pl_entries` 不受影响。修复 + 回归测试建议 A3/A4。
2. `BoundaryPrior` 块的 `window_column_indices` 为空（构造时未填），普查/下游如需按列分析该块，
   只能按形状推断；建议 B 统一块元数据时补齐。
3. `epochs` 语义在配置注释里未写明“区间数”；本次已用实验锁定并写入 `effective-config.yaml`。

## 5. 复现

```
cd <repo>
python3 doc/evidence/integrity-kernel-refactor/tools/analyze_frames.py
# 期望输出：census.json 与 fixtures/frames.json（内容与 hashes.txt 对应版本一致）
```
依赖：python3 + numpy + scipy（本机 1.24.4 / 1.10.1）。
