# baseline-report（工作包 A1/A2，2026-09-21）

> **P2 更新见 §7**：本文件 §1–§6 保留 A1/A2 当时状态（生产源码零修改），
> 其中 G7/G9/G10/G11 等条目已在 P2 部分或全部关闭，状态变更以 §7 为准。

- base/run SHA：`2772b3a1e3584b25c889239421404f550fba9f5d`（本地 HEAD == origin FETCH_HEAD，0/0）
- 范围：仅 A1+A2；**未做任何 B/C/D 算法改动**；生产源码零修改；工作树仅新增本证据目录（未跟踪、未提交）
- 证据：`code-map.md`、`census.json` + `census-summary.md`、`effective-config.yaml`、`runbook.md`、
  `fixtures/`（五类帧 + 影子诊断）、`raw/`（全部原始日志，sha256 见 `hashes.txt`）

## 1. 三栏结论表

### 1.1 已实现且有验证（复用即可，不重造）

| # | 能力 | 证据 | 状态 |
|---|---|---|---|
| 1 | 增量 FGO + 冻结交易（prepare/commit/discard 原子性、slot/ledger 溯源） | `IntegrityV2Transaction.*`（6 用例）+ 全部 198 测通过 | PASS（测试） |
| 2 | 冻结窗口 H/z + SVD/正规方程/LLT 数值契约 + 指纹失效 | `IntegrityV2Numerics.*`、`GateDNumerics.*`；独立复算 rank/ν/cond/`HᵀH` 一致 | PASS（测试+独立复算） |
| 3 | 白化一次、raw/whitened 双表示一致 | 5 帧 23 块全部 `‖W·J_raw−J_w‖/max ≤1.7e-16`（`census.json`） | PASS（独立复算） |
| 4 | 联合窗口检测器（parity² vs χ²，union-bound 风险） | `IntegrityV2Detector.*`；实测 A/C/G 帧统计量与独立计算一致（419.3/166, 12960/97.7） | PASS（测试+独立复算） |
| 5 | 故障模式生成（UWB 瞬时/persistent/ramp、IMU 6 轴区间常值、UWB×IMU 组合） | `IntegrityV2FaultModel.*`；假设构造公式与 5 组实跑计数逐项吻合（236/324/198/236/3588） | PASS（测试+实测） |
| 6 | 假设证据批量求解 + 共享上下文 + 低维定长 LDLT | `GateDProtectionLevel.*`；实测 `covariance_rhs_solves=1/帧`、`hypothesis_shared_hits`；ladder 与生产 Γ 一致 | PASS（测试+实测） |
| 7 | 候选核（KEEP 复用、低秩修正、谱证书、参考回退、0.25 步长 gate、块级缓存） | `IntegrityV2RankUpdate.*`、`GateDNumerics.*`、`GateDCache.*`、`GateDOracle.*`；KEEP 帧 `candidate_reference_svd=0` | PASS（测试+实测） |
| 8 | 完整替换索引集（FDE 排除候选）+ 稠密 oracle 对照 | `GateDOracle.*`、`test_dense_oracle.cpp` | PASS（测试） |
| 9 | PL 数学（Λ 边界、slope、逐轴、风险账本闭合、不可用语义、formal=false） | `IntegrityV2ProtectionLevel.*`、`IntegrityV2Risk.*`；实测 reason 串与 `pl_xyz` 相符 | PASS（测试+实测） |
| 10 | 健康状态机与恢复门槛、桥接确定性盒界（与优化协方差分离） | `IntegrityV2Health.*`、`IntegrityV2Bridge.*`、`IntegrityV2Reinitialization.*` | PASS（测试） |
| 11 | 诊断 v10 全套表（attempt/stage/candidate/coverage/step/evidence/ledger/health/bridge） | 7 个场景的逐帧导出全部生成 | PASS（实测） |
| 12 | 构建与既有测试入口 | `catkin build` exit 0；`catkin run_tests uwb_imu_pl` 198 tests / 0 fail（包级） | PASS |

### 1.2 真正缺口（本稿复核后仍存在）

| # | 缺口 | 现状证据 | 影响工作包 |
|---|---|---|---|
| G1 | 历史无故障/检测摘要：窗口外因子仅压缩为名义 `BoundaryPrior`，无 `F_b/T_b/d_perp`，历史故障方向不可监测 | `incremental_estimator.cpp:1178-1245`；`code-map.md §3.2`；manifest `history_recovery=active_window_only_maturity_delay` | C1/C2 |
| G2 | 三套数值表示并存（SVD+正规方程+LLT），无单一平方根上下文；`HᵀH` 条件数平方路径仍用于全部右端 | `integrity_window_snapshot.cpp`、`rank_update_kernel.cpp` | B1 |
| G3 | 完整 all-mode Gram 仍每帧构造（m×Σq 一次 GEMM），随假设数线性增长 | `hypothesis_evidence.cpp:evaluateContiguous`；实测 236 假设/帧 | B2 |
| G4 | 正常帧仍全量计算假设证据（无惰性 FDE / 证据裁剪）；健康帧动作已收缩为 KEEP_ALL，但证据循环未惰性化 | `integrity_monitor.cpp:1727`（动作）+ `hypothesis_evidence` 阶段计时 | B4 |
| G5 | 分组 envelope / 覆盖图不存在；FDE 只在“单动作覆盖 plausible 全集”时可提交，C/G 场景因此不可用 | 实测 C ep26 `COVERAGE_GAP`、G ep10 `NO_VALID_CANDIDATE` | B2/B3 |
| G6 | 风险：无联合盒并集公式（§5.9）、无遗漏/包络/选择预算账本项（`p_bridge/history/model_escape=0` 为硬编码），风险只对“剩余假设”逐轴 max | `protection_level_v2.cpp`、`risk_budget_audit.cpp`、config `risk` | B3/C3 |
| G7 | 被保护量为 body 原点（`C=[0|R]`），与带 lever arm 的 tag 观测不同一；无有限差分 C 测试 | `incremental_estimator.cpp:1393`；manifest scope | A3/B |
| G8 | 无法监测/可估计性约化缺失：窗口秩亏/`dof<=0` 直接 discard；`ker Z ⊆ ker G` 判据只在测试/文档层面，无 gauge/可估计子空间约化 | `finalizeIntegrityWindow` 的 `model_valid` 条件 | B1/B3 |
| G9 | 诊断缺陷：`hypotheses.csv` slope 列恒 inf（evidence 副本未赋值） | `integrity_monitor.cpp:1793`；实测 236/236 行为 inf | A3/A4（修复） |
| G10 | 无首因/全检查状态/离线继续诊断的**生产字段**；本轮以离线工具补齐（v10 已含底层数据） | `fixtures/README.md §3` | A3 |
| G11 | 无 proof-obligations、fault manifest、config-migration、独立参考 oracle（A3/A4 交付物） | 目录中不存在 | A3/A4 |
| G12 | 性能口径未重测：旧 629ms 属 K=20/ramp/624 假设的 benchmark 口径；本轮开发运行器 ~30–45ms/帧（K=10、236 假设），两者不可混用 | `runbook.md §4`；`raw/runs/*/timing.csv` | D |
| G13 | 真实 dropout/丢包帧、真机包络、非线性包络、Gate J 证据缺失 | 运行器无丢包注入；`formal_eligible=false` 于代码与运行输出 | A4/C/D |

### 1.3 路线图旧稿过时/需修正条目

| # | 旧稿条目 | 修正 | 证据 |
|---|---|---|---|
| O1 | `integrity_window.epochs` 语义未定 | = **区间数**；节点=epochs+1；实测试 epochs10→165 列，epochs20→315 列 | `effective-config.yaml`、`census-summary.md §3` |
| O2 | 624/61,104 假设“最近报告”引用 | 仅对应 K=20+ramp 的 benchmark 口径；当前配置 236（单）/ SD 变体实测 3,588（6 epoch）。公式已从代码导出并双向验证 | `census-summary.md §3` |
| O3 | 629ms/40ms 性能论断 | 不能与本轮 dev-runner 数字混合；性能必须 D 阶段同口径重测 | `runbook.md §4` |
| O4 | “诊断已有 slope 可读”隐含假设 | 实际 CSV slope 恒 inf（缺陷 G9），不能据此判断监测性 | `fixtures/README.md §4` |
| O5 | “template ⊆ recent-k ⊆ free”阶梯 | 与本实现无关（实现只有 window 内模板与（缺）历史摘要）；§1.2 反例 B 依然成立，但本仓库当前不构造 recent-k 概念 | `code-map.md §4` |
| O6 | 若按旧稿寻找 V3/KF 后端等 | 不存在第二套主估计器；不要新建同名模块 | `code-map.md` |

## 2. 本轮执行的命令与结果汇总

| 步骤 | 命令（详见 `runbook.md`） | 结果 |
|---|---|---|
| 证据采集 | git status/HEAD/remote fetch/ls-files/find AGENTS.md | PASS；HEAD==远端 0/0；无 AGENTS.md |
| 构建 | `catkin build uwb_imu_pl` | PASS（exit 0；`raw/build-release.log`） |
| 测试 | `catkin run_tests uwb_imu_pl` + 包级 `catkin_test_results --verbose` | PASS：198 tests / 0 errors / 0 failures / 0 skipped（`raw/run_tests.log`、`raw/catkin_test_results_package.txt`）。注：工作区根裸跑 `catkin_test_results` 会包含另一仓库的 stale failure XML，与本包无关（`raw/catkin_test_results_verbose.txt`，如实保留） |
| 开发运行器最小场景集 | A/C/D/E/F/G/H 共 7 次（30/30/30/30/30/45/226 epoch） | 全部 exit 0，日志 `raw/runs/*/stdout.log`；关键计数见 §1.1/§1.3 |
| census 变体 | SD 计数（6 epoch）、epochs=20 窗（30 epoch + bin） | PASS，`raw/census/sd`、`raw/replay/epochs20-census` |
| 冻结窗口导出 | 6 次导出（5 类帧 + epochs20） | PASS，`raw/replay/*/exports/*.bin` |
| 离线普查/影子诊断 | `python3 tools/analyze_frames.py` | PASS，`census.json`、`fixtures/frames.json` |
| 有效配置 | 复核 FNV 链 `b2863fca7f443d3d` == manifest | PASS |
| 未运行 | Debug 构建、性能 benchmark、ROS 回放/真机 | NOT_RUN（理由见 `runbook.md §4`） |

## 3. 五类帧基线行为（实测，作为后续回归对照）

| 类 | 帧 | 检测 | FDE/候选 | 提交 | PL/可用性 |
|---|---|---|---|---|---|
| 普通 | A ep30 | pass（0 ≤166） | 仅 KEEP_ALL 评估 | 是 | 有限；hpl 2.885>2 → UNAVAILABLE |
| 报警 | C ep25 | alarm（419>166） | KEEP_ALL 不合格；union 排除选中 | 是（SUCCESS_UWB_EXCLUSION） | 有限；hpl 2.794 → UNAVAILABLE |
| inf | C ep26 | pass | 无 eligible 覆盖 plausible 集 | 是（best-effort） | **inf**；AMBIGUOUS_UNAVAILABLE |
| 缺测/拒批 | G ep10 | alarm（1.3e4≫97.7） | 唯一候选被 0.25 gate 拒绝 | 否（discard） | inf；NO_VALID_CANDIDATE |
| 边缘化 | H ep201 | pass | KEEP_ALL | 是 | 有限；hpl 5.244 → UNAVAILABLE |

三阶梯对照（同帧固定 H/C/噪声/风险）：五帧的 one-dim / template / free-profile 变体全部可监测
（rank(Z)=q，σ_min≥7.4，cond≤12），**失败发生在其后的候选 eligibility/覆盖与数值 gate**，
本轮只定位“先失败的一步”，不作原因声明（见 `fixtures/README.md §2`）。

## 4. 交付物清单与状态

| 文件 | 状态 |
|---|---|
| `worktree-before.txt` | 完成（含 date/branch/HEAD/status/remote/submodule） |
| `remote-head.txt` | 完成（fetch 后 FETCH_HEAD==HEAD，left-right 0/0） |
| `tracked-files.txt` | 完成（611 条，生成时刻） |
| `agent-instructions.txt` | 完成（AGENTS.md/CLAUDE/copilot 检索为空，附结论） |
| `code-map.md` | 完成（§3.2 六入口 + 12 个补记模块，每题有结论） |
| `census.json` + `census-summary.md` | 完成（真实导出，绑定 bin sha256） |
| `effective-config.yaml` | 完成（全量有效值 + 优先级链 + hash 复核） |
| `baseline-report.md` | 本文件 |
| `runbook.md` | 完成（BUILD/UNIT/INTEGRATION/REPLAY 命令 + 版本 + NOT_RUN） |
| `fixtures/`（frames.json + README + 五类帧 bin 引用/哈希） | 完成（bin 在 `raw/replay`，不入库原则以 sha256 追溯） |
| `hashes.txt` | 完成（342 个文件） |
| `raw/`（构建/测试/7 场景/census/导出日志） | 完成（47MB，未提交） |

## 5. 诚实状态声明

- 项目状态**保持 `IMPLEMENTED_UNVERIFIED` / `formal_eligible=false` 不变**；本轮未改任何阈值、
  未删故障、未动配置默认值、未把失败改写为 SKIP。
- 未运行项全部标 NOT_RUN 并给出理由；两条“PASS”均附日志路径。
- 新发现缺陷（G9 及 BoundaryPrior 列索引为空）如实列入缺口表，**未在本轮修复**。
- 既有证据文档复核结论：`integrity-v2-gates.md` 自述 Gate A–H 功能链完成、Gate I 仅“active-window
  恢复链”且 checkpoint/边缘化先验恢复明确不在范围、**Gate J 未开始**——与本轮看到的 `formal_eligible=false`
  一致；`integrity-fault-model-selection/summary.json` 的 624/61,104 与旧性能数字已按 §1.3-O2/O3 修正口径，
  不得直接引用为本轮事实。

## 6. 下一批（A3/A4）输入清单

1. **A3（manifest 与接口冻结）**：
   - 以 `census.json` 的真实键/维度定义 `IntegritySnapshot`/结果/失败原因字段（路线图 §4.1–4.5），
     并把 `primary_failure / all_failures / not_evaluated_checks` 一次性加入诊断 schema（v11）
     与 `evidence.monitorability.protected_slopes` 修复一起做（附回归测试）。
   - 冻结“受保护参考点”契约：body 原点 vs tag（含 lever arm 有限差分测试）——需要指挥官确认被保护量声明。
   - 建立 `proof-obligations.md` 与 config migration（`epochs→intervals` 语义已由本轮锁定）。
2. **A4（独立参考与解析测试）**：需要 `raw Jacobians + raw covariance + raw fault mapping + C`
   的稳定导出（当前可用 replay bin：含 H/z/C/blocks/actions 与 raw 矩阵）；解析 fixture 与 §1.2 反例；
   IMU 原始样本端注入（`UWB_IMU_PL_IMU_FD_ORACLE` 路径已存在，可直接扩展）。
3. **数据/环境**：真实 bag 或 simulator 回放脚本、丢包注入能力（“缺测帧”严格版）、
   Debug/独立 catkin 空间（若需 Debug 结果）、D 阶段同口径性能运行（K=20/ramp 对照与当前 K=10 口径分列）。

## 7. P2（A3+A4）更新，2026-09-21

**范围**：A3（manifest、公共契约冻结、配置迁移）+ A4（独立参考与解析/数值对照）。
**未做任何 B/C/D 算法改动**：未实现平方根内核、历史摘要、分组包络、惰性 FDE；
所有阈值（0.25 gate、128 动作上限、`p_fa/p_md`、alert limits、风险预算）与
PL/detector/evidence/fault-model/rank-update 数学均未改动。与 A1/A2 相比，
本轮**修改了生产源码**（修复 + 契约新增），文件清单见最终报告。

### 7.1 缺口表状态变更

| # | P1 缺口 | P2 状态 | 证据 |
|---|---|---|---|
| G7 | 无 C=[0\|R] 有限差分测试 | **关闭**：`ReferenceFixture.ProtectedMapIsBodyOriginFiniteDifference`（中心差分，tol 1e-7，含杠杆臂反例断言）+ `gtsam::Pose3::retract` 探针 | `test/test_integrity_reference.cpp` |
| G9 | `hypotheses.csv` slope 恒 inf | **关闭**：`hypothesis_evidence.cpp` 在 `analyzeDynamic/analyzeFixed` 中同步写入 evidence 侧 slope；P2 重跑 A 场景 5930 行全有限 | `oracle-results.json` O5/O7 |
| G10 | 无首因/全检查状态生产字段 | **部分关闭**：v11 attempts 新增 `primary_failure/all_failures/not_evaluated_checks` + `diagnostic_snapshot_identity.csv`；离线诊断工具不变 | `raw/runs_p2/*/diagnostic_attempts.csv`、`raw/run_tests_p2.log` |
| G11 | 无 proof-obligations / fault manifest / config-migration / 独立 oracle | **关闭**：`proof-obligations.md`、`config/integrity_fault_manifest.yaml`（+`fault-manifest-resolved.yaml`）、`config-migration.md`、`tools/oracle_compare.py`（+`oracle-results.json`）、`tools/run_validation.py`（+`validation-manifest.json`） | 本目录 |
| 其他缺口（G1–G6、G8、G12、G13） | 未变 | 归 B/C/D；`validation-manifest.json` 中对应 ID 状态 `NOT_STARTED` 并标注归属 | `validation-manifest.json` |

### 7.2 本轮数字（与 P1 对照口径一致）

| 项 | P1 | P2 |
|---|---|---|
| 包级测试（`catkin_test_results` 计数口径） | 198 / 0 fail | **240 / 0 fail**（+42 = 2×21 新增 gtest 用例：8 参考 + 12 manifest + 1 IMU sweep） |
| 开发场景重跑 | A/C/D/E/F/G/H | A/C/D/E/F/G/H 全部 exit 0（`raw/runs_p2/*/stdout.log`），另加 epochs20 FULL_AUDIT |
| 冻结窗口导出 | 6 个 bin（v4） | 8 个 v5 bin（A×1、C×2、F×1、G×1、H×2、epochs20×1），`tools/replay_io.py` 全部解析通过，且 C++ `candidate_replay` 可端到端重放 |
| 诊断 schema | v10 | v11（新增列 + identity 表；validators 同步接受 v11） |
| 独立 oracle | 无（P1 为离线 census） | 8 窗口 × O0..O7 = **57 PASS / 0 FAIL / 7 NOT_RUN**（NOT_RUN 均为 FULL_AUDIT 未开/无 pending 行的真实缺数据） |
| 验证调度 | 无 | `run_validation.py --all`：13 PASS / 0 FAIL / 28 NOT_RUN（未开始项均有归属工作包） |

### 7.3 诚实状态声明（P2）

- `formal_eligible` 仍为 `false`，运行 reason 串保持
  `IMPLEMENTED_UNVERIFIED: Gate J calibration and independent review pending`。
- 状态词严格区分 PASS / FAIL / NOT_RUN：oracle 的 7 个 NOT_RUN 与
  validation 的 28 个 NOT_RUN 都给出原因或归属工作包，未把缺数据写成通过。
- “两边都 inf 即通过”被显式禁止：O5 对非有限值判 FAIL_NONFINITE；
  `OneDimensionalUndetectableFault` 要求 Γ=0 → DANGEROUS 分类而非数值巧合。
- v5 replay 布局修复逐条记录：v5 初版漏写 `factor_inventory`（发现后修复并重新导出
  全部 8 个 bin，`raw/replay_p2` 为修复后产物）。
- 本轮修改的生产文件（A3/A4 范围内）：
  `hypothesis_evidence.cpp`（G9）、`incremental_estimator.cpp`（BoundaryPrior 列索引）、
  `imu_fault_subspace.*`（多步长 sweep）、`integrity_monitor.cpp`（failure 记账/identity/oracle 导出）、
  `run_logger.*`（v11）、`candidate_replay.*`（v5）、`integrity_config.*`（迁移）、
  `types.hpp`（字段）、`CMakeLists.txt`（新目标）；阈值与算法语义零改动。

## 8. P3（B1）更新，2026-09-21

**范围**：统一平方根数值上下文（B1）；存储纪律与 prune；未做 B2/B3/B4/C/D。
**测试**：`catkin run_tests uwb_imu_pl` → **254 tests / 0 errors / 0 failures**
（P2 240；+14 = 2×7 新用例），日志 `raw/run_tests_b1.log`、`raw/b1_square_root_tests.log`。

### 8.1 缺口表状态变更

| # | 原缺口 | P3 状态 | 证据 |
|---|---|---|---|
| G2 | 三套数值表示并存、无单一平方根上下文 | **部分关闭**：求解与隐式 Qᵀ 已统一到一个 QR 上下文（求解 429→15 次 / 92,262→1,695 列）；rank/cond 仍以 SVD 为权威、`HᵀH` 仍作为基信息导出 | `square-root-context.md §1/§4` |
| G11 附带 | A 轮缺陷 G9 / BoundaryPrior 已修 | 不变（P2 完成） | `oracle-results.json` |
| 其他（G1、G3–G6、G8、G12、G13） | 未变 | 归 B2/B3/B4/C/D | `validation-manifest.json` |

### 8.2 本轮数字

| 项 | P2 | P3/B1 |
|---|---|---|
| 测试 | 240 / 0 fail | **254 / 0 fail** |
| 信息求解（LLT/谱） | 429 次 / 92,262 列 | **15 次 / 1,695 列**（回退） |
| 上下文求解 | — | 414 次 / 90,567 列；隐式 Qᵀ 406 次 / 89,274 列 |
| 符号缓存 | — | 339 命中 / 82 未命中 |
| 场景重放离散差异 | — | **0**（7 场景 × 19 文件） |
| 独立 oracle | 57 PASS/0 FAIL/7 NOT_RUN（P2 口径） | 36 PASS/0 FAIL/7 NOT_RUN（5 个保留 fixture） + 上下文 oracle 30 PASS |
| 验证调度 | 13 PASS/28 NOT_RUN | 18 PASS / 0 FAIL / 24 NOT_RUN |
| `raw/` 体积 | 98 MB | **7.5 MB** |

### 8.3 诚实声明

* 阈值与算法语义零改动；`formal_eligible` 仍为 `false`。
* 每假设 Z 三态分类为**审计层**，本轮不改变决策（见 proof-obligations 第 13 项）。
* 3 个 fixture（F30 / H205 / epochs20）在 prune 时删除，相关 oracle 项为 NOT_RUN 并给出再生命令。
* 计时只报观测：`hypothesis_evidence` 中位 5.35→5.69 ms；`window_fingerprint` 桶
  1.29→3.02 ms（含每窗口 QR 构建 ~1.7 ms）；不做 40ms 结论。
