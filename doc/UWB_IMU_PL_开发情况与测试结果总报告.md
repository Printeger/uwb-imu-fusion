# UWB–IMU–PL 统一完整性内核：开发情况与测试结果总报告

**对照文档**：`doc/UWB_IMU_PL_Codex_开发路线图.md`（下称"路线图"）
**日期**：2026-09-22 ｜ **分支**：`feature/realtime-uwb-imu-pl`
**代码终版**：`8584396` ｜ **证据终版**：`36f680e`（本报告随收尾提交入库）

---

## 0. 总结论（一页）

1. **路线图 §6 四个工作包全部完成**：勾选项 66/66（A 21/21、B 19/19、C 19/19、D 7/7）；
   A/B/C 三包的合并判据满足，D 包的合并判据除两项"门"外满足。
2. **仅两项门如实未过/未完成**（均为记录在案的结论，不是遗漏工作）：
   * **实时与资源门：不通过（blocker）**——40 ms 目标在成熟期差 2–3 个数量级，
     热点实测为 `window_boundary_provenance`（C1 历史参数化+摘要提取），修复属设计级
     （增量摘要更新 / 边界容量控制），已记录为后续工作；
   * **模型证据门：未完成**——Gate J（独立公式评审 + 真实数据校准）缺数据集未做。
3. **最终状态**：`IMPLEMENTED_UNVERIFIED`；`formal_eligible=false`；**Gate D 不宣告 PASS**；
   所有输出按"显式未保护"发布（`publication_protected=0` 贯穿全轮）。
4. **关键数字**：全量套件 **424 tests / 0 failures**；validation **58 PASS / 0 FAIL / 2 NOT_RUN**
   （@`8584396`）；oracle 重生成 41/0/9 与 35/0/8（含身份绑定陈旧门）；证据树哈希
   **155/155 OK**；证据树 6.2 MB；未缩任何覆盖、未改任何阈值/合同。

---

## 1. 对照路线图的章节映射总表

| 路线图章节 | 要求 | 交付状态 | 证据入口 |
|---|---|---|---|
| §2 工作包与产品语义 | A→{B,C}→D；增量 FGO 单一主上下文；健康帧不构造排除图 | ✅ | `final-review.md` §1–2 |
| §3 与代码对齐（第一组操作） | SHA/工作树/依赖冻结、真实入口、构建与回放命令 | ✅ | `code-map.md`、`runbook.md` §3–6 |
| §4 公共契约冻结 | snapshot/result/action 身份字段；启动与每帧验证分离；结果与失败原因 | ✅ | `types.hpp` 身份字段、manifest v11→v16 台账 |
| §5 数学与风险规格 | λ/Λ/τ 平方约定、平方根主路径、零状态行、可监测性/slope、单一保护模型、单双故障、分组上界、正常帧风险闭合 | ✅ | `risk-and-lazy-fde.md`、`square-root-context.md`、RSK-01..03 |
| §6 工作包 A | 21 项 | ✅ 21/21 | `baseline-report.md`、`census.json`、`effective-config.yaml`、`proof-obligations.md` |
| §6 工作包 B | 19 项 | ✅ 19/19 | `square-root-context.md`、`coverage_envelope` 证据、`risk-and-lazy-fde.md` |
| §6 工作包 C | 19 项 | ✅ 19/19 | `history-summary-*`、`c2-dual-channel.md`、`fde-post-selection.md`、`fde-publication.md` |
| §6 工作包 D | 7 项 | ✅ 7/7 | `d-round-report.md`、`final-review.md` |
| §7 历史摘要与检测细节 | 消元摘要、不丢不重、检测布局、PL 对应、fault-span、生命周期、重线性化 | ✅ | `history-summary-design/module/pipeline.md` |
| §8 模型有效性/FDE/发布 | 线性化与鲁棒权重、双通道模型误差、桥接不伪装高斯、FDE 与 PL 度量分离、post-FDE 保守保证、原子提交 | ✅ | `fde-post-selection.md`、`fde-publication.md` |
| §9 配置定稿与迁移 | 新 schema、一次解析迁移、冲突拒绝 | ✅ | `config-migration.md`、严格 loader 测试 |
| §10 测试矩阵 | CFG/GEO/NUM/MOD/COV/HIS/DET/RSK/FDE/OUT/PRV/GAT 家族 | ✅ 58 PASS / 2 NOT_RUN（文档化限制） | `validation-manifest.json`、`validation-report.json` |
| §11 性能与资源 | 固定口径、40 ms 待验证目标、逐帧 trace、热点实测 | ⚠️ 实测完成，目标**不达标**（blocker 记录） | `d-round-report.md` §D-3 |
| §12 合并验收与证据等级 | 8 门自查 | ⚠️ 6 门通过；实时与资源门不通过；模型证据门未完成 | `final-review.md` §9、`d-round-report.md` §D-4.4 |

---

## 2. 开发流程时间线（轮次 × 提交 × 计数）

| 轮次 | 内容 | 代码提交 | 证据提交 | 套件 | validation (@SHA) |
|---|---|---|---|---|---|
| P1 | A1+A2：工作树冻结、普查、诊断 fixtures、原始基线 | 见 `runbook.md` §3–6 | 同 | 198/0 | — |
| P2 | A3+A4：fault manifest、配置迁移、独立性 oracle | `492dd72` | 同轮证据目录 | 240/0 | 13 PASS / 0 FAIL / 28 NOT_RUN |
| P3 | B1：统一平方根上下文 | `9d5550f` | 同 | 254/0 | 36 oracle PASS + context 30 PASS |
| P4 | B2：单双故障同 registry、compact 模式、包络 | `82408e8` | 同 | 284/0 | — |
| P5 | B3+B4：联合界、风险账本、覆盖证书、惰性 FDE | `e40e2ef` | `d58a45d` | 300/0 | RSK-01..03 PASS |
| P6 | 设计冻结 + Stage 0（envelope 单侧修正、步长门归因） | `e97edee` | `1af17ab` | 302/0 | 28 PASS |
| P7 | dominance 修正落地 + 窄带回归 + 哈希机械规则 | `6484b73` | `c6544d5` | 304/0 | — |
| C1-a | 历史摘要模块（Householder，R_b/T_b/d_b/F_b/d_perp） | `bc5722c` | `4e5c1ad` | 322/0 | 30 PASS / 0 FAIL / 18 NOT_RUN |
| C1-c | 容量配置层 + 缓存身份修复 | `23f6e18` | `e4d1f10` | 328/0 | 32 PASS / 0 FAIL / 18 NOT_RUN |
| C1-b unblock | 历史故障参数化 + 摘要提取（route (i)） | `1f05f11` | `9288f73` | 346/0 | 34 PASS / 0 FAIL / 18 NOT_RUN |
| C1-closure | 接线进 `buildIntegrityWindow` + F1/F2/F3 修复 | `9257941` | `346af7a` | 376/0 | 40 PASS / 0 FAIL / 12 NOT_RUN |
| C1-follow-up | HIP 跨窗场景、carrier oracle、v15 CSV | `8b6c177` | `1d6cf4f` | 378/0 | — |
| M1 = C2 | 双通道检测（T_c+T_b 精确分裂 + §7.4 W 界） | `66b8eb6` | `0121601` | 386/0 | 44 PASS / 0 FAIL / 9 NOT_RUN |
| M2 = C3 | profile 似然、保证组、桥接/IMU 决策序 | `ecbaa8e` | `abb52ec` | 392/0 | — |
| M3 = C4 | 发布身份/原子状态机/看门狗（模块级） | `8c5330f` | `ff0dea2` | 398/0 | 50 PASS / 0 FAIL / 6 NOT_RUN |
| W1 | C3 接线进 `FdeManager::decide` 主链 | `b7feb9c` | `0f37905` | 404/0 | 50 PASS @`b7feb9c` |
| W2 | C4 接线进生产发布链、诊断 v16、replay v6、场景重基线 | `09a71e4` | `3550a38` | 416/0 | 50 PASS @`09a71e4` |
| D | §10 矩阵、§11 性能实测、清理、oracle 重生成、final-review | `8584396` | `36f680e` | **424/0** | **58 PASS / 0 FAIL / 2 NOT_RUN** @`8584396` |

---

## 3. 工作包交付明细

### 3.1 工作包 A（21/21）— 对齐、声明、诊断、独立参考

* `code-map.md` / `census.json` / `effective-config.yaml` / `baseline-report.md` / `proof-obligations.md` 全部交付；
* 缺口表与"过时条目"表诚实记录（含 P1 发现的两个真实缺陷：G9 slope 拷贝、BoundaryPrior 列索引空）；
* 独立参考实现（纯稠密，不复用生产求解器）+ §1.2 反例 + 无害零空间 fixture；
* 解析测试：ν=1、P=1/2、G=1/2、Γ=1/2、s²=1/2 全对；
* IMU 原始样本级注入对照通过。

### 3.2 工作包 B（19/19）— 统一数值、单双故障、风险、惰性

* B1：单 QR/窗口上下文替换全部重复分解，**信息类求解 429 → 15 组、求解列 92262 → 1695**；
  LLT/谱分解降级为**有计数**回退；
* B2：order=1/2 同 registry；compact 模式默认（`all_mode_gram_columns=0`，行压缩 ≈10.1×）；
  覆盖标签 EXACT/UPPER_ENVELOPE/UNCOVERED（DOM-01 相对单侧判据）；
* B3：λ/Λ/τ 约定统一（RSK-01..03）；逐轴联合界；非中心参数稳定求解；零空间不可用判定带
  模式/方向归因；
* B4：惰性 FDE——健康帧 `generated_actions=0`、无候选图；健康管理调用保留；动作目录与
  保证组与 C 对齐。

### 3.3 工作包 C（19/19）— 历史、检测、IMU 处置、发布事务

* **C1 历史摘要**：`R_b,T_b,d_b,F_b/d_perp` 平方根摘要（2 级 Householder）；故障参数化最细粒度列
  （UWB per (anchor,epoch) 常量+时线性伴随列；IMU per 轴区间常量；故障 key 保留在分隔符中）；
  容量 REFUSE（`HISTORY_CAPACITY_EXCEEDED`）；生命周期（出生/跨窗/持续/配置变化/重线性化）；
  摘要先于删除；κ/ν 审计；oracle 复核（κ rel 6.6e-47、rank 390=390 无截断）；
* **C2 双通道检测**：`T_c+T_b=T_pooled`（rel ≤1e-9）、ν_c+ν_b=ν_pooled；§7.4 W 核界（thin SVD，
  无显式逆；证书 `dual_channel_bound_w_<digest>`）；fault-span 已验证但管线保留未投影残差
  （不启用未证明压缩）；ρ 风险调整阈值；
* **C3 post-selection**：profile 似然（原始白化量，可比性身份强制）、保证组（共享参考=同一物理
  失败事件只计 max）、并集选择计费、桥接/IMU 决策序（bounded 包络不当高斯）；
* **C4 发布事务**（模块 + **W2 生产接线**）：`PublicationController` 显式状态持有者；三态身份门
  （admissible / same_time_rebind 无证明即拒 / requires_propagation 显式未保护）；白名单状态机；
  看门狗 wall/sensor 分离（冻结数据在重 FDE 前短路 `WATCHDOG_REFUSED`；"落后但前进"不误拒；
  时钟倒退与平台输入校验一致拒绝）；copy-on-write 提交；
* W1/W2 接线后**发布量零变化**：7 场景 + HIP + **H_mature_union 226 epoch 两侧全量对照
  = 零已发布量差异**（身份门 224 `ADMISSIBLE`/2 `REFUSED`、`finite_pl=224`、
  `numerical_contract_mismatches=0`）。

### 3.4 工作包 D（7/7）— 端到端、性能、清理

* §10 矩阵 order=1/2 分别执行（见 §4.2/§4.3）；
* §11 性能实测（见 §5，含 blocker）；
* 原始逐帧 trace 汇总工具 `tools/perf_trace_summary.py`；
* 清理审计：无可安全删除项（dense oracle 在役兼对照；eager 已惰性化；LLT/谱分解为计数安全网）；
  ADR 0001-0003 追加 D 状态；
* 6 个 NOT_RUN 终局处置（4 补测 PASS，2 文档化 NOT_IMPLEMENTED）；
* `final-review.md` 回答全部评审问题；未通过部署模型证据清单入 `d-round-report.md` §D-4.5。

---

## 4. 测试与验证结果汇总

### 4.1 全量套件

**424 tests / 0 errors / 0 failures**（`raw/run_tests_d_final.log`；口径：gtest 双计数，含 4 个
D 轮新增用例）。演化：198→240→254→284→300→302→304→322→328→346→376→378→386→392→398→404→416→**424**。

### 4.2 validation 矩阵（@`8584396`，`run_validation.py --all`）

**58 PASS / 0 FAIL / 2 NOT_RUN**（60 项；`raw/validation_d_final.log`、`validation-report.json`）：

| 家族 | 条目 | 结果 |
|---|---|---|
| CFG | CFG-01..03 | PASS（CFG-03 本轮补测） |
| GEO | GEO-01..05 | PASS（GEO-03 本轮补测） |
| NUM | NUM-01..04 | PASS |
| MOD | MOD-01..03 | PASS（MOD-01/MOD-03 本轮补测） |
| MOD | MOD-04/MOD-05 | **NOT_RUN = 文档化 NOT_IMPLEMENTED**（启动期拒绝，fail-closed） |
| COV | COV-01..04 | PASS |
| HIS | HIS-01..06 + M1/C1/V1/X1/X2 | PASS |
| DET | DET-01..04 + CH | PASS（DET-01 为 MAPPED_PARTIAL，按原口径） |
| RSK | RSK-01..03 | PASS |
| FDE | FDE-01..05 | PASS |
| OUT | OUT-01..07 | PASS（OUT-04..07 在 D 轮登记） |
| PRV | PRV-01..06 | PASS |
| GAT/DOM | GAT-01、DOM-01 | PASS |

### 4.3 order=2（双故障族）端到端批次（7 场景）

配置 `configs/research-order2-dev.yaml`（与产品配置逐行等价，仅 `double_faults_enabled: true`、
`max_fault_order: 2`）：

* 全部 7 场景 `effective_fault_cardinality=2`；双故障假设峰值 5280–7920/帧（`max_hypotheses`
  汇总 10796 vs order=1 的 236/帧）——真实展开，非声明；
* `mode_dense_allocations=0`、`all_mode_gram_columns=0`（compact 路径在 order=2 仍成立）；
* 发布门对照：order=1 与 order=2 的 `ADMISSIBLE/REFUSED` 仅 HIP_60 早期多 12 次 REFUSED
  （身份门保守侧扩大，非覆盖缩水；两档 `publication_protected=0`）。

### 4.4 H_mature_union 全量对照（W2 缺口收口）

两侧 226-epoch 运行完成；`publish_diff.py` = **exit 0，零已发布量差异**
（共有列 + 覆盖/决策计数逐行相同；两侧汇总计数器逐项相同）。日志 `raw/h_publish_diff_d.log`。

### 4.5 oracle 产物（同代材料重生成 + 陈旧门）

* `square-root-oracle.json`：**41 PASS / 0 FAIL / 9 NOT_RUN**；
* `oracle-results.json`：**35 PASS / 0 FAIL / 8 NOT_RUN**；
* 新增**身份绑定陈旧门**：bin 的 `identity_digest` ≠ 生产运行对应行 ⇒ 判 `stale bin` 转
  NOT_RUN（附精确重导出配方），**未以陈旧材料充数**；
* 修复一个真实缺陷（W2 引入）：replay v6 写入器漏写 `factor_inventory` 段，
  D 轮修复并强化为逐字段往返断言。

### 4.6 证据与可复现性

* `hashes-C1C2.txt` 机械生成（base `3550a38` → code `8584396`），**155/155 OK**；证据树 6.2 MB；
* 大产物按纪律写 `/tmp`，本轮收尾已清理（见 §7.4）；
* 全部命令可复现：`d-round-report.md` §D-6 与 `runbook.md` §21/§22。

---

## 5. 性能实测（诚实版，含 blocker）

### 5.1 dev-runner 场景（core_total，ms）

| 场景 (n) | order=1 p50/p95/p99/max | miss>40ms | order=2 p50/p95/p99/max | miss>40ms |
|---|---|---|---|---|
| A_nominal (30) | 39.7 / 140.5 / 152.7 / 152.7 | 50% | 244 / 475 / 497 / 497 | 87% |
| C_uwb_fde (30) | 40.8 / 132.1 / 158.8 / 158.8 | 53% | 224 / 391 / 442 / 442 | 83% |
| D_imu_bridge (30) | 41.9 / 136.7 / 141.9 / 141.9 | 53% | 221 / 439 / 737 / 737 | 83% |
| E_union (30) | 40.9 / 117.6 / 126.5 / 126.5 | 53% | 250 / 406 / 406 / 406 | 87% |
| F_ramp_unmonitorable (30) | 44.3 / 150.2 / 163.7 / 163.7 | 57% | 342 / 568 / 576 / 576 | 90% |
| G_continuous_rejection (45) | 9.4 / 13.3 / 21.0 / 21.0 | **0%** | 37.1 / 92.1 / 117.4 / 117.4 | 24% |
| HIP_60 (60) | 62.4 / 78.3 / 91.5 / 91.5 | 87% | 55.9 / 67.7 / 71.0 / 71.0 | 93% |

### 5.2 长轨迹基准（figure-eight）

| 变体 | 启动段 p50/p99 (ms) | 成熟段 p50/p99 (ms) | 峰值内存 |
|---|---|---|---|
| nominal (320，至 e199) | 752.9 / 10129 | **50618 / 131536** | 493 MB（运行中） |
| alarm（完成） | 1304 / 3030 | 2536 / 6047 | 122 MB |
| order=2（完成） | 1473 / 18768 | 28803 / 52331 | 297 MB |

* `core_total` 在 **epoch 13** 即越过 40 ms；
* 候选评估路径**不是瓶颈**（逐候选 wall p99 = 28 ms）；`pending_duration_s` p50=max=0.05 s
  （无排队积压）；
* **40 ms 目标：不达标（blocker）**。热点 = `window_boundary_provenance`（占成熟帧 99.5%）：
  C1 历史参数化 + 摘要提取；`boundary_rows` 每 epoch +≈23、时间幂律 ≈r⁴；
* **新发现对照**：同轨迹下"持续拒绝"（alarm）单帧 ~2.5 s，"持续提交"（nominal）增长至 >50 s
  ⇒ 增长由**成功提交的边界内容**驱动——**以拒绝换性能会缩小可用输出，明确不作为优化手段**。

### 5.3 修复路径（已记录，未实施）

属设计级，需覆盖/风险语义决策（可能成为"包 E"）：

1. 增量历史摘要更新（避免每帧全量重算边界 Householder）；
2. 边界容量控制（执行 `history.max_summary_rows`/`max_perp_rows`，见 §7.2）。

---

## 6. 已知限制与未完成证据

| # | 项 | 状态 | 位置 |
|---|---|---|---|
| 1 | **Gate J**（独立公式评审 + 真实数据校准） | 未完成（无数据集） | `d-round-report.md` §D-4.5 |
| 2 | **性能修复**（增量摘要 / 边界容量） | blocker，未实施 | §5.3 本报告 |
| 3 | 容量键缺口：`max_summary_rows`/`max_perp_rows` 未被执行（仅 `max_fault_columns`） | 记录未修（语义零改动原则） | `d-round-report.md` §D-3.5 |
| 4 | MOD-04/05（多轴共因/双 UWB 族、bounded-set 支撑界） | NOT_IMPLEMENTED（启动拒绝） | manifest note |
| 5 | `formal_eligible=false` / `IMPLEMENTED_UNVERIFIED` | 保持（设计） | 全轮一致 |
| 6 | 离线驱动 wall 口径 | 已在五个离线应用显式声明；生产默认不变 | `d-round-report.md` §D-3.6 |

---

## 7. 收尾动作记录（2026-09-22）

1. **.gitignore**：已复核全部大目录（`/results/` 1.2 GB、`/logs/`、`/Testing/`、
   `raw/` 证据暂存）均在忽略范围内；新增大/暂态产物防护（`*.bag`、`*.orig`、`*.rej`、
   `*.bak`、`*.tmp`、`/scratch/`）。
2. **push**：本地领先 origin **31 个提交**（behind=0，快进）；未推送提交中最大 blob 154 KB
   （纯源码）——无大文件上传。
3. **/tmp 清理**：`uwb_imu_pl_*` 运行目录、H 对照产物、oracle 材料、补丁脚本全部清除，
   **1167 MB → 11 MB**；`raw/replay_p2` 符号链接（指向 /tmp）已移除；重导出配方保留在
   `d-round-report.md` §D-6。
4. **证据归档**：已合规（6.2 MB ≤ 15 MB；`hashes-C1C2.txt` 155/155；raw/ 为本地哈希固定
   保留层，不入库）。

---

## 8. 复现命令索引

* 套件/验收/order=2 批次/基准/ trace 汇总：`d-round-report.md` §D-6（第一段）；
* 冻结 bin 重导出 + oracle 重生成：`d-round-report.md` §D-6（第二段）；
* H_mature_union 两侧对照与 oracle 重生成：`runbook.md` §22 D 节；
* 运行 schema 校验：`tools/validate_run_schema.py <run_dir>`；
* 发布差异逐行比较：`tools/publish_diff.py <base_dir> <new_dir>`。

---

## 9. 后续工作建议（按优先级）

1. **性能修复包（唯一实质开发任务）**：先冻结两项语义决策——(a) 边界容量 REFUSE 的阈值与
   对"不可用占比"的可接受线；(b) 增量摘要更新的证明义务（与身份绑定/证书一致性对齐）；
   然后实施增量摘要 + 容量执行，目标把成熟帧从"秒级"拉回 40 ms 量级（先解决 r⁴ 增长项）。
2. **Gate J**：拿到真实数据集后做独立公式评审 + 校准；同期把 `formal_eligible` 的转正路径
   文档化。
3. **学术/对外产出**：候选创新点查新（滑窗估计器的保故障灵敏度历史摘要；FDE post-selection
   计费；实时发布语义与证据体系）。

---

## 附录 A：提交链（本轮开发）

`492dd72`(P2) → `9d5550f`(B1) → `82408e8`(B2) → `e40e2ef`+`d58a45d`(B3+B4) → `e97edee`+`1af17ab`(P6)
→ `6484b73`+`c6544d5`(P7) → `bc5722c`+`4e5c1ad`(C1-a) → `23f6e18`+`e4d1f10`(C1-c) →
`1f05f11`+`9288f73`(unblock) → `9257941`+`346af7a`(C1-closure) → `8b6c177`+`1d6cf4f`(follow-up)
→ `66b8eb6`+`0121601`(C2/M1) → `ecbaa8e`+`abb52ec`(C3/M2) → `8c5330f`+`ff0dea2`(C4/M3)
→ `b7feb9c`+`0f37905`(W1) → `09a71e4`+`3550a38`(W2) → `8584396`+`36f680e`(D)。

## 附录 B：外部格式化事件处置摘要（#1–#8）

全部经 token 多重集机械判定为**纯格式化**（#7/#8 各含 1 个 `case '\\'` / 邻接字面量换行
artifact，零语义），由指挥方或开发方 `git restore` 回退并复核哈希；runbook §7–§20 有逐次记录；
仓库与编辑器侧无任何启用的自动格式化路径（§20 S0-3 审计）。
