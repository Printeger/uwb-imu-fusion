# final-review：UWB–IMU–PL 统一完整性内核（A/B/C/D 收官评审）2026-09-22

范围：路线图 §6 四工作包的最终状态、§10 矩阵、§11 性能、§12 合并门。数值与
命令的完整清单在 `d-round-report.md`（D 轮）与既往各轮证据文档中；本文件回答
评审问题并列出未完成证据。

## 1. 实际改了哪些模块（按包汇总）

* **A（对齐/声明/诊断/参考）**：`config/integrity_fault_manifest.yaml` + 严格装载
  （未知键/族/越界事件拒绝、迁移警告与冲突硬错）；`census.json` / `code-map.md` /
  `effective-config.yaml` / `fault-manifest-resolved.yaml`；独立参考 fixture
  （`test_integrity_reference`，纯稠密实现，无生产求解器）；诊断台账（v11→v16 的
  逐版加列）；验证 runner + manifest（`tools/integrity/run_validation.py`、
  `validation-manifest.json`）。
* **B（数值/单双故障/风险/惰性）**：frozen square-root context（一次 QR/窗口，
  `square_root_context.{hpp,cpp}`）；单双故障同一 registry 与 compact 模式缓冲
  （`hypothesis_generator`，order=2 时 5280–7920 双故障假设/帧、`mode_dense_allocations=0`）；
  覆盖计划与 envelope（`coverage_envelope.{hpp,cpp}`，EXACT/UPPER_ENVELOPE/UNCOVERED）；
  风险账本与联合分位数（`risk_ledger_*` 列、RSK-01..03）；惰性 FDE 与健康状态机
  （`B4LazyFde` 计数：`action_entities_constructed` vs `deferred`）。
* **C（历史/检测/IMU/FDE/发布）**：历史故障参数化（
  `history_fault_parameterization.{hpp,cpp}`，D-1 粒度 constant+time-linear+IMU 轴，
  容量 REFUSE）；摘要提取（`history_summary_extraction`：边界行 Householder、
  `R_b/T_b/d_b/F_b/d_perp`、κ/ν 审计）与生命周期（`HistorySummaryLifecycle`）；
  双通道检测（`dual_channel_detector`，T_c+T_b 精确分解 + W 核测试 + 风险调整阈值）；
  IMU 条件处置与 post-selection（`fde_post_selection`：profile 似然、保证组、拒绝
  未证明恢复）；发布事务（`publication_identity.{hpp,cpp}` + W2 生产接线：显式
  `PublicationController`、三态身份、看门狗冻结规则、原子链，诊断 v16 导出）；
  replay 编解码 v4→v6（先加版本标记再扩展，未知版本拒绝）。
* **D（端到端/性能/清理）**：五离线应用的 wall 政策显式化；四个缺口补测用例
  （CFG-03/GEO-03/MOD-01/MOD-03）；`tools/perf_trace_summary.py`；order=2 验收
  输入配置；本报告 + `d-round-report.md`；ADR 状态更新；性能 blocker 与容量键
  缺口记录。

## 2. 复用了哪些已有能力

GTSAM 因子图/预积分（CombinedImuFactor）、既有 dev-runner 与 benchmark 轨迹
生成器、既有 CSV 台账与 manifest 体系、C1-b 参数化与摘要模块（W1/W2 只做接线与
导出）、既有 oracle 工具（`context_oracle.py`/`oracle_compare.py`）、既有验证
runner/manifest（A 包交付物）——D 轮未新建第二个估计器、第二套风险公式或第二套
测试入口。

## 3. 默认（order=1）与双故障（order=2）分别覆盖什么

* **默认**：单 UWB 锚偏差（瞬时/持续/ramp 三形态，含跨窗持续）与 6 个 IMU 单轴
  区间常量事件；保护量 = body-origin 世界位置；`publication_protected=0`（平台
  认证未完成 ⇒ 显式未保护），`formal_eligible=false`。
* **order=2（`double_faults_enabled` + `max_fault_order: 2`，声明族 uwb_imu）**：
  增加 uwb×accel、uwb×gyro 对（9×Na 每类），实测 7 场景全部
  `effective_fault_cardinality=2`；不支持的族（two_uwb、imu_imu、
  same_device_multiaxis）在启动期拒绝（fail-closed）。
* 覆盖证据：COV-01..04 + 场景 `coverage_*` 列；交叉项完整性由 GEO-05 与 order=2
  字典的真实展开（`fault_cross_blocks` 全量计数）共同给出。

## 4. 仍可能出现的"正常不可用"原因（设计内，非缺陷）

1. 覆盖缺口 ⇒ `inf` PL / `AMBIGUOUS_UNAVAILABLE`（coverage-gap 路径）；
2. 检测零空间危险（`DANGEROUS_FAULT_NULLSPACE`）⇒ 无有限界；
3. 全零候选（`NO_VALID_CANDIDATE`，如 benchmark 合成流与 G 场景的持续拒绝段）；
4. 历史摘要无效/容量拒绝（`HISTORY_SUMMARY_INVALID` / `HISTORY_CAPACITY_EXCEEDED`）；
5. 发布身份不完整（早退尝试缺 `risk_proof_id;protection_level_m`）⇒ 显式 REFUSED +
   缺失清单；平台认证未完成 ⇒ 显式未保护（本状态贯穿全轮）；
6. 看门狗拒绝（冻结数据/墙钟超时）⇒ UNAVAILABLE（重 FDE 之前短路）；
7. `formal_eligible=false`：所有输出保持研究态。

## 5. 实际性能结论（含 40 ms 状态）

* 口径：原始逐帧 trace（`timing.csv`），benchmark 三变体 + 14 条场景轨迹；
  环境负载在报告中声明（并行 2 个 H 运行与 1–3 个基准变体）。
* **40 ms 目标：不达标（blocker）**。`core_total` 在基准轨迹 **epoch 13** 越界；
  成熟段 nominal p50 ≈ 14.5 s（e101–124），至 e199 已达 p50 50.6 s / p99
  131.5 s，H 侧单帧至 128 s（e188）。短轨迹（30–60 epochs）仅在 G（持续拒绝、
  无候选执行）满足 p99 < 40 ms；order=2 使 p50 再升 5–6×（完成全程 150 帧：
  成熟段 p50 28.8 s / p99 52.3 s，峰值 297 MB；alarm 完成：全程 9:22.5、
  122 MB）。
* 热点（实测，非预设）：`window_boundary_provenance` = C1 历史参数化 + 摘要
  提取，成熟帧占比 99.5%；规模量 `boundary_rows` 每 epoch +≈23（fixed-lag 边缘化
  未启动前边界持续累积），时间幂律 ≈ r^4。热点不属于 symbol/index/hash、PIM
  映射、概率反演、临时分配类别，故未实施投机优化；修复属设计级（增量摘要更新/
  边界容量控制），需覆盖与风险账本的语义决策。
* **性能 PASS 不写作完整性认证 PASS**：`formal_eligible=false` 保持；未缩任何
  覆盖、未改任何阈值/合同。

## 6. 未完成证据（如实）

1. **Gate J**：独立公式评审与真实数据校准未做（无数据集）——部署模型证据清单见
   `d-round-report.md` §D-4.5。
2. **性能修复**：增量历史摘要/边界容量控制未实现（blocker，§5）。
3. **容量键执行缺口**：`history.max_summary_rows` / `max_perp_rows` 未被执行
   （§D-3.5，记录未修）。
4. **MOD-04/MOD-05**：NOT_IMPLEMENTED（多轴共因/双 UWB 族、bounded-set 支撑界），
   启动期拒绝，见 manifest note。
5. **H_mature_union 两侧对照**：本轮完成情况与差异结果见 §9 状态行。

## 7. 逐条复现命令

见 `d-round-report.md` §D-6（套件/验收、order=2 批次、三基准变体、trace 汇总、
schema 校验）与 `runbook.md` §22（H 对照与 oracle 重生成命令）。所有命令均可
从仓库内文件复现；大产物写 `/tmp`，入库证据 ≤15 MB。

## 8. 被推翻或更正的旧结论 / 历史事件处置

| 结论/事件 | 最终处置 |
|---|---|
| P6：dominance 双侧绝对判据（相对性缺陷） | P7 已修（相对单侧 `env ≥ leaf·(1−tol)`）+ 窄带回归；本轮 DOM-01 复核 PASS |
| P6 记录：runbook/baseline 声称的一侧性修复不存在 | 已由 P7 修复并保留记录 |
| 格式化事件 #1–#8 及 #8 的第二波（P3/P4/…/W2/D；第二波 = 14:22 覆盖 D 提交文件集） | 全部为非开发方外部工具链；每次以 token 多重集判定、`git restore` 回退、哈希复核；#8 = 8/9 严格等价 + 1 个 `case '\\'` 展开 artifact，#8 第二波 = 8/9 严格等价 + 1 个字符串换行 artifact（均零语义）。防复发规则见 runbook §20 S0-3；本 agent 收尾不做格式化 |
| W2：H_mature_union 单侧成本 "25–40 min" | **更正**：超线性增长被低估；实测 e188/e163 单帧 128 s/75 s，全程每侧 ≈ 2–3 h（§22 D-4） |
| W2：其余离线驱动沿用默认 wall 政策 | D 轮显式声明 `offlineReplayPublicationLimits()`（五个应用），生产默认不变 |
| "40 ms 可达"（待验证目标的常规预期） | 实测 blocker（epoch 13 越界），修复路径记录而**不降目标、不缩覆盖** |
| 容量三键「已落地」 | 部分：仅 `max_fault_columns` 执行；其余两键记录缺口（§D-3.5） |
| W2「replay v6 = v5 布局 + 3 字段」 | **更正+修复**：v6 写入器实际漏写 `factor_inventory` 段（按文档布局的 v6 读取器全部错位）；D 轮修复并把往返测试强化为逐字段断言（`8584396`）；D 代 bin 按修复后写入器重导 |
| `basic_string::_M_create` 校验失败（W2 期） | 陈旧测试二进制 ABI 漂移；重跑套件后消失（`raw/run_tests_cwire_w2_stage3.log`）；D 轮无复现 |

## 9. 路线图 §6 合并判据最终状态表（A/B/C/D）

| 包 | 判据要点 | 状态 |
|---|---|---|
| A | census/映射完整；原有测试有记录；解析/独立参考通过；接口定义 | ✅ 达成（CFG/GEO 全 PASS；参考 fixtures 与 oracle JSON 在库） |
| B | 支持模式与独立参考一致；单双故障同过；无正常帧 eager；热区计数；无覆盖缩水 | ✅ 达成（NUM/COV/RSK/FDE-01..02 全 PASS；双故障 7 场景实测；`action_entities_deferred` 计数在库） |
| C | 历史摘要等价；检测布局与 PL 一致；跨边界不丢；非法重线性化失效；异步/超时测试 | ✅ 达成（HIS/C1/C2/C3/C4 全 PASS；FDE-03..05、OUT-01..07 在库；H 全量对照见 §9 状态行） |
| D | §12 全部必要门；命令与产物可复现；超限不缩声明；无未解释漂移 | ⚠️ 除"实时与资源门"与"模型证据门"外全部满足：实时与资源门 = 不通过（blocker，§5）；模型证据门 = 未完成（Gate J）。命令/产物可复现；无覆盖与语义漂移 |

**D 轮状态行（H 对照）**：H_mature_union 两侧 226-epoch 运行均完成；`publish_diff.py`
全量对照 = **exit 0，零已发布量差异**（共有列 + 覆盖/决策计数列逐行相同；身份门
224 `ADMISSIBLE` / 2 `REFUSED`、`protected=0`、`refused=0`；两侧汇总计数器逐项相同，
含 `finite_pl=224`、`numerical_contract_mismatches=0`）；oracle JSON 已同代重生成
（41/0/9 与 35/0/8，含身份绑定陈旧门）。差异表见 `fde-publication.md` §7.4 的 D 轮
更新，命令与日志 `raw/h_publish_diff_d.log` 见 `runbook.md` §22 D-4（同一提交内）。

**总判定**：实现满足已声明模型中的规格（软件与数学门），但**不能**标记为真实
部署的受保护输出：性能门未过、模型证据门未完成；状态维持
`IMPLEMENTED_UNVERIFIED`，`formal_eligible=false`，Gate D 不宣告 PASS。
