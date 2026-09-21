# C1-b/C1-c 执行记录：已交付项（C2/B5/C3 载体）与**阻塞项**（B1 核心）

状态：**C1-b 管线接入阻塞**——B1（历史故障参数化 → 摘要接入 `buildIntegrityWindow`）
不是本轮窗口内可诚实闭合的项；依赖它的 B2–B4、B6 生产者、B7、B8、C1c-C1、C1c-C4
一并**阻塞**（逐条见 §1，含原因与所需输入）。**不依赖 B1 的项已全部完成并锁测试**：
C1c-C2 容量键（配置层）、B5 缓存身份（含缺陷修复）、C3 版本绑定载体（§2）。
生产行为零变化：无调用点、无语义改动；测试 322 → **328/0**。
代码提交 `23f6e18`（run_sha 口径）；基线 `4e5c1ad`。

> 纪律声明：本文件顶部即阻塞清单。**未做**任何"B1-lite"（纯名义摘要替换特征分解）
> 或 default-off 集成开关——前者语义收益为零且改变 20+ 脆弱数值断言，后者把"历史
> 故障首次进入检测"这一本轮核心目的做成空壳，均属半成品，按纪律拒绝（§3）。

## 1. 阻塞项（原因 + 所需输入）

### 1.1 B1 边界构造替换（核心阻塞，B2–B4 随之）

**原因（代码事实，逐条可复查）：**

1. **历史故障列没有现成的构造路径。** 故障映射只在窗口块上构建：
   `HypothesisGenerator::generate` 的 `fill` lambda（`src/uwb_imu_pl/integrity/hypothesis_generator.cpp:675-700`）
   对每个 occurrence 要求 `explicitlyMonitored(window, id)` **且** `blockFor(window, id)`
   命中 `window.blocks`；被 fixed-lag 消元的旧历元因子在清单里是
   `FrozenFactorDisposition::UnrecoverableHistory`
   （`include/uwb_imu_pl/estimation/integrity_window_snapshot.hpp:62-79`），
   没有 block ⇒ 无 fault map。设计 §11.2 要求的"在 `linearize()` 之前把可监测
   故障模式的旧历元列加入线性化"因此需要一个**新的历史故障参数化子系统**：
   对任意历史 epoch 从 `frozen_slots` 的因子对象合成故障列（语义必须与
   HypothesisGenerator 一致：anchor id / onset / ramp / epoch-independent），
   且 `eliminatePartialMultifrontal` 只消状态键、**不消故障列**。
2. **行/列映射口径待原型验证。** 现路径取的是 `reduced->hessian(inside_ordering)`
   正规矩阵（`incremental_estimator.cpp:1221-1224`）再做特征分解；module §7 要求
   从消元后的**行**（Jacobian 口径）构造 `(R_b,T_b,d_b,F_b,d_perp)`。GTSAM
   `eliminatePartialMultifrontal(..., EliminateQR)` 归约因子的行抽取 API 未验证；
   设计 §11.3 的备选（对 (Λ,η) 对称 Cholesky + 取交叉块 + 被消元残差行）也需实证。
   **不做猜测**：这需要一个小型 API 原型（可用现有测试窗构造）先定路径。
3. **影响面必须同轮重基线（20+ 数值敏感断言 + 证据工具）。** 关键样例：
   `test/test_realtime_incremental.cpp:966`（fixed-lag↔unbounded，`1e-12` 级
   covariance/statistic/PL 对比 :1042-1046）；`test/test_integrity_v2.cpp`
   的 rank/dof 恒等式与 `1e-12` 分解计数；**精确** `rank/dof/detector_only_rows`
   的外部 oracle（`doc/evidence/integrity-kernel-refactor/tools/context_oracle.py:13,102-111`
   O8a/O8d）；身份字段 `boundary_summary_id` / `validity_assumptions =
   "history_nominal_boundary_only"`（`src/uwb_imu_pl/integrity/integrity_monitor.cpp:1376-1395`）。

**所需输入（按序，缺一不可）：**

1. 历史故障参数化子系统（新模块 + 对 `frozen_slots` 因子行的原型验证）；
2. 决策冻结：哪些历史模式/锚点/onset 进入摘要及其生命周期（§7.6 行 1–7），
   以及容量默认值的**执行语义**（本轮只交付了加载层，见 §2.1）；
3. 行抽取原型结论（GTSAM JacobianFactor vs §11.3 Cholesky 路径）；
4. 全量重基线计划（上述断言的逐条口径 + v15 + oracle 工具）。

### 1.2 B2（顺序：摘要先于删除）/ B3（组合与上下文）/ B4（池化检测）
全部以"摘要对象存在于管线"为前提，随之阻塞。池化插桩点已勘察定位，供下一轮直接
开工：`joint_window_detector.cpp:84,113-115,147-149`（statistic/dof/threshold）；
`rank_update_kernel.cpp:569,720-731`（candidate statistic）；
`hypothesis_evidence.cpp:505-518,540`（Γ/score 组装与 `faultResponse`）。

### 1.3 B6 冷启动
`FailureReason::HistorySummaryInvalid` 与字符串 `"HISTORY_SUMMARY_INVALID"` **已存在**
（`include/uwb_imu_pl/common/failure_reason.hpp:18`；
`src/uwb_imu_pl/common/failure_reason.cpp:32,88,92`），但**无生产者**；产出路径属 B1
接入的摘要管理器 ⇒ 阻塞。

### 1.4 B7 HIS-01 oracle
需要摘要对象才能定义"同一冻结问题"的逐项比较；现有固定滞后↔无界模式是可用样板
（`test/test_realtime_incremental.cpp:966-1046`）⇒ 阻塞于 B1。

### 1.5 B8 新场景（HIS-02）
场景参数**已具备**（`apps/r0_r1_development.cpp:124-129` `fault_epoch_begin/end`，
`parseScenario` 名称表），但"跨边界后仍可监测"的断言对象属 B1 ⇒ 阻塞；**未添加**
无断言的空场景（避免噪声）。

### 1.6 C1c-C1 生命周期 §7.6 行 1–7
全部作用于摘要生命周期（模式进入历史/源离窗/结束≠清除/淘汰证明/热切换拒绝/
容量动作执行）⇒ 阻塞于 B1。

### 1.7 C1c-C4 诊断 v15
v15 的字段源（Ω/ξ/κ/ν_⊥、池化检测量、历史 Γ 块、血缘列）全部来自 B1/B4 ⇒ 阻塞。
版本耦合点已列明（下一轮机械执行即可）：`src/uwb_imu_pl/io/run_logger.cpp:349,362,709`；
`tools/validate_run_schema.py:388-393`；`tools/gate_d_diagnostics.py:32,69,130,149`；
`test/test_fault_manifest.cpp:316,335`。

### 1.8 D1–D3 场景差异/可监测性变化/等价表
无 B1 则检测/PL 语义未变化，**无可列出的逐帧差异**（不允许编造）；
`equivalence-summary.json` 本轮未改动，原因即此。

## 2. 本轮已交付（每条：文件 + 函数 + 测试名）

### 2.1 C1c-C2 容量键（配置层，DONE）
* `include/uwb_imu_pl/config/integrity_config.hpp`：`HistoryCapacityConfig`
  （`max_summary_rows` / `max_fault_columns` / `max_perp_rows` /
  `capacity_action`，默认 `0/0/0/REFUSE`）+ `IntegrityConfig::history`。
* `src/uwb_imu_pl/config/integrity_config.cpp`：根 allowlist 增加 `history`；
  严格解析（**未知键、缺键、非法 action 硬错误**；节缺省 = 零容量默认；resolved
  dump 规范化，config hash 覆盖）。
* `config/realtime_uwb_imu_pl_research.yaml`：`history:` 节（0/0/0/REFUSE，
  注释说明 pre-C1-b 状态与"绝不静默丢最旧故障"语义）。
* 测试：`IntegrityConfig.HistoryCapacityKeysAreStrictlyLoaded`
  （shipped 值 / 非默认值 / 缺节默认 / 未知键拒绝 / 非法 action 拒绝 / 缺键拒绝）。
* **未含（阻塞）**：超限 → 不可用并计数（执行语义属 B1 的摘要管理器）。

### 2.2 B5 缓存身份（DONE，含缺陷修复）
* `include/uwb_imu_pl/integrity/statistical_bounds_cache.hpp`：
  `StatisticalBoundKey::history_summary_version`；
* `src/uwb_imu_pl/integrity/statistical_bounds_cache.cpp`：`NoncentralKey` 元组
  补齐 **`envelope_kind`（修复：该字段此前只声明在结构体中、未参与哈希 = 静默
  别名轴）** 与 `history_summary_version`；注释写明"无声明而未哈希的轴"。
* 测试：`B3Risk.HistorySummaryVersionIsPartOfCacheIdentity`
  （同键命中 / 版本+1 未命中但值一致 / envelope_kind 两个取值不互相别名且可命中）。
* 生产默认（`version=0`）行为不变；PL 调用点未改（`protection_level_v2.cpp:64`）。

### 2.3 C3 版本绑定载体（DONE）
* `include/uwb_imu_pl/integrity/history_fault_summary.hpp`：`HistorySummaryVersion`
  （`linearization` / `whitening` / `mode_set` / `capacity`）+ 
  `digestHistorySummaryVersion`（与 integrity config 哈希同常数/次序约定的 64bit 折叠）。
* 测试：`HistoryFaultSummary.HISM2VersionDigestBindsAllComponents`
  （确定性；四个分量各自敏感；分量顺序敏感）。
* **未含（阻塞）**：生产侧填充与"变更即重建"触发（属 B1）。

## 3. 明确未做与理由（纪律）

* **未做 B1-lite**（用 q=0 的纯名义摘要把特征分解换成模块路径）：语义收益为零
  （无故障保持），却改写 20+ 脆弱数值断言的底层路径——风险高、价值低，且会制造
  "B1 已接入"的错觉。
* **未做 default-off 集成开关**：本轮核心目的是"历史故障首次进入检测与 PL"，
  默认关闭的集成 = 空壳；默认开启 = 未验证的核心语义改动。二者皆半成品，拒绝。
* **`auto_shrink`**：仓库 grep 0 命中（无设置点）⇒ "保持关闭"无需改动（记为
  vacuous-safe，随 B1 复核）。

## 4. 命令与计数

| 项 | 命令 | 结果 |
| --- | --- | --- |
| 构建 | `catkin build uwb_imu_pl` | PASS |
| 新测试（3 个） | 见 §2 各测试名 `--gtest_filter` | 全部 PASS |
| 全量 | `catkin run_tests uwb_imu_pl` + `catkin_test_results` | **328 tests / 0 errors / 0 failures**（= 322 + 2×3） |

**计数**（摘要行数 / ν_⊥ / κ_b / 分解次数 / 回退 / 容量拒绝）：本轮**不适用**——
摘要未接入管线，无运行期产出（这正是 §1.1 阻塞的直接后果）；不编造数字。

## 5. 影响面清单（供 B1 落地时逐条重基线，均为本轮勘察确认）

`test_realtime_incremental.cpp:966,1042-1046`（fixed-lag↔unbounded 1e-12 级）；
`test_integrity_v2.cpp` 的 rank/dof 恒等式、`1e-12` 分解计数（:1590±）、
candidate `1e-10`、dof==rows-rank（:604,:653）；
`test_square_root_context.cpp:176-201`（detector-only 语义锁）；
`test/test_fault_manifest.cpp:316,335`（诊断版本字面量，随 v15）；
`tools/context_oracle.py:13,102-111`（O8a/O8d **精确** rank/dof）；
`integrity_monitor.cpp:1376-1395`（identity 字段口径）；
`apps/integrity_round2_scenario.cpp:230-233`（boundary 数值消费方）。

## 6. 存储与哈希纪律

* 本轮证据文件：本文件、`runbook.md §13`、`proof-obligations.md §18`、
  `validation-manifest.json`（`HIS-C1`/`HIS-V1` MAPPED；`HIS-01..06` note 更新）、
  `config-migration.md §5`、`code-map.md` 补充（边界路径未替换的显式记录）、
  `hashes-C1C2.txt`（机械规则最后生成：`git diff --name-only 4e5c1ad..23f6e18 ∪
  证据树`，排除自身与瞬态 `__pycache__/`）。
* `validation-report.json` 以代码提交 SHA 生成；runner `--all` 的派生工件
  （`square-root-oracle.json`/`oracle-results.json`）副作用处置同 §12：恢复为提交
  状态（陈旧转储会产出环境性降级，非回归）。
* 不 push；无格式化步骤；大产物不入库。
