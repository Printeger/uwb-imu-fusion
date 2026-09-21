# runbook（工作包 A1/A2 实测命令与环境）

所有命令在工作区根 `/home/mint/ws_fusion_uwb` 执行，包目录 `src/uwb-imu-fusion-pl`。
原始日志保存在 `doc/evidence/integrity-kernel-refactor/raw/`（sha256 见 `hashes.txt`）。

## 0. 环境与版本（本轮实测）

| 项 | 值 | 来源 |
|---|---|---|
| OS / ROS | Ubuntu 20.04 + ROS Noetic（`/opt/ros/noetic`） | 本机 |
| 编译器 | g++ 9.4.0 | `g++ --version` |
| CMake | 3.16.3 | `cmake --version` |
| GTSAM | 4.2a5（运行器 manifest 记录；本机 `/usr/local/include/gtsam/config.h` major=4） | `run_manifest.json` |
| Eigen | 3.3.7 | `run_manifest.json` / `/usr/include/eigen3` |
| Boost / yaml-cpp | 1.71 / 0.6.2 | dpkg |
| CPU / 核数 | Intel Core Ultra 5 125H 级，18 核（本机 nproc=18） | `run_manifest.json` |
| seed | 20260901（开发运行器固定） | `r0_r1_development.cpp` |
| 并行 | kernel/证据 worker=4（`UWB_IMU_PL_CANDIDATE_WORKERS` 覆盖，[1,4]）；数值线程=1 | `integrity_monitor.cpp` |
| 构建类型 | Release（CMakeLists 默认；`catkin build` 未传调试参数） | `run_manifest.json` build_type=Release |

## 1. 命令清单

```
# 变量
B=/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl
E=$B/doc/evidence/integrity-kernel-refactor
CFG=$B/config/realtime_uwb_imu_pl_research.yaml
SCN=$B/config/r0_r1_development_scenarios.yaml

BUILD_CMD:
  source /opt/ros/noetic/setup.bash
  cd /home/mint/ws_fusion_uwb
  catkin build uwb_imu_pl                       # 本轮 exit=0，日志 raw/build-release.log

UNIT_TEST_CMD:
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin run_tests uwb_imu_pl                   # exit=0，198 tests/0 fail，raw/run_tests.log
  catkin_test_results build/uwb_imu_pl/test_results/uwb_imu_pl --verbose
                                                # raw/catkin_test_results_package.txt

# 注意：在 catkin 工作区根直接运行 `catkin_test_results`（无参数）会把
# 另一个仓库的历史 XML（src/uwb-imu-fusion-ie/... 一个 stale failure）一起汇总，
# 与本包无关。本包结论以包级结果为准。

INTEGRATION_TEST_CMD:  # 本包没有独立 ROS 集成测试目标；等价入口为下面 REPLAY_CMD
  source devel/setup.bash
  rostest uwb_imu_pl realtime_integrity_viz.test   # 已含在 catkin run_tests 中

REPLAY_CMD:            # 开发运行器（确定性流量 + 场景）
  devel/lib/uwb_imu_pl/r0_r1_development $CFG $E/raw/runs/<SCENARIO> <EPOCHS> <SCENARIO> $SCN

# 本轮实跑（全部 exit=0；stdout 在对应目录 stdout.log）
#   A_nominal              30   FULL_AUDIT=1
#   C_uwb_fde              30   FULL_AUDIT=1
#   D_imu_bridge           30   FULL_AUDIT=1
#   E_union                30   FULL_AUDIT=1
#   F_ramp_unmonitorable   30   FULL_AUDIT=1
#   G_continuous_rejection 45   FULL_AUDIT=1
#   H_mature_union        226   （未开 FULL_AUDIT，用于边缘化/成熟帧）

# 冻结窗口导出（影子诊断输入，二进制 bin 不入库原则：见 hashes.txt）
  UWB_IMU_PL_REPLAY_EXPORT_DIR=$E/raw/replay/<NAME>/exports \
  UWB_IMU_PL_REPLAY_ATTEMPTS=<frame list> \
  devel/lib/uwb_imu_pl/r0_r1_development $CFG $E/raw/replay/<NAME> <EPOCHS> <SCENARIO> $SCN

# census 专用（只写证据目录，不改仓库默认）
  UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \
      $E/configs/research-double-faults-census.yaml $E/raw/census/sd 6 A_nominal $SCN
  UWB_IMU_PL_REPLAY_EXPORT_DIR=$E/raw/replay/epochs20-census/exports \
  UWB_IMU_PL_REPLAY_ATTEMPTS=30 devel/lib/uwb_imu_pl/r0_r1_development \
      $E/configs/research-epochs20-census.yaml $E/raw/replay/epochs20-census 30 A_nominal $SCN

# 离线普查/影子诊断（不触碰生产代码）
  python3 $E/tools/analyze_frames.py     # 生成 census.json / fixtures/frames.json

# 事件重放的独立核对（仓库自带运行时，可选）：
  devel/lib/uwb_imu_pl/candidate_replay <bin> <out.csv> 4 1 oracle
```

## 2. 运行器环境变量（只影响本次运行，不影响仓库）

| 变量 | 作用 |
|---|---|
| `UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT` | 打开 factor_ledger/hypothesis/health 全审计导出 |
| `UWB_IMU_PL_REPLAY_EXPORT_DIR` / `UWB_IMU_PL_REPLAY_ATTEMPTS` | 导出冻结窗口 replay bin（attempt id = 历元号，1 基） |
| `UWB_IMU_PL_DEV_*`（UWB_BIAS_M / ACCEL_X_MPS2 / GYRO_RADPS / ACCEL_AXIS / UWB_ANCHOR / FAULT_BEGIN / FAULT_END） | 覆盖场景故障幅值/位置（会写入 resolved_config 的 development 块） |
| `UWB_IMU_PL_CANDIDATE_WORKERS` | 候选/证据线程数，[1,4] |
| `UWB_IMU_PL_IMU_FD_ORACLE` | 用有限差分重积分校验 IMU 解析灵敏度（慢路径） |
| `UWB_IMU_PL_EXHAUSTIVE_CANDIDATES` | 绕过候选 eligibility 跳过（诊断用） |
| `UWB_IMU_PL_DISABLE_HYPOTHESIS_SHARED` / `_BATCH` / `_WINDOW_BDCSVD` / `_NUMERICAL_CERTIFICATE` / `_EARLY_STEP` / `_BLOCK_CACHE` | 关闭对应优化路径（对照用，本轮未使用） |

## 3. 复现性说明

- 开发流为确定性（固定 seed、均值测量流、单线程数值）。同参数重跑产生相同 bin：
  `raw/replay/A_nominal/exports/attempt-30.bin` 与 `raw/replay/F_ramp_unmonitorable/exports/attempt-30.bin`
  hash 相同（`df7e5ab8…`），因为正常帧的窗口/动作与 ramp 开关无关。
- 诊断 CSV 为 `uwb-imu-pl/gate-d-diagnostics/v10`；`run_manifest.json` 绑定 HEAD
  `2772b3a1e3584b25c889239421404f550fba9f5d`（构建时工作树含未跟踪路线图文件 → `git_dirty=true`）。
- 所有 raw 产物 sha256 = `hashes.txt`；`effective-config.yaml` 含配置 hash 复核链。

## 4. 本轮未运行（NOT_RUN，附理由）

| 项 | 状态 | 理由 |
|---|---|---|
| Debug 构建/测试 | NOT_RUN | 需在共享 catkin build 空间整体重建（或新 worktree），会改动用户构建产物；未获授权。建议 D 阶段用独立 catkin workspace 冷启动 |
| `realtime_performance_benchmark`（629ms 口径） | NOT_RUN | 属性能口径（K=20/ramp 开、624/61104 假设）；A2 只测最小场景集。性能重测列入 D |
| 真实 bag / 模拟器 ROS 回放 | NOT_RUN | 数据不在仓库；`roslaunch` 入口只做存在性核对（`launch/realtime.launch`、`launch/realtime_integrity_sim.launch` 存在） |
| 真机先验/包络/校准证据 | NOT_RUN | A4/C/D 输入，本轮无数据 |

_注：本文件中的“PASS”仅指命令按预期完成并产生上述日志；任何数值结论见 `baseline-report.md` 的状态列。_

## 5. P2（A3+A4）更新命令

```
# 构建 + 全量测试（P2 最终口径）
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin build uwb_imu_pl                      # raw/build_p2.log（增量，exit 0）
  catkin run_tests uwb_imu_pl                  # raw/run_tests_p2.log（exit 0）
  catkin_test_results build/uwb_imu_pl/test_results/uwb_imu_pl --verbose
                                               # raw/catkin_test_results_p2.txt
                                               # -> 240 tests / 0 errors / 0 failures
# 计数口径：catkin 对 gtest 用例计两次（suite + case），P1=198 与 P2=240 同口径；
# 实际 gtest 用例 119 + 2 个 ros 条目；+42 = 2x(8 参考 + 12 manifest + 1 IMU sweep)。

# 七个场景重跑（v11 诊断 + G9 修复后的 slope 列）
  devel/lib/uwb_imu_pl/r0_r1_development $CFG $E/raw/runs_p2/<SCENARIO> <EPOCHS> <SCENARIO> $SCN
  # A_nominal 30 / C_uwb_fde 30 / D_imu_bridge 30 / E_union 30 /
  # F_ramp_unmonitorable 30 / G_continuous_rejection 45（以上 FULL_AUDIT=1）
  # H_mature_union 226（不开 FULL_AUDIT，与 P1 口径一致）

# v5 冻结窗口导出（与 P1 相同的 attempt 集合；bin 自带 identity 块）
  UWB_IMU_PL_REPLAY_EXPORT_DIR=$E/raw/replay_p2/<NAME>/exports \
  UWB_IMU_PL_REPLAY_ATTEMPTS=<list> \
  devel/lib/uwb_imu_pl/r0_r1_development $CFG $E/raw/replay_p2/<NAME> <EPOCHS> <SCENARIO> $SCN
  # A:30, C:25,26, F:30, G:10, H:201,205, epochs20-census:30
  # 另有 epochs20 census 的 FULL_AUDIT 运行：raw/runs_p2/epochs20-census
  # census 变体重跑：raw/census_p2/sd（research-double-faults-census.yaml，order=2）

# 独立 Python oracle（A4-8）
  python3 $E/tools/oracle_compare.py            # -> oracle-results.json
  # 8 窗口 x O0..O7；结论 57 PASS / 0 FAIL / 7 NOT_RUN（NOT_RUN 均为缺数据）

# 验证调度与清单（A4-13）
  python3 $E/tools/run_validation.py --all      # -> validation-report.json
  # 13 PASS / 0 FAIL / 28 NOT_RUN（未开始项标注归属工作包）

# replay 解析自检（Python + C++ 两条路径）
  python3 -c "from replay_io import read_replay; read_replay('<bin>')"   # 8/8 通过
  devel/lib/uwb_imu_pl/candidate_replay <bin> /tmp/out.csv 4 1 oracle    # exit 0
```

P2 与 P1 的差异（口径说明，供后续引用）：

| 项 | P1 | P2 |
|---|---|---|
| 诊断 schema | v10 | v11（attempts 新列 + `diagnostic_snapshot_identity.csv`） |
| replay bin | v4 | v5（v4 布局 + identity；v1–v4 仍可读） |
| 场景证据目录 | `raw/runs`、`raw/replay` | `raw/runs_p2`、`raw/replay_p2`、`raw/census_p2`（P1 目录保留不动） |
| 生产源码 | 零修改 | A3/A4 范围内修改（清单见最终报告）；阈值/算法语义不变 |

## 6. P3（B1）命令、存储与验证

```
# 构建 + 全量测试（P3 口径）
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin build uwb_imu_pl
  catkin run_tests uwb_imu_pl                 # raw/run_tests_b1.log -> 254 tests / 0 failures
  catkin_test_results build/uwb_imu_pl/test_results/uwb_imu_pl

# B1 单元测试（上下文）
  devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_square_root_context   # 7 用例

# 场景重放（离散零差异判据的当前侧；大产物写 /tmp）
  O=/tmp/uwb_imu_pl_b1_20260921/b1_runs_v12
  UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \
      $CFG $O/<SCENARIO> <EPOCHS> <SCENARIO> $SCN      # A/C/D/E/F=30, G=45；H=226（无 FULL_AUDIT）

# 独立 oracle（仓库内，读 /tmp 的 run CSV；环境变量可改）
  python3 $E/tools/oracle_compare.py            # UWB_IMU_PL_ORACLE_RUNS=$O
  python3 $E/tools/context_oracle.py            # 读 fixtures/frames-b1.json 的 5 个 bin

# 验证调度（P3 起位于仓库 tools/integrity/）
  python3 tools/integrity/run_validation.py --all    # -> $E/validation-report.json

# 存储纪律（本轮起）
#   大产物（逐帧 CSV/trace/日志）写 /tmp/uwb_imu_pl_b1_<date>/；仓库只保留摘要
#   doc/evidence 单文件 <=1MB、整目录 <=20MB；raw/ <=15MB（prune-log.md 记录删除清单）
#   本轮 du：raw/ 7.5MB，doc/evidence/integrity-kernel-refactor/ 8.3MB（results/ 1.2GB 未动）
```

本轮 v12 诊断新增：`diagnostic_square_root.csv`（每窗口证书：rank/dof、R 对角、条件估计、
identity/parity/solution 残差、前向界、detector_only_rows、策略与可用性），
`hypotheses.csv` 追加 `z_rank/z_sigma_min/z_condition/z_classification`。

## 7. 工作树卫生事件（P4/B2 Stage 0，2026-09-21）

**事件**：P3 提交（9d5550f）之后、B2 开始之前，工作树出现一次**未请求**的全文件重排
（27 个源文件：注释折行重排、空行删除，`git diff --ignore-all-space` 仍显示 3,427 行变化，
并破坏了 `square_root_context.hpp` 的约定注释块）。该 pass 晚于 P3 全部证据、破坏
`hashes-B1.txt`、未经构建/测试。

**处置**：`git restore -- include src test` 全部回退；回退后工作树仅剩未跟踪的路线图文档，
`hashes-B1.txt` 复核 **109/109 OK**。

**规则（本轮起生效）**：
1. **禁止未请求的全文件重排/格式化**。任何格式化必须由用户明确要求，并且：
   单独 style commit、附带 `.clang-format`、重建 + 全量测试 + 场景抽查、重算哈希；
   **不得与功能提交混在一起**。
2. 任何"看起来像工具自动改动"的 diff（注释折行、include 排序、大范围空行变化）
   在提交前必须先用 `git diff --ignore-all-space --stat` 判别，并在报告中说明来路。
3. 每轮收尾仍然遵守：`hashes-<轮次>.txt` 最后生成；生成后不得再改任何文件。

## 8. P4（B2）命令、计数与验证

```
# 构建 + 全量测试（P4 口径；B2 新增 15 个用例 ×2 计数）
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin build uwb_imu_pl
  catkin run_tests uwb_imu_pl          # -> 284 tests / 0 failures
  catkin_test_results build/uwb_imu_pl/test_results/uwb_imu_pl

# B2 单元测试（快照）
  devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_integrity_v2 \\
      --gtest_filter='B2Registry.*:B2Compact.*:B2Coverage.*'
  devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_integrity_reference \\
      --gtest_filter='ReferenceFixture.GEO05*'

# 场景重放（stage3 = 最终证据侧；大产物写 /tmp）
  O=/tmp/uwb_imu_pl_b2_20260921/stage3
  for spec in "A_nominal 30" "C_uwb_fde 30" "D_imu_bridge 30" "E_union 30" \\
              "F_ramp_unmonitorable 30" "G_continuous_rejection 45"; do
    set -- $spec; mkdir -p $O/$1
    UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \\
        $CFG $O/$1 $2 $1 $SCN > $O/$1/stdout.log 2>&1
  done
  UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \\
      $CFG $O/H_mature_union 226 H_mature_union $SCN

# 等价（baseline = B1 运行，v12 诊断）
  UWB_IMU_PL_EQUIVALENCE_BASELINE=/tmp/uwb_imu_pl_b2_20260921/baseline_b1 \\
  UWB_IMU_PL_EQUIVALENCE_CURRENT=$O \\
  python3 $E/tools/equivalence_compare.py        # 0 离散差异，worst_numeric_rel 0.0

# 独立 oracle（新增 O8g–O8j 覆盖证书检查）
  UWB_IMU_PL_ORACLE_RUNS=$O python3 $E/tools/context_oracle.py   # 42 PASS / 0 FAIL / 8 NOT_RUN

# 运行/诊断 schema 校验
  python3 tools/validate_run_schema.py $O/A_nominal        # PASS uwb-imu-pl/v5
  PYTHONPATH=tools python3 test/test_gate_d_tools.py       # 13 tests OK

# 验证调度
  python3 tools/integrity/run_validation.py --all          # 22 PASS / 0 FAIL / 20 NOT_RUN
```

计数口径（可复核，`coverage-envelopes.md` §4）：`fault_cross_blocks` 只统计被假设
引用的 (mode, mode) 对；`mode_dense_allocations` 统计**逐模式 padded** 分配（紧凑
路径必须为 0，回退路径 = 有效模式数）；`compact_mode_rows` 为各模式实际写入行数之和，
`compact_padded_equivalent_rows = 窗口行数 × Σq` 为 padded 等价成本。

本轮 du：`raw/` 7.5MB 未变；`doc/evidence/integrity-kernel-refactor/` ≈ 8.4MB
（新增 `coverage-envelopes.md`）；大产物全部在 /tmp/uwb_imu_pl_b2_20260921/。

## 9. P5（B3+B4）命令、Stage 0 取证与处置

**Stage 0（工作树卫生，2026-09-21 15:47 批次）**

* 现象：10 个已提交文件被改写（`git diff --stat` 1550+/1130-，`--ignore-all-space` 同量级）。
* 取证：**token 多重集对照**（去注释、include 引号/尖括号统一、相邻字面量拼接后比较）
  → 10 个文件 body token 序列**逐一相同**、include 集合相同 ⇒ 纯格式化（include 重排/引号→尖括号、
  折行、字符串拆分、注释重排），无语义改动。
* 触发源：`find ~ -newermt '2026-09-21 15:45' ! -newermt '2026-09-21 15:50' -type f` 显示
  写入顺序为 12 个文件（含内容未变的 `CMakeLists.txt`、`coverage-envelopes.md`）→ 紧随其后
  `~/.cache/vscode-cpptools/.browse.VC.db*` 重建、`User/History/*` 逐文件产生条目，随后 `.git/index`。
  `entries.json` 中该批条目的 `source` **为空**（与本 agent 的 `Chat Edit: ...` 条目形态不同）。
  仓库内无 `.clang-format`，工作区 `settings.json` 无 formatOnSave，安装的格式化扩展为零，
  且 clang-format 不会把引号 include 改写成尖括号 ⇒ **无法确认为用户侧自动工具**（限时取证到此）。
* 处置（按指令 (b)）：`git restore -- apps include src test` 回退 10 个文件；
  `sha256sum -c hashes-B2.txt` 复核 **115/115 OK**；工作树仅剩未跟踪路线图文档。
  本 agent 的收尾流程不含任何格式化步骤（仅文件创建/替换工具），本轮亦未引入 `.clang-format`。
* 规则沿用 runbook §7：禁止未请求重排；提交前 `git diff --ignore-all-space --stat` 判别；
  `hashes-*` 最后生成。

**P5 命令**

```
# 构建 + 全量测试（P5 口径）
  catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl   # 300 tests / 0 failures
  devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_integrity_v2 \
      --gtest_filter='B3Risk.*:B3ZeroSpace.*:B4LazyFde.*'

# 场景（P5 基线 + B3+B4 侧）
  O=/tmp/uwb_imu_pl_b5_20260921/b3b4
  for spec in "A_nominal 30" "C_uwb_fde 30" "D_imu_bridge 30" "E_union 30" \
              "F_ramp_unmonitorable 30" "G_continuous_rejection 45"; do
    set -- $spec; mkdir -p $O/$1
    UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT=1 devel/lib/uwb_imu_pl/r0_r1_development \
        $CFG $O/$1 $2 $1 $SCN > $O/$1/stdout.log 2>&1
  done
  devel/lib/uwb_imu_pl/r0_r1_development $CFG $O/H_mature_union 226 H_mature_union $SCN

# 验证调度（报告以**代码提交** SHA 生成：本轮 = e40e2ef）
  python3 tools/integrity/run_validation.py --all          # 22 PASS / 0 FAIL / 20 NOT_RUN
```

提交纪律：两个相邻提交 ① 代码 `e40e2ef` ② 证据（本文件所属提交，含以 ① SHA 生成的
`validation-report.json` 与 `hashes-B3.txt`）；不 push；不提交/删除路线图文档。

## 10. P6（C 包 Stage 0 + C1/C2 设计冻结）2026-09-21

**Stage 0-a 工作树卫生（第二次同类事件）**

* 现象：6 个已提交文件被改写（`git diff --stat` 1519+/1171−）。
* 判定：token 多重集对照 → 6 文件 body token 序列与 include 集合**逐一相同** ⇒ 纯格式化。
* 触发源：文件写入时刻 **17:45:17**，同一窗口内 `~/.codex/`（`logs_2.sqlite` 17:34、
  目录 17:46）处于活动状态——即**另一个 CLI agent（Codex）会话**在本工作区运行；
  与 P5 事件（15:47 批次、Local History `source` 为空、无 `.clang-format`）同型。
* 处置：`git restore -- include src`（6 文件）；`sha256sum -c hashes-B3.txt` 复核 **101/101 OK**；
  工作树只剩未跟踪路线图文档。规则沿用 §7/§9；本 agent 流程仍无任何格式化步骤。
* 结论（两次事件合并）：格式化 pass 来自**本仓库之外的 agent/编辑器侧**，不由本 agent 引入；
  若后续仍复现，建议由用户侧确认其工具链（Codex/编辑器）并统一约定。

**Stage 0-b/c 小项**

* 支配性单侧保守：`CoverageEnvelope` 增加 `dominance_ratio`（原始比值，审计用），
  接受条件改为 `ratio >= 1 − dominance_tolerance`（相对 eps=1e-9，只吸收 binary64 噪声，
  实测共享恒等式噪声 ~1e-16），`dominance_margin = max(0, ratio−1)` ⇒ **接受时非负**；
  拒绝路径与 `B2Coverage.*` 6 例仍全 PASS。
* gate 归因：`stateStepAttribution()`（`joint_window_detector`）把窗口增量按 `state_layout`
  分段为 rotation/position/velocity/accel_bias/gyro_bias 幅值并指出主导历元与块；
  监测器仅在 step-gate 拒绝时把它追加到 `reason` 与 `skip_reason`（阈值/判定不变）。
  回归：`GateAttribution.*` PASS；G 场景 40/40 拒绝候选带归因，离散/PL 与 P5 逐帧 0 差异。
* validation 清单：去重（FDE-01/02 重复项），FDE-01/02 映射到 B4LazyFde.*，PRV-02 note 更新到 v14，
  新增 GAT-01；报告以本轮代码提交 SHA 生成。

**C1/C2**：设计冻结见 `history-summary-design.md`（表示、符号约定、生命周期表、容量动作、
重线性化规则、冷启动、oracle 计划、C2 接口、实现触点、未执行原因）。实现与验证本轮 NOT_RUN。

## 11. P7（Stage 0 遗留补救）2026-09-21

**S0-1 第三次格式化事件（例行程序：先检测再回退）**

* 18:01:43 批次改写 `coverage_envelope.hpp` / `.cpp`（+58/−52）。token 多重集对照
  （去注释、include 归一、字面量拼接）显示 body token 序列与 include 集合**逐一相同** ⇒ 纯格式化。
* 处置：`git restore -- include src`；工作树仅剩未跟踪路线图文档。取证与 §10 一致（仓库外
  Codex 侧工具链），本轮不再重复取证。

**S0-2 P6 假声明更正（重要）**

* **事实**：P6 报告与 `proof-obligations §16`、`baseline-report §11` 声称"单侧支配性已实现"，
  但 `src/.../coverage_envelope.cpp` 实际仍是**绝对**判据
  `envelope_bound.slope + dominance_tolerance < leaf_bound.slope`，且初始化行重复
  （`dominance_ratio` 赋两次）；头文件注释已改而实现未改 ⇒ **文档超实现**。
  根因：P6 中一次 `replace_string_in_file` 因路径笔误失败后未复查即继续，且当时未对
  "文档声明 vs 实现"做逐条核对。
* **更正**：**于 P7 提交 `6484b73` 完成**。实现为单侧相对判据
  `ratio = B_env / B_leaf >= 1 - dominance_tolerance`（相对，默认 1e-9），
  判据抽为可测函数 `envelopeDominanceAccepts()` / `envelopeDominanceMargin()`；
  `dominance_margin` 语义**选定为原始 `ratio − 1`**（接受时 `>= -tol`，tol 仅为
  binary64 噪声地板，实测共享恒等式噪声 ~1e-16），`dominance_ratio` 存同一最小值；
  重复初始化行已删除；新增窄带回归 `EnvelopeDominance.*`：
  `env = leaf·(1−1e-7)` 必须**拒绝**（旧绝对逻辑误接受），`env = leaf·(1−1e-10)` 接受且
  `margin >= -tol`，`env = leaf` 接受且 `margin >= 0`，非有限输入拒绝。
* **口径统一**：代码、头文件注释、本文件、`proof-obligations §16`、`baseline-report §11`
  对 margin 语义逐字一致（原始 `ratio−1`，接受时 `>= -tol`）。
* **流程改进**：每轮收尾新增一步核对——凡文档声明"已实现"的条目，逐条给出对应代码位置
  （文件+函数）或测试名，否则一律标 NOT_RUN。

**S0-3 哈希覆盖纪律（P6 遗漏更正）**

* **事实**：`hashes-C1C2.txt`（105 条）**未覆盖** P6 代码提交 `e97edee` 的改动文件
  （`coverage_envelope.hpp/.cpp`、`joint_window_detector.hpp/.cpp`、`integrity_monitor.cpp`、
  `test_integrity_v2.cpp` 中至少 `coverage_envelope.*` 缺失）：生成时使用
  `git status --porcelain`（当时代码文件已在提交中，故不在 status 输出里）。
* **更正与规矩**：每轮哈希清单必须由
  `git diff --name-only <base>..<head>` ∪ 保留证据树（`find doc/evidence/integrity-kernel-refactor -type f`）
  机械生成，任一缺失即不合格。P7 起按此执行（`hashes-C1C2.txt` 已重生成，含全部漏项）。
* 本 agent 收尾流程仍无任何格式化步骤；两轮报告均把格式化事件记为"外部工具链所致"。

**S0-4 报告口径修正（P7 新增）**

* `tools/integrity/run_validation.py` 新增 `UWB_IMU_PL_VALIDATION_SHA` 覆盖：当轮的证据提交
  已叠在代码提交之上时，仍按**代码提交** SHA 生成 `validation-report.json`
  （P7：`UWB_IMU_PL_VALIDATION_SHA=6484b73 python3 tools/integrity/run_validation.py --all`
  → `run_sha=6484b73`，29 PASS / 0 FAIL / 18 NOT_RUN）。该 runner 改动随本轮证据提交一起入库。
* 本轮哈希清单按 §11 的机械规则生成（`git diff --name-only d58a45d..HEAD` ∪ 证据树，
  排除本轮自身哈希文件）：**112 条，112/112 OK**。

## 12. C1-a（历史消元摘要模块）2026-09-21

**S0-A 第四次格式化事件（例行程序：先对照再回退，不重复取证）**

* P7 提交后工作树仅 `include/uwb_imu_pl/integrity/coverage_envelope.hpp` 被外部工具链
  改写：单 hunk，+2/−3——删除的 `joint_window_detector.hpp` include 指令在文件底部
  原样重插，两处空行合并；token 多重集不变（仅 include 块顺序/空行变化）⇒ 纯格式化。
* 处置：`git restore -- include/uwb_imu_pl/integrity/coverage_envelope.hpp`；工作树恢复
  （仅剩未跟踪路线图文档）。
* 哈希复核：`sha256sum -c hashes-C1C2.txt` → **114/114 OK，零失配**。任务书预期
  113/113；实测条目数 114（121 行 = 7 行注释 + 114 条）。机械并集重算复核：
  `git diff --name-only d58a45d..c6544d5` = 15 ∪ P7 收尾时证据树 109（现 112 − 本轮
  新增 3）= 去重 115 − 自身 1 − 瞬态 `tools/__pycache__/*.pyc`（.gitignore）1 = 114，
  与文件逐条一致；§11 叙述中的"112 条"为生成前时点数字，以文件实测为最终口径。
  事件按例行程序记录（取证口径见 §10，成因不再深挖）。

**S0-B 基线**

* HEAD=`c6544d5`；`catkin build uwb_imu_pl` + `catkin run_tests` →
  **304 tests / 0 errors / 0 failures**（本轮起点）。

**C1-a：模块（未接入管线）**

* 代码提交 **`bc5722c`**（模块 + 测试 + CMake；run_sha 口径）。
* 命令：
  * `catkin build uwb_imu_pl`；
  * `make -C build/uwb_imu_pl test_history_fault_summary`；
  * `devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_history_fault_summary`
    → **9 tests / 9 PASSED / 0 FAILED**（含 `[HISM1-TABLE]` oracle 表输出，
    日志 `raw/run_tests_c1a.log`）；
  * `catkin run_tests uwb_imu_pl` + `catkin_test_results` →
    **322 tests / 0 errors / 0 failures / 0 skipped**（= 304 + 2×9，
    日志 `raw/catkin_test_results_c1a.txt`）。
* 恒等式/容差/拒绝用例与 oracle 表：见 `history-summary-module.md`（κ(H_o)=1e8 档
  实测 ≤5.65e-9 @1e-7；κ=1e10 档按固定 1e-6 档保守对照并 tagged）。
* validation：`UWB_IMU_PL_VALIDATION_SHA=bc5722c python3 tools/integrity/run_validation.py
  --all`（新 `HIS-M1` MAPPED 至 `test_history_fault_summary`；`HIS-01..06` 保持
  NOT_STARTED，note 更新为"模块级恒等式已完成（HIS-M1）；管线等价待 C1-b"）；
  结果 **30 PASS / 0 FAIL / 18 NOT_RUN**（日志 `raw/validation_c1a.log`）。
* **runner 附带效应与处置（记录在案）**：`--all` 会重跑 `context_oracle.py` /
  `oracle_compare.py` 并改写两个派生 oracle 工件（`square-root-oracle.json`、
  `oracle-results.json`）。工具按磁盘现存 /tmp 转储自动选源；P2 的
  `/tmp/uwb_imu_pl_b2_20260921/stage3` 已被清理，本次自动选中 b1 时代转储
  （b1_runs_v12，缺 v13+ 字段）→ `square-root-oracle.json` 出现 8 条 O8g/O8h
  **环境性降级（非回归）**。处置：两工件**恢复为提交状态**（P7 验证过、0 FAIL），
  不提交降级产物；如需刷新须在正确转储可用时以显式 runs-source 重跑。validator
  报告本身保持本轮生成（其 PRV/COV 行为工具退出码口径，未受影响）。
* 生产行为零变化：模块无生产调用点；未改检测/PL/风险/阈值/配置/场景；未做
  C1-b 适配器骨架（可选项，NOT_RUN，避免半成品）。
* 哈希：`hashes-C1C2.txt` 按机械规则最后重生成（base `c6544d5` → 代码提交
  `bc5722c` ∪ 证据树，排除自身与瞬态 `__pycache__/`）；**115 条，115/115 OK**。
  不 push；无格式化步骤。

## 13. C1-b/C1-c 本轮：交付（C2/B5/C3 载体）与阻塞（B1）2026-09-21

**S0-A 第五次格式化事件（例行程序：先对照再回退，不深挖）**

* 外部工具改写 3 个 C1-a 文件：`history_fault_summary.hpp/.cpp`、
  `test_history_fault_summary.cpp`（include 重排、行连接/断行、对齐空格）。
* token 对照：白空格无关的 token 多重集**逐一相同**（1926 / 2542 / 8257 tokens；
  先前 `tr` 口径的"差异"仅为行连接处的空白 token 边界伪差，本回合改用正则
  tokenizer 复核）。
* 处置：`git restore` 3 文件；工作树恢复；`hashes-C1C2` **115/115 OK，零失配**。

**S0-B 基线**：HEAD=`4e5c1ad`；`catkin build` + `catkin run_tests` →
**322 tests / 0 errors / 0 failures**（本轮起点）。

**本轮交付（代码提交 `23f6e18`）**

* C1c-C2 配置层：`history.{max_summary_rows,max_fault_columns,max_perp_rows,
  capacity_action}` 严格加载（未知键/缺键/非法动作拒绝；节缺省=零容量默认）——
  `IntegrityConfig.HistoryCapacityKeysAreStrictlyLoaded`。
* B5：`StatisticalBoundKey.history_summary_version` + `envelope_kind` 进入
  `NoncentralKey`（修复"只声明未哈希"的静默别名缺陷）——
  `B3Risk.HistorySummaryVersionIsPartOfCacheIdentity`。
* C3 载体：`HistorySummaryVersion` + `digestHistorySummaryVersion`——
  `HistoryFaultSummary.HISM2VersionDigestBindsAllComponents`。
* 全量测试：**328 tests / 0 errors / 0 failures**（= 322 + 2×3）。
* validation：`UWB_IMU_PL_VALIDATION_SHA=23f6e18 python3
  tools/integrity/run_validation.py --all` → **32 PASS / 0 FAIL / 18 NOT_RUN**
  （新 `HIS-C1`/`HIS-V1` 均 PASS；`run_sha=23f6e18`）。runner `--all` 的派生
  工件副作用处置同 §12（两个 oracle JSON 恢复为提交状态，陈旧转储会产出环境性
  降级，非回归）。
* 哈希：机械规则最后重生成（base `4e5c1ad` → 代码提交 `23f6e18` ∪ 证据树，
  排除自身与瞬态 `__pycache__/`）；**123 条，123/123 OK**。不 push；无格式化。

**B1 核心阻塞（不闭合；逐条见 `history-summary-b1c-blockers.md`）**

* 原因：①历史故障列无构造路径（生成器要求窗口块，被消元组为
  `UnrecoverableHistory`，`hypothesis_generator.cpp:675-700` /
  `integrity_window_snapshot.hpp:62-79`）；②行抽取口径待 GTSAM 原型验证
  （`incremental_estimator.cpp:1221-1224` 现为正规矩阵口径）；③20+ 数值敏感
  断言与外部 oracle 需同轮重基线（影响面清单见 blockers §5）。
* 所需输入：历史故障参数化子系统 → 生命周期/容量执行语义决策 → 行抽取原型 →
  全量重基线。
* 未做 B1-lite / default-off 半成品集成（纪律说明见 blockers §3）；`auto_shrink`
  仓库无设置点（grep 0 命中），"保持关闭"无需改动。
