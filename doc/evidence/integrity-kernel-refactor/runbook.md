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
