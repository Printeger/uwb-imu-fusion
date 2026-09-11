# IMU 辅助残差 FDE Stage1 实施与六输入结果

日期：2026-09-11

结论：`IMPLEMENTED_AND_ENGINEERING_VERIFIED / SIX_INPUT_NEGATIVE_INCOMPLETE`。
论文主代码路径已切换为 `imu_aided_fde`，legacy `automatic_discovery` 保留。工程门和 Walk1
`[8,11]s` smoke 通过；六输入中没有 producer 发布 Stage2 cache，因此没有 candidate-dependent
final 轨迹，也没有可用的 LCB 相对 suppress 精度结论。T10=C2-C、T11=C 和 C1--C3 边界不变。

## 1. 基线、初始状态与范围

- 起始和结束代码基线均为 `HEAD=2f9c2e41cbda6b2911cc9c716e2fb0b1347eb0c2`；本轮未 commit、未 push。
- 开始时唯一工作区项为用户已有未跟踪目录 `doc/v2/ie_0911/`；未修改、删除或纳入本轮结果。
- 冻结 `doc/v2/paper_structure.tex` 和 roadmap 未修改。
- 只实现本轮 FDE amendment、工程测试、固定 smoke、六输入矩阵、锁定 GT 评价与文档同步；没有调参或新增研究问题。

## 2. 合同 amendment

`AGENTS.md`、`STATUS.md`、`METHOD_CONTRACT.md` 和 `EXPERIMENT_CONTRACT.md` 已登记：

- `imu_aided_fde` 是论文 Stage1 主路径；`automatic_discovery` 仅保留为 legacy/development。
- 前端属于 RAIM/FDE-family residual front end，不是完整 ARAIM，也不是 certified integrity method。
- production FDE 不接受 GT/oracle；空候选是成功状态并进入 raw Stage2。
- Stage2、$R_c$、local sigma、LCB、live-$C$ legacy final、suppress 和一次 fallback 定义不变。

## 3. FDE 方法与输入

新增 `include/uifgo/nlos_fde.h` 和 `src/nlos_fde.cpp`。`ImuAidedFdeSupportProvider` 只接收物理
graph/initial `Values`、UWB factor metadata、input plan、配置和版本化 identity context。

reference 使用与 fixed-rejection 共用的 tightly-coupled checked LM。对每个 `valid && planned` 的真实
`uwb_range` factor，一一读取 `unwhitenedError()[0]` 与 factor noise：

```text
r = h + beta - z
q = r / sigma
T = q^2
fault = T > Chi2inv(0.99, 1) = 6.6349
positive_excess = r < 0
candidate = fault && positive_excess
```

等于阈值不剔除。大正 residual 只记录双边 fault，不成为 positive-excess candidate。

## 4. 确定性 temporal partition

记录按 `(tag_id, anchor_id, timestamp, obs_id)` 稳定排序。同 link 健康观测立即闭段；candidate 间隔
严格大于 `gap_threshold_s` 才分段。run 必须同时满足 `count >= minimum_count` 和
`duration >= minimum_duration_s`。短 run 从 partition 删除，但 observation CSV 保留 candidate、raw run ID
及过滤原因。segment ID、ordinal、snapshot/context/partition hash 均确定性生成；没有 amplitude、
change-point、merge、L1/TV、ADMM 或 outer loop。

## 5. Runner、配置与 legacy 兼容

- `ConfigLoader` 接受并严格校验 `nlos.mode: imu_aided_fde`、显式 temporal 字段及
  `solver.chi2_reject_prob: 0.99`；拒绝 oracle support 和其他概率。
- 7 份 `config/paper/ie0911/*.yaml` 论文输入配置切到 FDE，删除不消费的 legacy discovery/ADMM 字段；
  Stage2、$R_c$、LCB/final 与原 gate 参数保持不变。
- Runner 独立执行 FDE；成功时以 reference LM `Values` 作为 Stage2 初值；拒绝
  `structured_bias_only + imu_aided_fde`。
- fixed-rejection 改用同一 preliminary LM 与 scalar factor residual/noise reader；原 mask/final 语义不变，
  legacy automatic-discovery 测试继续通过。

## 6. Artifact 与 cache 准入

每个 FDE producer 无论成功失败均写：`fde_status.json`、`fde_observations.csv`、
`support_partition.json` 和字节等价兼容文件 `partition.json`。status 包含 provider/version、reference、
p/DoF/threshold、planned/tested/fault/candidate/raw/filtered/retained 计数、partition identity 和
`gt_read=false`。

保留 cache schema v2 和 `AUTO_DISCOVERY` namespace 名称。FDE Stage1 identity 绑定 provider/version、
input/source/common preparation、p/DoF/threshold、temporal 参数、preliminary LM、物理 graph/initial Values、
calibration；不绑定仅供 Stage4 使用的临时 gate YAML 字节。发布端强制校验 3 个 FDE artifact、partition
字节等价和 provider；final replay 校验 mode、Stage1 hash 和 partition provider。测试证明 legacy/FDE
payload/cache identity 不可互用。

## 7. 工程测试

最终留档命令均为 exit 0：

```text
catkin build uwb_imu_fgo --no-status --summarize --jobs 2
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target tests -j2
cmake -E chdir /home/mint/ws_fusion_uwb/build/uwb_imu_fgo ctest -R "test_nlos_fde|test_config|test_paper_methods|test_fde_runner_contract|test_ie0911_evaluator" --output-on-failure
cmake -E chdir /home/mint/ws_fusion_uwb/build/uwb_imu_fgo ctest --output-on-failure
```

结果：targeted 5/5；完整 CTest 29/29，0 failure，real time 126.82 s。覆盖 residual 方向、阈值严格性、
健康/gap 切断、短段删除、factor noise、planned 一一覆盖、空候选 success、persistent `+0.4/+0.6 m`
fixture、identity 敏感性、失败与 provider-aware cache 完整性。日志：

- `/home/mint/ws_fusion_uwb/res/ie0911_fde_smoke_20260911_03/engineering_logs/catkin_build.log`
  (`sha256:6c6199d33151682838330adfe26b5b836e7c05c551ab3bf7d1812ac5edf372c1`)
- `build_tests.log` (`sha256:86b82941fce258ea24c9fa6d5434b4b38a86bb94044ddc53630217c9289eb2ab`)
- `targeted_tests.log` (`sha256:1c9f4cc4ef49a7cb93e1769cb00e4062b800fc78adf4f6d60e430a93a130f9b8`)
- `ctest.log` (`sha256:a0d1a39b14dd4edefea26e92169179094fad143761e637f75879f5d0da2ff149`)

## 8. Walk1 `[8,11]s` smoke

固定 manifest 命令使用 `config/paper/ie0911/smoke_batch.yaml`。所有尝试均保留：

- `_01`：producer 成功，四个 final 被错误包含 Stage4 YAML 字节的 Stage1 identity 拒绝；修复为只绑定
  Stage1/common/physical graph 语义。
- `_02`：cache replay 成功，suppress/structured 成功；空支持 LCB 在 replay 误调用非空 score validator；
  修复为空支持直接复用缓存空 score，保持 raw final 定义。
- `_03`：最终通过。FDE 55 planned/55 tested、0 fault、0 candidate、0 segment，状态
  `REFERENCE_AND_ALL_TESTS_COMPLETE_EMPTY_SUPPORT`；raw Stage2、suppress、structured、LCB partial 和
  LCB fixed-full 均成功，四个 final 共用 cache
  `t09stage2cache-sha256:3e1c86bbc332e371866645cf62a1939fb1eafc6e433d4b13ae56a68e37d91a99`。

最终目录：`/home/mint/ws_fusion_uwb/res/ie0911_fde_smoke_20260911_03`；batch manifest
`sha256:d101c98c1719ad7e6f9a459770b4090b6d0cd51e06f7b4b74c5eefde683cb407`。producer 的
`support_partition.json` 与 `partition.json` 同为
`sha256:96eac8d169884361829f2241d53f300d3f026b72c0c22a9712864211459ce813`；目录无
discovery/ADMM/outer-trace artifact，也无 `MAX_OUTER_ITERATIONS`。

## 9. 六输入五方法矩阵

实际目录：`/home/mint/ws_fusion_uwb/res/ie0911_fde_six_20260911_01`。36 个调度 cell 全部到达终态：
4 `COMPLETE`、8 `COMPLETE_WITH_RUN_FAILURE`、24 `PARENT_CACHE_UNAVAILABLE`。

| 输入 | FDE | planned/tested | fault/candidate/segment | Stage2/cache | Cauchy |
|---|---|---:|---:|---|---|
| SFUISE Walk1 | SUCCESS empty | 1074/1074 | 0/0/0 | raw Stage2 stop conditions failed；无 cache | COMPLETE |
| SFUISE Walk2 | SUCCESS empty | 1357/1357 | 0/0/0 | raw Stage2 stop conditions failed；无 cache | COMPLETE |
| SFUISE Walk3 | SUCCESS empty | 1483/1483 | 0/0/0 | raw Stage2 stop conditions failed；无 cache | final LM max iterations |
| MiLUV random | reference LM failed | 2427/0 | 0/0/0 | NOT_RUN；无 cache | final LM max iterations |
| MiLUV circular | reference LM failed | 1587/0 | 0/0/0 | NOT_RUN；无 cache | COMPLETE |
| Vicon test | reference LM failed | 1447/0 | 0/0/0 | NOT_RUN；无 cache | COMPLETE |

3 个 reference failure 均为 `PRELIMINARY_LM_FAILED:CONDITIONAL_LM_MAX_ITERATIONS`。没有按结果修改
p、temporal 参数、kappa、区间或任何 Stage2/final 阈值，也没有算法重跑。6 个 producer 均保留完整
FDE artifacts，且均无 discovery/ADMM/outer trace。

## 10. 锁定 GT 评价与聚合

所有 estimator 结束后，`evaluate_runs.py` 才读取 `config/paper/ie0911/step2_evaluation.yaml` 中已锁定的
六份 GT；哈希校验通过，命令 exit 0。扩展 paired evaluator 在同一 GT 交集上定义 RMSE、P95、horizontal
RMSE、vertical RMSE 的正 improvement 为 `suppress_all - method`，百分比除以 suppress；coverage 同时报
绝对百分点与相对百分比，零分母为 unavailable。

本轮 24 个 candidate-dependent final 均无 admitted cache/trajectory，因此上述 paired 指标、LCB 相对
suppress 变化、coverage 变化、$R_c$、LCB offset 和共同 cache identity 全为 `UNAVAILABLE/NOT_RUN`。
独立 Cauchy 4 条可评价轨迹的 aligned ATE RMSE 为 Walk1 `343.326 m`、Walk2 `709.994 m`、MiLUV circular
`266.702 m`、Vicon `1683.464 m`；Walk3 与 MiLUV random 无轨迹。这些严重漂移值不支持方法收益。

冻结聚合：

- `evaluation.json`：`sha256:888813537ba45c66a3c9e4fb3c33c992d3947b807c47de267969fe27feeb75b0`
- `fde_aggregate/fde_aggregate.json`：`sha256:30c0122490fcbd870a2a179ed2c0ea47772ba2fa7b1dc30ea1468b68ae86cd93`
- `fde_aggregate/fde_aggregate.csv`：`sha256:df4644d3d7309c9bb3d119d5eccd6a86f4f96c82d450e63b04cc6c98b3102e9e`

## 11. 交付、失败与 claim 边界

本轮修改：合同/状态/claim ledger/论文工作稿；FDE header/source；shared fixed-rejection 基元；config/runner；
provider-aware scheduler/cache；paired evaluator与新 aggregate；FDE/config/cache tests；7 份论文配置。

完整文件清单：

```text
AGENTS.md
CMakeLists.txt
config/paper/ie0911/{walk1_smoke,sfuise_walk1,sfuise_walk2,sfuise_walk3,miluv_random,miluv_circular,vicon_test}.yaml
doc/ie_sprint/{STATUS,METHOD_CONTRACT,EXPERIMENT_CONTRACT}.md
doc/ie_0911/FDE_STAGE1_RESULT.md
include/uifgo/{config,nlos_fde}.h
paper/{main.tex,CLAIM_EVIDENCE.md}
src/{config,nlos_fde,paper_methods}.cpp
test/{test_config.cpp,test_nlos_fde.cpp,test_fde_runner_contract.py}
tools/run_ie_paper.cpp
tools/paper/{run_experiments.py,evaluate_runs.py,ie0911_fde_aggregate.py}
```

失败均保留：smoke `_01` identity 接线、`_02` empty-LCB 接线、六输入 3 个 raw Stage2 stop failure、3 个
reference LM failure、2 个 Cauchy final LM failure和24个 parent-cache unavailable。前两项属于不改变方法
参数的 correctness fix；六输入算法结果未修复、未调参、未重跑。

支持的结论仅为：FDE 定义已实现；factor-residual/noise 一一审计、确定性 temporal partition、空候选、
provider-aware fail-closed cache 和 isolated artifacts 已通过工程验证。六输入不支持 trajectory improvement、
LCB benefit、recoverability operational value、完整 ARAIM/certified integrity、formal held-out RQ/U14 或
C1--C3 升级。未 commit、未 push。
