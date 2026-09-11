# 0911 第一步交付（2026-09-11）

状态：`IMPLEMENTED_VERIFIED_ENGINEERING_SCOPE / REAL_AUTOMATIC_SMOKE_FAILED_STAGE1`。
第一步实施与验证已结束；不宣称真实 LCB 全链通过。第二步精度矩阵 **NOT_RUN**。
T10=C2-C、T11=C、C1–C3 限制不变；原 `doc/v2/ie_0911/` 材料未修改。

## 实现与身份

基线 `282fe5f79a55a18fa07d658d09f52fed13af2d9a`，分支
`feature/uwb-imu-fusion-ie-postprocessing`。按用户后续明确指令提交并推送，提交身份以本文件所在 commit 为准；最终交付源码、配置、runner/core、GTSAM 哈希见
[CODE_IDENTITY.json](evidence/CODE_IDENTITY.json)，源码差异由该提交本身保存。
用户授权 amendment 已登记于 AGENTS、METHOD/EXPERIMENT_CONTRACT 和 STATUS。

- `LocalAmplitudeSigmas` 使用原物理 R，独立检查有限性、对称性、满秩和正定性，LLT 解单位向量求逆对角的平方根。不是 `1/sqrt(R_ss)`，不加 jitter、damping 或 prior。
- `lcb_partial` 按段冻结 `max(0,c_hat_stage2-2*sigma_c_local)`；结构/数值不可用及零补偿 suppress，不串联旧 eta/s/gamma 门槛。`lcb_fixed_full` 复用同一 LCB 集合，只把补偿改为 Stage2 幅值。
- 固定-offset factor 保留 raw z，残差 `h+beta+delta-z`，每条恢复观测一次；final 无 live C。原 `structured_debias` 保留联合 live-C 语义，旧默认不迁移。
- 固定模式复用 refitter 无 C 导航求解及一次 suppress fallback，不运行要求 live C 的 final recoverability 重评分。轨迹、残差与导航协方差均来自同一 final graph/Values；`segment_bias.csv` 只有表头，不伪造最终动态 bias 后验。
- `fixed_compensations.csv` 分列 Stage2 幅值、局部 sigma、固定补偿和 decision/final mask；`fixed_method.csv` 保存请求与实际方法。aggregate inference identity 额外绑定方法、kappa、全部冻结记录；修改冻结值后 exporter 拒绝。固定模式 `decisions.csv` 的组行仅汇总是否存在恢复段，逐段决定以固定补偿表和 observation masks 为准。
- 现有 `run_ie_paper.cpp` / `run_experiments.py` 接通四个主方法和全额 variant。候选方法共享自动 Stage2 cache 及 final 导航初值；Cauchy 独立运行。LCB cache replay 在已校验的同一 Stage2 graph/Values 上重算完整 R/列映射，**不重新优化**；旧 cache reader 只有 eta/s，不能拿它冒充完整 R。
- MILUV 复用 CSV loader，paper 路径固定 tag 10 和配置杆臂，group window=0，源行身份在选 tag/裁剪前保存。`--anchor-ids` 挑选实际配置 anchor；bag/CSV 时间裁剪和 anchor 选择都在初始化前。source/plan/config、源码和二进制参与缓存身份。
- 独立 evaluator 增加水平/高度 RMSE/P95、固定补偿评价和有独立几何/静态 beta/时基 provenance 的测距参考接口。后者是含噪测量参考，非 latent bias truth，更不是 post-fit residual。

## 六项验收

| 项目 | 实际结论 | 证据 |
|---|---|---|
| 1. 数值与选择规则 | PASS：相关 R 逆对角反例、幅值列置换、零补偿、空候选、非有限、秩亏、非正定 | [recoverability result](evidence/recoverability1.result.json)、[最终 CTest](evidence/ctest_final.result.json) |
| 2. 最终图与导出 | PASS：残差符号/Jacobian、raw 唯一、无 C、冻结值篡改拒绝、一次 fallback、同图 covariance | [最终 CTest](evidence/ctest_final.result.json)、[XML](evidence/final_test_xml/) |
| 3. 非零补偿 production final | PASS：实际 certified optimizer callback，partial/full 同集合；正常 final 无 fallback，受控失败 fallback 各一次 | [partial](evidence/fixed_production2_run/)、[full](evidence/full_production1_run/)、[独立重算](evidence/fixture_audit1.json) |
| 4. 六输入四方法入口 | PASS **加载/初始化/建图** 24/24，optimizer calls=0；不是 24 条轨迹通过 | [逐 run ledger](evidence/startup2_runs/ledger.json)、[身份/源行/裁剪审计](evidence/input_audit2.json) |
| 5. 真实固定 smoke + 独立评价 | Cauchy PASS；自动 Stage1 **FAILED/MAX_OUTER_ITERATIONS**，Stage2 和候选 final NOT_RUN；失败已进入分母 | [batch](evidence/smoke2_runs/batch_manifest.json)、[metrics](evidence/smoke_metrics.json) |
| 6. 构建、回归与交接 | PASS：真实 catkin 包构建 exit 0；CTest 26/26，GTest XML 161 项、0 failures/errors；第二步入口已配置但未执行 | [包构建](evidence/build_package_final.result.json)、[测试](evidence/ctest_final.result.json)、[命令索引](evidence/COMMAND_INDEX.json) |

## 非零补偿实例

来自已有两段含 bias 工程 fixture，经 `a19_r08_pipeline --ie0911-fixed-engineering` 实际 final 优化：

| segment | Stage2 c_hat (m) | local sigma (m) | partial delta (m) | 全额 variant delta (m) |
|---|---:|---:|---:|---:|
| r07-segment-0 | 0.39999999994194146 | 0.036036185607444378 | 0.32792762872705272 | 0.39999999994194146 |
| r07-segment-1 | 0.29999999994662963 | 0.036041928700793063 | 0.22791614254504350 | 0.29999999994662963 |

两种 normal final 均恢复 4 条候选、保留 4 条 reference；14 factors、6 个 X/V/B keys、0 C，covariance AVAILABLE。
partial 的 obs 70000 最终 residual 为 `-0.069309460114940435 m`。独立脚本从 `input_ranges.csv` 与
最终 `trajectory.tum` 重算两种模式全部 16 条 range residual，与导出在 `1e-10 m` 内一致。
这验证固定补偿进入了真实优化，**不证明定位精度改善或 LCB 具有已校准的安全概率**。
旧工程 summary 的 `candidate_excluded_*` 字段在新增固定模式中沿用了历史字段名；其数值是最终图规模，
固定模式实际包含恢复候选。以 `normal/final/final_factor_metadata.csv` 和独立审计为准。

普通 binary64/V2 fixture 的最初两次 LCB 求解分别因外层上限和 lambda exhaustion 触发 fallback，保留
[inference1](evidence/inference1.result.json)、[inference2](evidence/inference2.result.json)。未放宽容差；后续成功证据明确来自
既有 certified production 分支。单元测试使用显式 fault injection 验证确定性 fallback，不能冒充普通 solver 成功。

## 真实 smoke、六输入与语义限制

固定 Walk1 起始后 `[8,11]s`；科学设置运行前写入 [smoke manifest](../../config/paper/ie0911/smoke_batch.yaml)。
首次 launcher 使用错误的 SFUISE 绝对路径，加载前失败；只修路径后执行一次实际算法运行。
Cauchy exit 0；Stage1 在原 50 outer 上限失败（exit 1），不调整区间、kappa、预算或参数重跑。
`suppress_all / structured_debias / lcb_partial / lcb_fixed_full` final 均为 `PARENT_CACHE_UNAVAILABLE`。

Cauchy 轨迹：[trajectory.tum](evidence/smoke2_runs/runs/t09-b9c90d3ed8b94c5a89a5797ca75d4b6b/trajectory.tum)。
自动分支失败：[run_status.json](evidence/smoke2_runs/runs/t09-4f184d116f4d4234ada0c0d5937a4a4d/run_status.json)。
先冻结 37 项 estimator payload hash，再用独立 GT exporter/evaluator 评价；评价后 37 项仍一致。
没有 strace 级新 file-open 审计（NOT_RUN）；estimator 入口未调用 GT loader，GT 导出在另一进程、另一目录。

12 个匹配位姿的 development aligned ATE RMSE/P95 为 `0.1133224833 / 0.1592241293 m`，
水平 RMSE `0.1053822296 m`、高度 RMSE `0.0416721837 m`。这些仅为短窗口接线诊断；
GT tracker/IMU 点关系和独立 frame transform 未闭合，raw-frame 误差不可用，不进入论文主结果。
测距参考的独立 beta/geometry/time provenance 亦未闭合，真实测距补偿精度为 UNAVAILABLE。

六输入采用 SFUISE Walk1/2/3、MILUV random/circular、自有 no-obstacle bag（`vicon_test` 配置）；
不是 sim_circle。统一 [six_inputs.yaml](../../config/paper/ie0911/six_inputs.yaml)。

| 输入 | IMU | planned ranges | keyframes | anchors | 已验证范围 |
|---|---:|---:|---:|---:|---|
| Walk1 | 4839 | 1074 | 229 | 5 | 四方法 prepare |
| Walk2 | 6229 | 1357 | 293 | 5 | 四方法 prepare |
| Walk3 | 6684 | 1483 | 314 | 5 | 四方法 prepare |
| MILUV random | 42317 | 2427 | 2427 | 6 | tag10 四方法 prepare |
| MILUV circular | 27738 | 1587 | 1587 | 6 | tag10 四方法 prepare |
| 自有 no-obstacle | 14680 | 1447 | 367 | 4 | 四方法 prepare |

同一输入四种 method 的 common preparation identity 相等。额外实际检查了 SFUISE 四个真实 anchor 子集、
MILUV `[origin+8,origin+11]s` 裁剪及源 obs_id/行身份不变。六输入许可、独立 LOS beta、GT 点/杆臂/时钟标定
没有因 prepare 通过而升级：仍仅 development/portability，不是正式 benchmark 准入。

## 实际命令与首次失败

完整命令、退出码、耗时和本地日志索引：[COMMAND_INDEX.json](evidence/COMMAND_INDEX.json)。
每个前缀对应 `.command.json / .result.json / .log`（build1 使用 `.exit`）；原始日志保留在本地隔离证据目录，Git 交付保存命令、结果和关键 XML/审计产物。

- `build1/build2`：新增 anchor 入口误传 Config 值而非指针，exit 1/2；已修复，保留失败。
- `build3`：系统 gtsam_unstable 与 `/usr/local` GTSAM 混链，exit 2；CMake 显式绑定同安装目录后通过，不修改或升级依赖。
- `smoke1`：SFUISE 路径错误，loader 前失败，batch exit 2；首批启动检查中 Walk1/2/3 的 12 个失败同样保留。
- `inference3`：测试二进制尚在链接时启动，PermissionError，未执行测试；人工记录 `NOT_LAUNCHED/126`。后续等待 build 完成再运行，最终 CTest 通过。
- scheduler 补齐 common preparation 前失败的逐 cell 记账，并保存实际 command；失败不会使后续独立方法消失。
- 普通求解与真实 Stage1 的算法失败均保持负结果；不通过改容差将它们改写成成功。

代表性实际命令：

```bash
catkin build uwb_imu_fgo --no-status --summarize --jobs 2
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target tests -j2
cmake -E chdir /home/mint/ws_fusion_uwb/build/uwb_imu_fgo ctest --output-on-failure
```

## 第二步启动命令（NOT_RUN）

下面会运行六输入精度比较，**本轮未执行**，使用新的输出目录。包含四个主方法、共享 cache producer 和全额消融。
真实 smoke 已暴露原 Stage1 上限失败，因此此命令可记录更多失败，不能预期必然产生每种方法的轨迹。

```bash
python3 tools/paper/run_experiments.py \
  --manifest config/paper/ie0911/six_inputs.yaml \
  --runner /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner \
  --output-root /home/mint/ws_fusion_uwb/res/ie0911_step2_01
```

仅检查入口的已验证命令形式：

```bash
python3 tools/paper/ie0911_startup.py \
  --manifest config/paper/ie0911/six_inputs.yaml \
  --runner /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner \
  --output /home/mint/ws_fusion_uwb/res/ie0911_prepare_01
```

第二步完整精度矩阵、六输入完整轨迹评价、正式 locked/test/held-out、真实测距参考指标、LCB 安全概率校准、
新 prefix/U13、论文数字/图表和 release 均 **NOT_RUN**。第一步完成即停止，不自动继续第二步。
