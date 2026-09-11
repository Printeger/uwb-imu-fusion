# 0911 Recovery / post-fit FDE v2 结果

状态：**DONE / LOCALIZATION_AND_ENGINEERING_GATES_PASS / WALK1_ZERO_CANDIDATE**。
授权与定义见 [当前任务协议](../ie_sprint/RECOVERY_FDE_V2_PROTOCOL.md)。T10=C2-C、T11=C、C1–C3 不升级。
实施与验证阶段未自动提交或 push；任务完成后用户已明确授权提交并推送本轮实现。冻结论文结构、roadmap、`doc/v2/ie_0911/`、旧结果及默认配置未修改，受保护的未跟踪 `doc/v2/ie_0911/` 不纳入提交。

## 实现和原因

paper 全部 planned raw Gaussian graph 增加一次 checked LM reference；all-range/FDE 直接复用，Cauchy/Huber 从其 Values 开始。失败明确终止，prepare-only/cache replay 不求解 reference。新增 `raw_reference.json`，记录目标、状态、迭代、位置尺度和 graph/Values 身份。

单独 warm start 恢复 all-range，但 Cauchy 仍达到 100 次迭代预算。实际链接库的 Robust loss 与 IRLS 线性化不一致：Gaussian residual 被 robust 加权后再次送入 loss。固定 r=.6、sigma=.15、k=2.3849 的 Cauchy 反例：实际 loss=1.5714894027765274，标准 loss=3.806335111274521；FD 梯度=1.0554429921150188，IRLS 对应梯度=6.9935006712883085。paper-only `PaperRobustNoise` 修正 Mahalanobis 距离来源，保留原 IRLS、参数和 solver；未修改安装的 GTSAM 或 legacy。Cauchy/Huber 的标准 loss、FD 梯度与 IRLS 回归测试通过。旧 paper robust 数字不可与新版本混用。

本次受控差别是 raw LM 和此 correctness fix。paper yaw对齐关闭（yaw_align_frames=0）、固定 lever `[.1,-.025,0]`、1074 planned ranges、sigma=.15 等保持不变。legacy 使用 15 帧 yaw 对齐（实测358度）、分阶段开放 lever 标定及913条 range；未宣称这些差别无影响，也未分别证明其因果贡献。

FDE 使用同一 raw reference 的完整白化 Gaussian Jacobian，包括实际 IMU 交叉协方差、导航和 IMU bias 等全部未知量。一次 SparseQR，逐 UWB 行计算残差投影范数；复用既有列尺度、满秩证据和数值容差，不形成全图稠密逆/projector，不加入 damping/jitter/prior。不可确认秩或残差方差时失败，不回退 measurement r/sigma。provider/cache 身份升级到 `imu_aided_postfit_fde_v2`；保存 measurement sigma、post-fit residual variance、signed normalized residual、独立诊断 r/sigma、测试状态与 linearization identity。保持既有 p=.99、1 DoF 和固定查表阈值6.6349，严格大于，只有负 residual fault 参与原 temporal 聚合。

原始 support 为空且 reference/FDE/观测覆盖/graph/Values/obs_id/noise 审计通过，Stage2 返回 `SUCCESS_EMPTY`；与交替 refit 的 `CONVERGED` 分开。Stage2 optimizer=0，四个同图 final optimizer=0；cache 发布和 replay 核验求解证据及身份。原始非空但全 suppress 仍走旧 final/fallback，非空 Stage2、LCB、accepted live-C 定义未变。

## 定位门与 full Walk1

fresh legacy（禁用进程内 GT topic，独立 evaluator 评价）：**0.169642148677 m**；与历史0.169642149m差约3.23e-10m。
all-range：**0.163853268157 m**，Cauchy：**0.163847230941 m**。定位门通过。
三者均229匹配、0未匹配，相同时间范围 `[1664959676.9893188,1664959736.082964]`，paper完整输出229 planned keyframes。raw reference94次迭代，objective `2029100318.3603289 -> 128.93460199296939`，最大位置范数4.4662750263m。

固定20ms最近邻与scale=1 SE(3) aligned指标，GT仅在独立 evaluator 中用于最终门及五方法评价；锁定 GT SHA-256 `2bd90465cc6f9445e94c75087a293c36cfc25a4ac1165654a9765431c8816c33`。初次legacy复现沿用旧入口，在估计后做了内置GT评价；另执行 `legacy_03_no_gt` 禁用该读取，轨迹与初次复现逐字节相同，最终门采用后者。paper估计/FDE均不读GT/oracle。

| full Walk1 方法 | aligned RMSE (m) | aligned P95 (m) | matched / unmatched |
|---|---:|---:|---:|
| robust_cauchy | 0.163847230941 | 0.294238429213 | 229 / 0 |
| suppress_all | 0.163853268157 | 0.298978069968 | 229 / 0 |
| structured_debias | 0.163853268157 | 0.298978069968 | 229 / 0 |
| lcb_partial | 0.163853268157 | 0.298978069968 | 229 / 0 |
| lcb_fixed_full | 0.163853268157 | 0.298978069968 | 229 / 0 |

以上表格由锁定 evaluator JSON 自动生成；完整 horizontal/vertical、配对指标和coverage见 `full_walk1/aggregate/fde_aggregate.csv`。四个候选方法相对suppress的变化均为0，因为没有候选，不能作为补偿有效性或不同策略等价性证据。raw-frame误差因 frame/point provenance 未闭合仍为 `UNAVAILABLE_FRAME_OR_POINT_PROVENANCE`；bias truth、独立测量参考未提供，不用post-fit residual替代。

## 工程与生产门

- 最终构建exit0；受影响6个CTest通过，最终完整CTest **29/29**、exit0、124.87s。覆盖reference失败/身份错误、空集合与全suppress区别、cache版本/发布/replay；projector/Schur对照、异方差、完整nuisance、真实CombinedImuFactor非对角协方差、高leverage、零冗余、近退化、非有限、严格阈值等号/符号和damping独立性。
- Walk1起始后 `[8,11]s` smoke：5方法和1 producer全部COMPLETE/exit0；55/55 planned已测试，QR一次、rank180，0 fault/0 segment，`SUCCESS_EMPTY`。
- full Walk1串行：5方法和1 producer全部COMPLETE/exit0；1074/1074已测试，QR一次、rank3435，0 fault/0 segment，`SUCCESS_EMPTY`。
- 两个真实run均Stage2 optimizer=0；四个final optimizer=0，无fallback。实际reference/Stage2/四final graph与Values SHA相同，trajectory字节相同；去除方法上下文inference_id后，bias、residual和covariance数值/行完全相同，协方差AVAILABLE。`empty_reuse_audit.json`实测通过。Stage2 trace仅表头；final CSV保留明确 `SUCCESS_EMPTY`/`NOT_RUN` 状态行，所有迭代数值和停止判据为空，不是假造优化trace。
- 六输入×五方法30项配置加载/prepare均exit0且optimizer_calls=0，详见`prepare/ledger.json`。这只验证准备，不代表其它输入估计通过。

## 可复现证据和实际命令

隔离根目录 `R=/home/mint/ws_fusion_uwb/res/recovery_fde_v2_20260911_01`；所有运行采用新目录。每个batch cell的实际runner命令/exit code在`runs/<id>/command.json`，batch登记依赖和状态；完整源差异、二进制/冻结材料hash见`release_snapshot.json`和`release.patch`。以下命令均已运行，exit0（构建工作目录为workspace/build目录，工具工作目录为repo）：

```bash
catkin build uwb_imu_fgo --no-status --summarize --jobs 2
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target uwb_imu_fgo_paper_runner -j2
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target test_nlos_fde -j2
# cwd=/home/mint/ws_fusion_uwb/build/uwb_imu_fgo
ctest --output-on-failure
# cwd=repo; BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner
bash "$R/legacy_03_no_gt.sh"
python3 "$R/evaluate_localization_gate.py"
python3 tools/paper/run_experiments.py --manifest config/paper/ie0911/smoke_batch.yaml --runner "$BIN" --output-root "$R/smoke"
python3 tools/paper/run_experiments.py --manifest "$R/full_walk1.yaml" --runner "$BIN" --output-root "$R/full_walk1"
python3 tools/paper/evaluate_runs.py --batch-manifest "$R/full_walk1/batch_manifest.json" --evaluation-manifest config/paper/ie0911/step2_evaluation.yaml --output "$R/full_walk1/evaluation.json"
python3 tools/paper/ie0911_fde_aggregate.py --batch "$R/full_walk1/batch_manifest.json" --evaluation "$R/full_walk1/evaluation.json" --output "$R/full_walk1/aggregate"
python3 "$R/prepare_inputs.py"
python3 "$R/audit_empty_runs.py"
```

日志分别为`engineering/build.log`、`engineering/build_final.log`、`engineering/build_imu_fde_test.log`、`engineering/ctest_release.log`、`legacy_03_no_gt_wrapper.log`、`localization_gate_final.log`、`smoke.log`、`full_walk1.log`、`full_walk1_evaluation.log`、`full_walk1_aggregate.log`、`prepare.log`、`empty_reuse_audit_02.log`。成功定位对照的实际命令保存在`localization_fixed/runs/*/command.json`。

失败保留：legacy最初启动恰逢链接中途，exit127/共享库file too short，见`legacy/`与旧wrapper；warm-only Cauchy最大迭代失败，见`localization/`和`localization_diagnostic/`，未调参数；链接库robust反例测试exit1（`engineering/linked_robust_before.log`）；中间编译scope错误exit2（`build_fde_v2.log`）；SparseQR未排序R索引导致工程测试abort134（`empty_targeted_01.log`），改为既有triplet装配后通过；测试使用GTSAM equals(tol=0)严格比较导致的fixture断言已改为标量精确比较；辅助probe缺TBB链接/非native ABI失败不作为通过证据。独立empty审计首版误把显式状态行当迭代行而exit1（`empty_reuse_audit.log`），保留首版脚本，改为核验空数值与SUCCESS_EMPTY/NOT_RUN后exit0，无生产结果改写。

## 限制与 NOT_RUN

六输入精度矩阵、本轮其它输入full估计、global test、largest-residual isolation、solution separation、正式held-out/RQ/U14、独立bias truth/标定覆盖率检验均 **NOT_RUN**；非空真实候选未发生，不制造候选或按结果调p/temporal/kappa/阈值/区间。局部Gaussian检验不消除数据依赖初始化prior和标定缺口；单观测1%不等于整段1%。local sigma不是校准置信保证或最终bias后验。该轮证明定位恢复、标准化实现和空候选工程链，不升级C1–C3或恢复收益claim。

最终文档检查：工作论文`paper/main.tex`在隔离`paper_build/`连续两遍pdflatex exit0（本机article fallback，仅编译验证，非投稿版式验收）；新结果/协议本地链接及`git diff --check`通过。冻结结构与roadmap SHA-256均保持STATUS登记值。最终命令账本为`commands_final.json`。
