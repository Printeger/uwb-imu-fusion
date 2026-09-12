# ISAS Walk1 clean harness 结果

最终裁决：`SMOKE_PASS / DEVELOPMENT_ONLY`。四方法成功，producer 成功，0 fallback。
当前 git commit `439282c2790ab1414ca9b276dd61a18b5e1643a8`。
完整新产物位于 [walk1-clean-20260912T092140Z-2732b6b82486](results/walk1-clean-20260912T092140Z-2732b6b82486/summary.json)；
Git 可保存的 CSV/身份/失败摘要位于 [evidence](evidence/walk1_harness_20260912/)。
旧科学结果、核心算法、CUSUM/recovery/solver 参数没有改动。

## 统一指标（m）

| 方法 | ATE RMSE | ATE median | ATE P95 | 1 s translation RPE RMSE | GT samples |
|---|---:|---:|---:|---:|---:|
| M0 | 0.163853268 | 0.136166321 | 0.298978070 | 0.691923024 | 229 |
| M1 | 0.163847231 | 0.139577795 | 0.294238429 | 0.708137249 | 229 |
| M3 | 0.163853268 | 0.136166321 | 0.298978070 | 0.691923024 | 229 |
| M4 | 0.163853268 | 0.136166321 | 0.298978070 | 0.691923024 | 229 |

所有方法匹配同一 229 个 GT samples，matched GT timestamp hash
`fafcaed54cf9db4727a4d6fca6b7a1239a6fcc58cedbed11b38197fd2b65cf4c`。固定评价区间
`[1664959676.9893188,1664959736.082964]s`，nearest 0.02s，scale=1 SE(3) alignment。
GT 为 `vive/tracker_1` 原点，未有 tracker-to-IMU 外参；ATE 和 RPE 都只能作为现有 evaluator 的
开发代理指标，不是已标定同刚体定位精度。raw-frame ATE UNAVAILABLE。
RPE 复用原 evaluator 的约 1 s horizon（首个 >=1 s 且误差 <=0.02 s），无另写数学。

M3/M4 均返回 `NO_CANDIDATES`，共用同一新 cache，轨迹数值与 M0 一致。
不支持非空 recovery、校正收益、fallback 有效性、泛化或论文 test claim。

## 运行与所有失败

实际命令（仓库根）：

```bash
python3 experiments/scripts/run_walk1_smoke.py
```

| 批次 | harness exit | 结果/原因 |
|---|---:|---|
| 20260912T091952Z-fbbd262655fc | 1 | Python3.8 不支持 str.removeprefix；preflight 失败，estimator 全部 NOT_RUN。替换字符串处理。该次未记录单调 wall/结束时间，CSV 时间留空，wall=0 表示没有 backend 调用，不表示 preflight 零耗时 |
| 20260912T092015Z-c2200396dba8 | 1 | M0/M1/producer 成功；M3/M4 CACHE_SUCCESS_EMPTY_EVIDENCE_MISSING。harness 错把 PL cache 标 AUTO_DISCOVERY；改为已有 PL_BIDIRECTIONAL_CUSUM 调度 path，未改算法或伪造 evidence |
| 20260912T092140Z-2732b6b82486 | 0 | 四方法与 producer 成功，统一 evaluator 与 GT 集合核对通过；最终 scheduler exit0 |

每批原 CSV、config hash、命令/退出码、失败原因均保留在 evidence 和隔离结果目录。
后两次各有 5 次 backend 调用，无单 run 内重试；后一批是修复 harness 接线后的完整复验。
这不是调 detector/recovery 参数后反复择优。

`python3 experiments/scripts/test_walk1_harness.py` exit0，5/5 工程测试：状态/失败/fallback 分类、
配置身份敏感性、原 evaluator 已知刚体变换与 1s RPE、缺轨迹不可用、PL namespace/冻结数值。
日志：[engineering_tests.log](evidence/walk1_harness_20260912/engineering_tests.log)。

## 交付与限制

代码只新增 experiments/scripts/run_walk1_smoke.py 与 test_walk1_harness.py；
运行时包装已有 scheduler.runner_call 做身份/时间记录，复用原 scheduler/runner/evaluator。
[HARNESS.md](HARNESS.md) 定义运行协议；README、STATUS 与 claim ledger 同步更新。
每 batch 的 runs.csv 是统一运行表（含 AUX producer），trajectory_metrics.csv 只含四方法。
失败方法也占一行，缺指标留空并附状态；完整 configuration JSON 进入每次 invocation 证据。
输出隔离且 UUID 唯一；complete_config_hash 不用局部参数子集替代完整 invocation。

尚未实现/未执行：其他数据集入口、参数 CLI、大矩阵、干净环境自动安装/数据下载、
正式 VAL/TEST、独立标定 GT 外参、严格插值 1s RPE、注入/非空恢复与真实 fallback 测试。
依赖当前工作空间已编译 runner 与锁定 clean measurement cache/GT；不是单个仓库即可运行的自包含发布包。
本轮 build/核心 CTest/strace truth 访问审计 NOT_RUN；无核心代码修改或依赖升级，无提交/push。
用户上轮五类数据验证已暂停，仅完成初始清点，未升级模板为已验证 recording。
