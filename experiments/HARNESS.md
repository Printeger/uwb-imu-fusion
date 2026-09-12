# ISAS Walk1 clean harness（0912）

本轮用户授权替代五类数据验证；只运行一次 full ISAS Walk1 clean 四方法 smoke。
“不要接 SFUISE”按不扩展 SFUISE 输入解释；本地 ISAS-Walk1 来自 data/SFUISE。
当前 branch 为 feature/uwb-imu-fusion-ie-postprocessing，不另建/切换分支。

固定方法：M0=现有 paper all_range（Base FGO-LS），M1=robust_cauchy（既有 scale=2.3849），
M3=suppress_all，M4=lcb_fixed_full。M3/M4 共享一次新 PL_BIDIRECTIONAL_CUSUM_V1
producer/Stage2 cache；AUX_STAGE2_PRODUCER 单列记账，不是第五种比较方法。
无新 estimator，没有更改 detector、recovery 或 solver 参数。

复用 config/paper/ie0911/sfuise_walk1_pl_bidirectional_clean.yaml，包含原 CUSUM 全部参数、
原 recovery/solver 数值和完整 clean 观测。原 clean measurement-only cache 与 payload SHA-256
在启动前核对，不含注入，不重新生成；依赖本工作空间既有 res/nlos_injection_20260911_01/inputs。
本轮只支持这个锁定输入，不接受外部 dataset/method/parameter 参数。
M0/M1 通过既有 scheduler 禁用 NLOS；final thresholds 显式原值 0.1/0.1/1.0，
避免 scheduler 缺省填零。cache producer 是真实独立运行；不复用旧 Stage2 或轨迹。

## 冻结 evaluator 与评价协议

全部四方法调用原 tools/paper/evaluate_runs.py 的 trajectory_metrics，采用同一 ground_truth.tum
（SHA-256 2bd90465cc6f9445e94c75087a293c36cfc25a4ac1165654a9765431c8816c33）。
评估先按估计 sensor 时间裁至闭区间 [1664959676.9893188,1664959736.082964] 秒，
该区间来自旧 full Walk1 计划，不看本次结果选择。nearest_within_tolerance=0.02 s，
scale=1 SE(3) 每轨迹独立 alignment；不允许 per-method 不同窗口或关联参数。
逐方法导出 matched GT timestamps/hash，四者完全相同才给比较通过；缺方法或不同集合则明确失败，
不通过事后取交集来隐藏缺失。原 evaluator 的 RPE horizon=1 s：选择首个 >=t+1 s 的匹配样本，
要求偏差 <=0.02 s；报告 translation RPE RMSE（m），不是新实现或严格插值到 1 s 的指标。

GT reference 是 vive/tracker_1 原点，tracker-to-IMU 外参未闭合；所有数字明确为既有
DEVELOPMENT_TRACKER_PROXY_NOT_CALIBRATED_BODY_ERROR。raw-frame ATE 不可用，
RPE 也不能冒充已标定同刚体误差。这里沿用原 evaluator 代理指标做工程 smoke，不宣称论文准入。
父进程只在 worker 退出后加载 GT pose；worker 只收到 measurement cache 与 stripped config。
后续 range-evaluator 任务已强制 estimator bwrap mount allowlist，并完成真实不可读 probe 与隔离 smoke；
新证据见 [RANGE_EVALUATOR.md](RANGE_EVALUATOR.md)。未使用 strace，不追溯升级历史非隔离 run。

## 执行与输出

```bash
python3 experiments/scripts/run_walk1_smoke.py
```

需要当前已编译 uwb_imu_fgo_paper_runner、Python numpy/yaml/jsonschema 及上述锁定 res 输入/GT。
不自动构建、下载或安装依赖。每次新建 experiments/results/walk1-clean-UTC-UUID/，不使用 latest。
run_id 由现有 scheduler UUID 创建；没有启动的计划方法也有独立 not-run UUID 和明确失败行。

每个 batch 保存 lock.json、batch.yaml、command.json、scheduler.log、原 batch/caches/runs，
以及统一 runs.csv、trajectory_metrics.csv、summary.json。
新的 trajectory evaluation 中间 TUM/逐样本关联存于工作空间 evaluator_private/icra/trajectory，
不放在 estimator 挂载目录；每次 backend 的 *.isolation.json 记录实际 mount allowlist。runs.csv 包括四方法和 AUX producer；
metrics 只含四种方法。每次 backend invocation 保存开始/结束 UTC、退出码、wall time 和完整配置对象。
complete_config_hash 是 canonical JSON SHA-256，覆盖实际 YAML、method、execution type、
operating point、anchor subset、环境覆盖和 parent cache hash；runner/source/evaluator hash 另存。
wall time 是每次 backend 调用耗时（final replay 不包含共享 producer 耗时）；AUX 单列保留费用。
每次结果只写本 batch 的统一 CSV，无并发共享 CSV 覆盖问题。

预先定义 smoke PASS：四方法有效输出和 evaluator ATE 可用、相同 matched GT 样本集合，
producer 成功；不以定位数值优于其他方法为门，不调任何参数。失败/fallback 原样保留。
没有有效 horizon pair 时 RPE 独立 UNAVAILABLE，不能用 0 补值。
大矩阵、其他数据集、未见 TEST、注入/nonempty recovery、非零校正收益、真实 fallback smoke 均 NOT_RUN。

本轮结果见 HARNESS_RESULT.md（运行后登记）。
