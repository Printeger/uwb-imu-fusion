# 单/双故障假设选择实现与实测报告

日期：2026-09-17。结论：配置开关已贯通假设生成、证据、PL、FDE、缓存身份和诊断；主配置默认改为仅单故障。结果仍为 `DEVELOPMENT_ONLY / IMPLEMENTED_UNVERIFIED`，没有改变 formal-ready/Gate 状态。

## 实现

- `fault_models.single_faults_enabled`，默认 `true`。
- `fault_models.double_faults_enabled`，默认 `false`。
- `fault_models.max_cardinality: 2` 只表示实现支持上限。运行 manifest 的 `scope.max_fault_cardinality` 写实际启用阶数（S 为 1，D/SD 为 2）。
- 新增 `fde.max_exclusion_cardinality: 2`，独立保持原 FDE 排除来源数上限；切到单故障不会把联合排除动作错误挡掉。
- D 开启时继续由 `combinations.uwb_plus_accel`、`uwb_plus_gyro` 选子型；两者均关时报错。`two_uwb=true`、IMU+IMU 和 UWB+UWB 仍不支持/拒绝。
- 旧 YAML 缺三个新增字段时，在序列化和哈希 resolved config 之前补为 S-only 与 FDE=2，manifest 不再隐式表达旧默认。
- 模式始终完整构造，`single=false` 只禁止生成单故障假设，不删除构造双故障所需的 UWB/IMU 模式。
- 风险只在实际假设上分配。稳态 S/SD 的 hypothesis risk 总额均为 `9.9e-6`，等分分别为 `1.5865384615384613e-8` 与 `1.6201885310290652e-10`；总风险仍为 `4.0e-5`。
- `FrozenHypothesisNumerics` 增加故障策略指纹；策略不一致时 fail closed，不复用缓存。原窗口/模式/假设内容身份检查仍保留。
- diagnostics v10 记录五类实际假设数、总数、有效阶数、风险与数值工作量；manifest 记录两个开关、支持阶数、实际监控阶数及 FDE 排除上限。

四种配置：

```yaml
# S（默认）
single_faults_enabled: true
double_faults_enabled: false

# SD
single_faults_enabled: true
double_faults_enabled: true

# D-only（开发实验，不覆盖独立单故障）
single_faults_enabled: false
double_faults_enabled: true
```

`false/false` 明确报错。

## 基线与环境

- HEAD：`e6ded3069bbc4cd5cf98f14257e96f61066685b8`；本任务修改前当前工作区内容清单 SHA256：`269f5570e9fa8332a8a8afed23920e4511a6c81c048b7392573e2db12f8417ae`。
- 修改前冻结可执行/项目动态库 SHA256：`b5b7...e5bd` / `958a...0841`；最终为 `72af...c7f` / `f50a...2e1a`。两者均核对实际加载的隔离 `libuwb_imu_pl.so`。
- Release，GCC 9.4.0，Intel Core Ultra 5 125H，WSL2；固定 CPU 0-3；候选/假设 worker=4；Eigen/OMP/OpenBLAS/MKL 线程=1；seed=20260901；8 基站；20 epoch 窗口；默认 ramp；关闭逐假设 full audit。
- S 配置 SHA256 `3f1b...e12d`，SD `8a2b...f0d7`。两种模式使用同一最终二进制，仅两个开关不同。

## 正确性

- 98 个 C++ 测试、38 个 Python 工具测试全部通过。覆盖四组合/非法子型/旧 YAML、D-only 模式保留、风险审计、缓存策略失效、full-plausible FDE、修改观测集合候选数值路径、秩亏/临界数值和固定滞后边缘化前后回归。
- 满窗实测：S=`504 UWB + 60 accel + 60 gyro = 624`；D-only=`30,240 UWB+accel + 30,240 UWB+gyro = 60,480`；SD=`61,104`。顺序由生成器测试覆盖，运行逻辑未硬编码这些数。
- 修改前冻结 SD 与新 SD 跑相同 32 帧：状态、协方差相关输出、检测、PL、候选资格/选择、提交/拒绝、健康/事务离散结果一致；公共数值字段最大绝对误差和相对误差均为 0。仅缓存诊断 `context_bytes` 因新增 64-bit 策略指纹增加 8 字节。
- 正常主实验 S/SD 都是 KEEP_ALL，稳态每帧 128 个动作生成、1 个候选实际 kernel、1 次 PL；每帧 1 SVD、1 LLT、2 次状态求解、1 次 covariance RHS（787 列）、1 次 spectral RHS（787 列）。共享缓存均为 300 hit / 0 miss，通用低维回退为 0。
- 故障短序列有预期行为差异，不能当成纯性能收益：S 在 C 的故障帧成功 `SUCCESS_UWB_EXCLUSION`；S 在 D 的第 28 次输入成功 `SUCCESS_IMU_EXCLUSION_GENERIC_BRIDGE`。E 是联合故障、超出 S 覆盖，虽第 28 次输入选择 IMU exclusion，也不宣称完成联合故障隔离。SD 在这些 32 帧短序列没有成功排除，保持原 full-plausible 覆盖而拒绝或不可用；未放宽 FDE。

## 正常序列性能

每模式 3 轮交替 S1/SD1/S2/SD2/S3/SD3；每轮 140 帧，丢弃前 40 帧，合并 300 个稳态样本。分位数为 Hyndman-Fan type 7 线性插值，未删尾延迟；SD 第 2 轮宿主尾延迟明显（p50 1742.240 ms，另两轮 904.063/798.964 ms），仍完整计入。

| `core_total` | S ms | SD ms | SD/S 加速 | 降幅 |
|---|---:|---:|---:|---:|
| mean | 674.389 | 1269.080 | 1.882× | 46.860% |
| p50 | 628.667 | 919.333 | 1.462× | 31.617% |
| p95 | 1127.970 | 2886.207 | 2.559× | 60.919% |
| p99 | 1354.073 | 5177.268 | 3.823× | 73.846% |
| max | 1850.830 | 5528.400 | 2.987× | 66.521% |

完整 140 帧进程 wall time 均值 S/SD 为 89.287/154.110 s（1.726×，降 42.063%）。峰值 RSS 最大值为 231,068/292,360 KiB；按三轮均值，S 少 58.65 MiB（降 20.662%）。

主要阶段（各阶段独立统计，不能相加成整帧分位数）：

| 阶段 | S mean / p50 ms | SD mean / p50 ms |
|---|---:|---:|
| window preparation | 193.290 / 178.995 | 276.040 / 186.875 |
| model generation | 168.185 / 155.079 | 312.304 / 219.850 |
| hypothesis evidence | 198.924 / 179.381 | 329.439 / 247.119 |
| candidate evaluation / PL | 41.709 / 40.653 | 105.862 / 82.662 |
| FDE decision | 0.318 / 0.296 | 44.226 / 32.406 |

S 的新 p50 最大瓶颈依次为 hypothesis evidence 179.381 ms、window preparation 178.995 ms、model generation 155.079 ms；窗口内保护性 SVD p50 129.465 ms。40 ms 未达到：S p50 仍为目标的 15.717 倍，且 p99 仅为样本观察值，不是实时保证。

故障帧 attempt 25 的 S/SD `core_total`：C 2409.015/1266.203 ms，D 1097.574/1241.436 ms，E 479.232/798.612 ms。这里候选、PL 调用、拒绝与后续图状态不同，不用于宣称算法加速；逐帧明细保存在精简 `summary.json`/`performance.csv` 的来源运行中。

## 限制与剩余风险

- 本次只增加模型选择，不重新优化已有内核；正常 S 仍远超 40 ms。当前优先瓶颈是证据、窗口 SVD 和模式生成，不是 GPU/日志。
- 性能环境存在明显宿主尾延迟，报告未删样；需要硬实时结论时应在隔离裸机上重复。
- S-only 明确不覆盖联合故障，D-only 明确不覆盖独立单故障；不能包装成原 SD V2 范围证据。
- 短故障序列只提供开发级正例/拒绝路径，不改变 formal-ready/Gate 结论。

复现命令见 `reproduce.sh`，机器可读摘要见 `summary.json`，数据表见 `performance.csv`。
