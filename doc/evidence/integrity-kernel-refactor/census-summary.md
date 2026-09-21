# 普查可读摘要（census-summary）

配套机器可读文件：`census.json`（由 `tools/analyze_frames.py` 从真实 replay 二进制与诊断 CSV 生成；
原始产物 hash 见 `hashes.txt`）。本摘要所有数字均为本轮实测/独立复算，未使用推算值。

## 1. 冻结窗口（H / z / C）实测

| 帧 | m | n | rank(H) | ν=m−rank | cond(H) | parity 统计量 | χ² 阈值(p_fa=1e-6) | 判定 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| A_nominal ep30（普通帧） | 253 | 165 | 165 | 88 | 3.28e5 | 0.0 | 166.0 | pass |
| C_uwb_fde ep25（报警帧） | 253 | 165 | 165 | 88 | 2.92e5 | 419.3 | 166.0 | **alarm** |
| C_uwb_fde ep26（inf 帧） | 252 | 165 | 165 | 87 | 3.00e5 | 0.0 | 164.6 | pass |
| G_cont_rejection ep10（缺测/拒批帧） | 130 | 90 | 90 | 40 | 1.05e5 | 1.30e4 | 97.7 | **alarm** |
| H_mature_union ep201（边缘化帧） | 253 | 165 | 165 | 88 | 9.63e5 | 0.0 | 166.0 | pass |

- 记录值（来自 C++ 的 SVD）与独立 numpy SVD 的 rank/ν/cond 逐一核对一致；`base_information` 与独立 `HᵀH` 的最大相对偏差 ≤ 2e-26（见 `census.json.base_information_rebuild_rel_err`）。
- 流是确定性**均值流**（`measurement_realization: covariance_mean`），正常帧 parity 精确为 0。
- 稀疏性：A 类满窗口 `nnz(H)=3653/25333=8.8%`，`nnz(HᵀH)=6975/27225=25.6%`；按当前变量顺序做符号消元的 fill-in=0（满窗口），但 G 帧（有拒批缺口）fill-in=11。**当前窗口的链式结构在满窗口时不产生 fill-in，但不等于其他 ordering 亦然。**

## 2. 块结构（A_nominal ep30，共 23 块）

| 类别 | 块数 | 行/块 | 说明 |
|---|---:|---:|---|
| BoundaryPrior（历史压缩） | 1 | 15 | 来自窗口外因子的 partial-QR + 稠密特征分解；秩=15（本例）。**无任何故障方向/检测信息保留** |
| UwbBatch 显式历史 | 10 | 8 | epoch 20..29（各 anchor 一行，行序=配置 anchors 顺序） |
| UwbBatch pending | 1 | 8 | epoch 30 |
| CombinedImu 显式历史 | 9 | 15 | epoch 21..29 |
| CombinedImu pending | 1 | 15 | epoch 30 |
| BoundaryInput 槽位（未进显式块） | 41 | — | 由 `factor_inventory` 记录，压缩进 BoundaryPrior |

- 白化一致性：全部块满足 `‖W·J_raw − J_whitened‖/max|J_whitened| ≤ 1.7e-16`（**无重复白化**）。
- C（`protected_state_map`）仅在最后状态块的平移切坐标上非零：列 153..155（=`[0₃ₓ₃ | R]`），受保护量=body 原点世界位置；不包含 tag lever arm。
- 每块 `fault_units`、`whitening_model_id`、`window_column_indices` 与槽位均逐块记录在 `census.json`。

## 3. 假设构造（代码构造路径 + 实测）

构造公式（由 `hypothesis_generator.cpp` 逐行导出，并与实测核对）：

```
窗口覆盖 [proposed − min(epochs, proposed), proposed]
窗口内 UWB 显式组数 O = 覆盖内的已提交 UWB 组数（饱和时 = epochs+1；含当次 pending 组）
occurrences K = 窗口内显式 CombinedImu 组数（饱和时 = epochs；被拒批历元会减少）
单故障 S = Na · O · (2 + ramp) + 6·K                    （ramp∈{0,1}）
双故障 D = S + (Na·O·(2+ramp)) · (3K) ×2                （uwb×accel 与 uwb×gyro 各一份）
```

| 运行 | 配置 | 实测 (single_uwb / accel / gyro / uwb_accel / uwb_gyro) | 合计 |
|---|---|---|---|
| A_nominal ep30 | K=10, O=11, ramp off | 176 / 30 / 30 / 0 / 0 | **236** |
| F_ramp ep30 | K=10, O=11, ramp on | 264 / 30 / 30 / 0 / 0 | **324** |
| G ep45（多次拒批） | K=9, O=9, ramp off | 144 / 27 / 27 / 0 / 0 | 198 |
| H ep201（成熟） | K=10, O=11, ramp off | 176 / 30 / 30 / 0 / 0 | 236 |
| SD census（double_faults=true, proposed=6） | K=6, O=6, ramp off | 96 / 18 / 18 / **1728** / **1728** | **3,588** |

**旧报告 S=624 / SD=61,104 的来源已核实并可精确复现**：Na=8、`epochs=20`（21 个 onset）、ramp 开
→ UWB=8·21·3=504，加 6·20=120 → S=624；SD=624+504·60·2=61,104。对应旧
`realtime_performance_benchmark` 口径（K=20、ramp 开、单/双故障），**不是当前研究配置**
（当前 K=10、ramp 关 → 236/帧）。本结论由公式与两次独立运行（epochs=20 census、SD census）交叉验证。

区间/节点语义：`epochs` 是**区间数**；窗口含 `epochs+1` 个 15 维节点。实测 epochs=10→H 253×165，
epochs=20→H 483×315（同一 attempt 30），对应的假设数 236 vs 456。

## 4. 每假设记录（诊断 CSV，真实导出）

`diagnostic_attempts.csv`（v10）每帧包含 `hypothesis_count` 与五类计数、risk 分解、
数值工作计数（base_svd/base_llt/…/fault_gram_ldlt）、cache 命中、oracle 标记等；
`hypotheses.csv` 每假设包含 `parameter_dimension、fault_rank、sigma_min/max、condition_number、
monitorable、plausible、conditioned_statistic、log_evidence、onset_epoch/time、prior/hmi 分配`。

- **已知缺陷（本轮发现并如实记录）**：`hypotheses.csv` 的 `slope_x/y/z` 恒为 `inf`。
  根因：`integrity_monitor.cpp:1793` 从 `evidence[i].monitorability.protected_slopes` 取值，
  而批量证据路径只把 slope 写入 `FrozenHypothesisNumerics.pl_entries`（及 PL 路径的
  `hypothesis.monitorability`），`evidence` 副本保持结构体默认 `+inf`。
  **不影响 PL 计算**，但审计表该列不可用；修复建议 + 回归测试列入下一批（A3/A4）。
- 时间支撑：UWB 模式以真实 `onset_time`（秒）与测量时间戳构造 ramp；IMU 模式为单历元区间常值。

## 5. 重复计入检查（本轮可得证据）

- `finalizeIntegrityWindow` 以 group_id 去重（`no_duplicate_rows`），实测所有保留帧为 true；
- 槽位账目 `explicit ∪ boundary = 全部 slot` 且不交（`slot_accounting` 已核对）；
- `base_information == HᵀH`（≤2e-26）；
- 更细的“测量级 lineage 重复/漏记”需要 factor_ledger × window 的行级对齐检查（roadmap HIS-03），
  属 A3/A4 范围；本轮未宣称完成。

## 6. 有效配置与 hash

- 基准配置：`config/realtime_uwb_imu_pl_research.yaml` sha256 `9285fa6e…11331`
- 有效配置全量导出：`effective-config.yaml`（含优先级链与 `config_hash` 复核：FNV 变体= `b2863fca7f443d3d`，与 `run_manifest.json` 一致）
- census 专用变体（仅证据，不改仓库默认）：`configs/research-epochs20-census.yaml`、`configs/research-double-faults-census.yaml`（hash 见 `raw/config_hashes.txt`）
