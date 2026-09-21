# §5.9 验证过的联合界、风险账本与惰性 FDE（工作包 B3 / B4）

基线与代码提交：`82408e8`（P4）→ `e40e2ef`（P5，代码）。本轮**零阈值/零合同改动**：
0.25 步长 gate、128 动作上限、p_fa/p_md、alert limits、风险预算总额与 FDE 选择合同均未动；
`formal_eligible` 保持 `false`。

## 1. §5.9 逐轴联合界（B3-a）

**命题**：对假设 `h` 与受保护轴 `d`（本仓库 3 轴：x/y/z）

```
L_{h,d} = s_{h,d} · sqrt(Lambda_h) + k_{h,d} · sigma_d
k_{h,d} = Phi^-1(1 - alpha_{h,d}/2)
Lambda_h 解 F_{chi2_nu(Lambda_h)}(tau) <= beta_h        （保守侧端点）
```

则 `P(exists d: |e_d| > L_{h,d} , accept | h) <= max(sum_d alpha_{h,d}, beta_h)`。

**本轮核对与修正**：P4 形态与 §5.9 *形式*一致，但每轴尾概率取成了整假设尾概率
`alpha_h = allocation_h / pi_h`，而账本按 `pi_h · alpha_h` 计费——即每轴花了 3 倍于已付费的
风险（账实不符的**放宽**方向）。本轮把每轴尾概率改为**等分** `alpha_{h,d} = alpha_h / 3`
（`axisTailSplit()`），于是

* 计费 `charge_h = pi_h · max(sum_d alpha_{h,d}, beta_h) = max(allocation_h, pi_h·beta_h)`
  与账本一致（RSK-01 断言 `pi_h · charge_h == max(allocation_h, pi_h·beta_h)`）；
* `k` 严格变大（RSK-01 断言 `k(alpha_h/3) > k(alpha_h)`），PL 只会**更保守**；
* 三个 PL 路径（streaming / shared / frozen-all-in）共用同一 `faultAxisBounds()`，
  无效输入（beta=0、beta>=1、零 dof、非法先验）**拒绝**并给出原因，不再截断成近似值。

**差异清单（vs P5 基线运行 `/tmp/uwb_imu_pl_b5_20260921/baseline`）**：

| 场景 | PL 变化数 | min Δ | max Δ | 离散差异 |
|---|---|---|---|---|
| A_nominal | 90 | **+0.000e+00** | +1.122e-01 | **0** |
| C_uwb_fde | 75 | +0.000e+00 | +9.814e-02 | **0** |
| D_imu_bridge | 72 | +0.000e+00 | +1.125e-01 | **0** |
| E_union | 72 | +0.000e+00 | +1.125e-01 | **0** |
| F_ramp_unmonitorable | 90 | +0.000e+00 | +1.093e-01 | **0** |
| G_continuous_rejection | 12 | +0.000e+00 | +1.556e-01 | **0** |
| H_mature_union | 669 | +0.000e+00 | +2.873e-01 | **0** |

所有 PL 变化为**单侧**（`min Δ = 0`，即不存在任何减小）——逐条满足"更保守"要求；
`availability / label / batch_committed / selected_action_id / fde_status /
risk_budget_valid / conditional_passed / monitorable / plausible` 在 7 场景全部**逐帧 0 差异**。

**数值实现（B3-a 第 3 条）**：小尾分位数走 **complement**（`Phi^-1(1−tiny)` 不再消减到 1.0/±inf，
1e-300 尾返有限值）；非中心性边界二分带收敛判据（宽度停止 + 残差 1e-12·max(1,beta) + 单调性回退）
并显式返回**保守侧端点**（`F(Lambda) <= beta`、`residual <= 0`）；缓存键含
`detector_id / dof / contract_version / threshold / p_md / envelope_fingerprint`
（禁止按模式名缓存），版本或包络指纹变化必 miss（RSK-01 边界用例断言）。

## 2. 风险账本（B3-a 第 2 条）

`buildRiskLedger()` 产出 10 项，每项带 `value / status / source / note`，并随发布输出导出
（诊断 v14 列 `risk_ledger_*`，attempt 行 CSV-quoted 字符串）：

| 项 | 计费 | 状态 | 来源 |
|---|---|---|---|
| `nominal` | ✅ | VALIDATED | config `nominal_axis_tail × 3 轴` |
| `p_nm` | ✅ | VALIDATED | config `p_nm` |
| `hypotheses` | ✅ | VALIDATED | `Σ_h pi_h·alpha_h`（分配策略 `prior_weighted_outcome_conditioned`） |
| `hypotheses_miss_channel` | ❌ | **ASSUMED_UNVALIDATED** | §5.9 的 `Σ_h pi_h·max(0, beta_h − alpha_h)` 通道 |
| `bridge` / `history` / `model` | ❌ | **ASSUMED_UNVALIDATED** | config `p_*_escape`（当前配置为 0，按"未验证假设"入账，不当作已证明的零） |
| `omitted` | ❌ | **NOT_IMPLEMENTED** | manifest `omitted_event_set`（列表已导出，质量未量化） |
| `envelope` | ❌ | **NOT_IMPLEMENTED** | 覆盖证书（在线未启用 ⇒ 显式 0 + 资格统计） |
| `selection` | ❌ | **NOT_IMPLEMENTED** | C3 预留槽位 |

`charged_total` 只累加 VALIDATED 项（= P4 的现有判据，故可用性逐帧不变），`declared_total`
另外把未计费通道显式相加，因此**不存在隐藏零项**；`all_terms_validated=false` 与
`formal_eligible=false` 一起阻断面元资格而不把研究态打成不可用。

**已知缺口（需与预算重标定一起做）**：完整的 §5.9 计费还需把 miss 通道入账。以当前配置
（`pi_h·beta_h ≈ 1e-7 × N_h`，N_h 为冻结 registry 假设数）加上名义 3e-5 会超过 `4e-5` 预算，
使多数帧不可用；本轮将差值**显式导出**（C 帧：charged 4.0e-5 vs declared 5.004e-5，缺口≈1.0e-5），
作为 B3 收口的后续输入（与预算重标定/C3 选择合同一并处理）。

## 3. 零空间决定（B3-b）

`classifyDetectionResponse()` 现在额外返回**每轴残差**与 §5.5 投影界
`||g V_r Sigma_r^-1||`，决定随之落到证据路径：

* `full rank`：行为与 P4 逐帧一致（本批 7 场景全部为 full-rank，35k+ 行）；
* `dangerous`（分类 3）：`monitored=false / plausible=false / valid=false`，
  reason 指明**受保护轴的残差与模式 id**（`dangerous detection nullspace: ... axis residuals x=…,y=…,z=…; modes 1`），
  不再以"字典维数"整体拒绝；
* `indistinguishable`（分类 4）：不可用，reason 要求参考回退；
* `harmless`（分类 2）：允许有限界，`bound_from_projected_path=true`，斜率取投影路径值
  （Γ 仅审计；PL 门接受该界）。

测试：`B3ZeroSpace.DangerousNullspaceIsUnavailableWithDirectionLevelReason`（构造
`A = H·e_0` → 检测空间不可见而受保护状态被推动）、
`B3ZeroSpace.HarmlessNullspaceKeepsAFiniteProjectedBound`（合成窗口的两状态受限，
用零响应退化表示；真实无害零空间需要受保护映射的零空间方向，本批 7 场景不存在）。

## 4. 覆盖证书进入发布路径（B3-b）

每帧的精确遍历证书随发布输出一并落盘（诊断 v13→**v14**，只加列不删列）：
`coverage_status`（COMPLETE/INCOMPLETE）、`coverage_exact_leaves`、`coverage_enveloped_leaves`、
`coverage_uncovered_leaves`、`coverage_envelope_count`、`coverage_accepted_envelope_count`、
`coverage_proof_count`、`coverage_envelope_online`；叶级标签仍在 `hypotheses.csv`
（`coverage_label`/`coverage_envelope_id`）。证书不新增旁路：它附在原来的 attempt 输出上，
历史有效性 gate、事务与窗口身份绑定不变。包络在线启用仍默认关闭（`coverage_envelope_online=0`），
启用前义务记于 proof-obligations 第 15 项。

## 5. 惰性 FDE（B4）

* `HypothesisGeneratorConfig::lazy_action_entities`（默认 true）：`generate()` 只产出模式描述符、
  假设（含 `hmi_allocation`）与 KEEP_ALL 参考解；单模式动作实体（含替换组与桥接块）由
  `ensureActionEntities()` **按需、幂等**构造。
* 需要证据的路径显式触发：报警帧/强制屏障帧在调用 `actionsForPlausibleSet()` 时构造；
  健康管理路径（`pending_health.observeEvidence`）**未删除**，计数为证。
* 计数（A_nominal 30 帧，7 场景同型）：`action_entities_constructed=0`、
  `bridge_blocks_built=0`、`candidate_graph_built=0`、`action_entities_deferred=5930`、
  `evidence_calls_fault_path=30`（PL 灵敏度完整）、`evidence_calls_health_path=420`。
* 报警帧：动作 ID 与 eager 路径逐一相同（KEEP_ALL=1，模式动作按模式顺序），
  7 场景的 FDE 状态/选择/提交/PL 可用性**逐帧 0 差异**。

## 6. 明确未做（NOT_RUN，按指令保留）

| 项 | 原因 |
|---|---|
| 完整 §5.9 计费（miss 通道入账） | 当前预算下不闭合（见 §2 缺口）；需预算重标定，属 B3 收口后续 |
| G 帧 gate 拒绝的**逐轴/物理量归因**（rotation/position/velocity/bias 分解入 reason） | 未在本批补齐：0.25 gate 与阈值已验证不变（离散 0 差异），现有归因仅到 base/selected 步长范数与块级 stage 记录 |
| C1 历史摘要、C2 分离残差布局/§7.4 双通道 PL、C3 选择合同与保证组预算、C4 原子发布历史改造 | 本轮范围外 |
| 性能收口（D） | 本轮只报计数与观测 |
| 单侧支配容差的 .clang-format/风格收口 | 见 runbook §9（Stage 0 触发源取证与处置） |

## 7. 复现

```bash
cd /home/mint/ws_fusion_uwb && source /opt/ros/noetic/setup.bash
catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl     # 300 tests / 0 failures
devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_integrity_v2 \
    --gtest_filter='B3Risk.*:B3ZeroSpace.*:B4LazyFde.*'
O=/tmp/uwb_imu_pl_b5_20260921/b3b4    # 7 场景（A/C/D/E/F=30, G=45, H=226）
python3 tools/integrity/run_validation.py --all            # 22 PASS / 0 FAIL / 20 NOT_RUN
python3 doc/evidence/integrity-kernel-refactor/tools/context_oracle.py   # UWB_IMU_PL_ORACLE_RUNS=$O
```
