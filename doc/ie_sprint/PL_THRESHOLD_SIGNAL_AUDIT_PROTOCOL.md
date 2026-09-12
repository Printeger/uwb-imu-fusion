# PL group threshold / per-anchor persistent-signal diagnostic protocol

状态：`DONE / GROUP_STATISTIC_TASK_MISMATCH_PERSISTENT_PER_ANCHOR_SIGNAL_PRESENT`。

授权来源：用户于 2026-09-12 给定的 PL threshold/signal root-cause 诊断任务。IE 诊断基线为
`f2ee3f0d3aee47f8a1da2eaf2792295fb6bc44ee`，实际位于
`feature/uwb-imu-fusion-ie-postprocessing`；只读 PL source branch
`origin/feature/realtime-uwb-imu-pl` 位于锁定提交
`ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`。两条 Git 历史不得混合或强行切换。

## 1. Scope and invariants

- 本轮只诊断 group chi-square threshold 与逐 anchor persistent conditional innovation signal。
- 复用现有 locked Walk1 clean/injected scenario cache、初始化、keyframe plan、target link
  `27956:20276`、闭区间 `[1664959678.3077347,1664959686.3077347]`、`+0.5 m`、30 个
  affected observations，以及 f2ee3f0d 的 causal shadow replay。不得重新生成场景。
- 不修改 PL probability/current group detector、production config/provider/cache、injection、anchor、
  duration、temporal support、IE/recovery backend、Stage2、Rc、LCB、optimizer 或 evaluator science。
- 不实现 CUSUM、sequential GLR、新 FDE、ARAIM、isolation v6 或任何 support generator。
- 输出根固定为
  `/home/mint/ws_fusion_uwb/res/pl_threshold_signal_audit_20260912_01`，拒绝覆盖。

## 2. Replay reproduction and truth boundary

先用同一 clean/injected configs 各运行一个不接收 truth 参数的独立 shadow replay 进程。进程中隐藏
truth、GT/oracle 和通用 data roots，保留 file-open trace。必须复现：clean 224 groups/0 alarms；
injected 的 30 个 affected groups 为 0 alarms，最大 `T` 在 binary64 合理容差内等于
`4.860523050243609`。不满足则唯一裁决为 `PRECHECK_INVALID:<reason>` 并停止解释。

所有 `nu,S,z` 与 conditional quantities 在 truth-blind 进程中先生成并封存 SHA-256；封存后独立
evaluator 才读取 injection truth，按 `(keyframe/group,anchor,obs_id)` 配对 clean/injected。truth 只用于
选择并标注 target 30 observations、target anchor 和已知 `+0.5 m`，不得参与 statistic construction。

## 3. Source-neutral diagnostics

锁定 PL sign convention 为 `nu=z-h`、`H=d(h-z)/dx`；因此正 excess range 对应正 `nu`。对每条当前
group observation 导出 `nu_m`、`R_mm`、`H_m P H_m^T`、`S_mm` 与
`marginal_z=nu_m/sqrt(S_mm)`。使用同一完整 physical innovation covariance
`S=HPH^T+R`，对每个 row 以稳定 LDLT solve 计算

```text
nu_cond = nu_m - S_m,-m solve(S_-m,-m, nu_-m)
S_cond  = S_mm - S_m,-m solve(S_-m,-m, S_-m,m)
z_cond  = nu_cond / sqrt(S_cond).
```

禁止显式 inverse；维度、有限性、对称、SPD/positive conditional variance 任一失败都保存 invalid reason。
另保存 `nu_m*(S^-1 nu)_m` 的 additive group quadratic contribution（各 row 求和为 group `T`）及
`nu_cond^2/S_cond = T_full-T_without_m` 的 conditional increment，二者只作分解诊断。

解析测试必须覆盖 diagonal SPD、correlated SPD、near-singular-but-valid、2-anchor 与 5-anchor，并逐项
对照手算/独立 stable solve；不通过不得解释真实数据。

## 4. Offline evaluation

对每个实际 group 的固定 `T,dof`，只在独立 evaluator 离线计算
`P_FA={1e-5,1e-4,1e-3,1e-2,0.05,0.10}` 的 chi-square threshold。每档报告按实际 DoF 的 threshold、
clean 224 groups alarm count/fraction，以及 30 affected groups alarm count/recall。任何 sweep threshold
不得写回 config。最大 affected `T` 另以其实际 DoF 报 upper-tail probability。

对 target 30 observations 配对 clean/injected，并对同组 healthy anchors报告 `nu`、marginal z、conditional
z 及差值。分解 raw amplitude、measurement/prior variance fraction、conditional/marginal variance ratio、
target/healthy quadratic contributions。target 与 healthy anchor 分别报告 conditional z 的 N、mean、
median、sample std、P10/P90、正 excess sign fraction、max absolute value及
`Z_sum=sum(z)/sqrt(N)`；逐 epoch 保存 cumulative signed sum。正号在读取 truth 前已由 PL source convention
固定，不按结果选择。

## 5. Single verdict and stop

为避免看结果后移动“合理/明显”的含义，本 diagnostic 预先采用以下保守操作化规则：A 要求至少一个
`P_FA<=0.05` 档同时满足 clean empirical alarm fraction `<=0.05` 且 affected recall `>=0.5`。若 A
不成立，B 要求 target 的 paired `delta conditional z` 满足 `Z_delta>=3`、injected target 正号比例
`>=0.70`，且 `Z_delta` 至少为 `2*max(1,max(0,max_anchor healthy Z_delta))`；否则为 C。这里
`Z_delta=sum(z_injected-z_clean)/sqrt(N)`，healthy 按同 anchor 配对计算。这些界只用于本轮 A/B/C
root-cause 分类，不是 detector threshold、性能保证或后续算法参数。

Correctness note：第一次 evaluator transcription 曾误用 `max |healthy Z_delta|`，会把与已锁定 positive
excess 相反的负向 shift 当作竞争 NLOS evidence，违反本协议第 4 节的单侧符号定义。该次输出保留在
`attempts/absolute_healthy_sign_error_attempt1/`；最终 evaluator 只把 healthy 正向 evidence 与 target
正向 evidence 比较。sealed production-style statistics 未重跑或改变。

Closeout note：最终 evaluator 增补了 truth-blind status 与 file-open trace 的显式 gate；增补时一次手写
truth sidecar 路径错误在读 truth 前 exit 1，日志保留于 `attempts/missing_trace_gate_attempt2/`，随后只修正
命令路径并以同一 sealed statistics 重跑。最终 forbidden open count=0、evaluator exit 0。

唯一主裁决只能是：

- `GROUP_THRESHOLD_MISCONFIGURATION_DOMINANT`：合理 single-epoch operating point 同时保持可接受 clean
  alarm 且明显召回 injected；
- `GROUP_STATISTIC_TASK_MISMATCH_PERSISTENT_PER_ANCHOR_SIGNAL_PRESENT`：合理 sweep 仍弱，但 target
  conditional sequence 有持续正向、且明显区别于 clean/healthy；
- `CONDITIONAL_INNOVATION_SIGNAL_NOT_SEPARABLE`：target 与 clean/healthy 无明确 separation；
- `PRECHECK_INVALID:<reason>`：replay、identity、pairing或数值前置失败。

无论结果为何，本轮结束即停止。A 只提出使用 independent clean/calibration data 的 calibration protocol；
B 只允许建议另立任务评估 one-sided sequential CUSUM/GLR；C 停止 innovation-based automatic detector
开发并收缩 scope。本轮不得提前实现这些下一步。

实际结果：六个 sweep 档在 clean 为 0/224、affected 为 0/30；最大 T=4.86052305024、DoF=5 对应
upper-tail probability 0.4331383777。target injected conditional z mean=1.09263378、30/30 正向、
Z_sum=5.98460168，paired Z_delta=6.62197631；healthy 最大正向 paired Z_delta=0。因此唯一裁决为 B。
完整证据见 [`PL_THRESHOLD_SIGNAL_AUDIT.md`](../ie_0911/PL_THRESHOLD_SIGNAL_AUDIT.md)。
