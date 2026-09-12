# PL group threshold / per-anchor persistent-signal audit

日期：2026-09-12

IE 基线：`f2ee3f0d3aee47f8a1da2eaf2792295fb6bc44ee`

锁定 PL source：`ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`

协议：[`PL_THRESHOLD_SIGNAL_AUDIT_PROTOCOL.md`](../ie_sprint/PL_THRESHOLD_SIGNAL_AUDIT_PROTOCOL.md)

## 唯一裁决

```text
GROUP_STATISTIC_TASK_MISMATCH_PERSISTENT_PER_ANCHOR_SIGNAL_PRESENT
```

在 `P_FA=1e-5` 到 `0.10` 的全部离线 sweep 档位，clean 均为 `0/224` alarm，injected affected
均为 `0/30` alarm。最大 affected statistic `T=4.860523050243609` 的实际 DoF 为 5；要让它刚好
触发，chi-square upper-tail probability 必须放宽到 `0.43313837770065733`，不是对当前门限作小幅、
合理放宽即可解决。

与此相反，target anchor 20276 的 per-anchor conditional innovation 在 30 帧中呈稳定正向偏移：
injected 平均 `1.0926 sigma`、30/30 正号，累计 `Z_sum=5.9846`；同 identity 的 clean 为
`Z_sum=-0.6374`，paired shift 为 `Z_delta=6.6220`。其他 anchor 没有同等级的正向 paired shift，
最大正向值为 0（实际四个值均为负）。因此故障信息存在，但其主要形态是同一 anchor 跨 epoch
持续累积，而非单个 group 的 omnibus chi-square 异常。

本轮没有修改 production detector、概率、配置、cache、support 或 recovery；Stage2、recovery E2E
和下一 detector 均为 `NOT_RUN/NOT_IMPLEMENTED`。

## 1. Locked replay 与前置检查

复用了原 Walk1 clean/injected configs、初始化、输入 plan、conditional prior/replay、target link
`27956:20276`、闭区间 `[1664959678.3077347,1664959686.3077347]`、`+0.5 m` 和原 30 个 affected
obs IDs，没有重新生成场景。新增诊断重放与 f2ee3f0d sealed preflight 的 post-bootstrap epoch artifact
逐字段核对；clean/injected 的 `pl_conditional_epochs.csv` 均字节一致。

| 检查 | 结果 |
|---|---:|
| clean detected groups / alarms | 224 / 0 |
| injected affected observations covered | 30 / 30 |
| injected affected groups / alarms | 30 / 0 |
| max affected T / keyframe / DoF | 4.860523050243609 / 33 / 5 |
| current threshold at `P_FA=1e-5` | 30.85618994044592 |
| detector truth/GT/oracle successful opens | 0 |
| row diagnostic invalid count | 0 |

Production-style `nu,S,z` 和 conditional quantities 先由不接收 truth 参数、隐藏 truth/GT/oracle/data
roots 的两个独立进程生成并封存；statistics seal SHA-256 为
`df78c6594034255c6e8c05de4f415ee65e3f96afe3d4b01f6c9e768b275addfb`。独立 evaluator 验证
seal 与 file-open trace 后才读取 injection truth，按 `(group_id,anchor_id,obs_id)` 精确配对。

## 2. Sign 与 conditional formula

锁定 PL source 的 measurement rows 明确计算 `range_m - predicted`；IE UWB factor 的
`unwhitenedError` 是 `h+beta-z`，诊断 runner 使用其相反数。因此实际物理 innovation 是

```text
nu = z - h - beta
```

正 excess range `z += 0.5 m` 对应正 innovation；persistent signed evidence 的固定乘子为 `s=+1`，
不是看到 truth 后选出的方向。

对每个 row 使用同一完整 `S=HPH^T+R`，通过 LDLT solve 计算
`nu_m|-m=nu_m-S_m,-m solve(S_-m,-m,nu_-m)` 与对应 Schur conditional variance；没有显式求逆。
解析测试覆盖 diagonal SPD、correlated SPD、near-singular-but-valid、2-anchor 和 5-anchor；直接
GTest 为 13/13 通过，其中新增 conditional tests 为 5/5。数值无效时的 fail-closed reason 路径亦保留，
本次真实数据没有触发。

## 3. Group threshold sweep

每个 group 保持实际 `T,dof` 不变，只离线计算 threshold；下表的 threshold 按本数据实际出现的
DoF 4/5 分列。所有档位 clean alarm 与 affected recall 均为零。

| nominal P_FA | threshold DoF=4 | threshold DoF=5 | clean alarms / 224 | clean fraction | affected alarms / 30 | recall |
|---:|---:|---:|---:|---:|---:|---:|
| 1e-5 | 28.473255 | 30.856190 | 0 | 0 | 0 | 0 |
| 1e-4 | 23.512742 | 25.744832 | 0 | 0 | 0 | 0 |
| 1e-3 | 18.466827 | 20.515006 | 0 | 0 | 0 | 0 |
| 1e-2 | 13.276704 | 15.086272 | 0 | 0 | 0 | 0 |
| 0.05 | 9.487729 | 11.070498 | 0 | 0 | 0 | 0 |
| 0.10 | 7.779440 | 9.236357 | 0 | 0 | 0 | 0 |

最大 affected group 的 `T=4.860523`、DoF=5，对应
`P(ChiSquare_5 >= T)=0.4331383777`。也就是说，threshold-only 路径需要把单 epoch nominal false-alarm
tail 放宽到约 43.31% 才能碰到本数据的最大值；本协议定义的合理区间最高 5% 仍为 0 recall，即使额外
列出的 10% 也仍为 0 recall。

## 4. Target clean/injected paired signal

| conditional z | N | mean | median | sample std | P10 | P90 | positive fraction | max abs | Z_sum |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| clean | 30 | -0.1164 | -0.2483 | 0.3478 | -0.5023 | 0.3366 | 0.3667 | 0.7107 | -0.6374 |
| injected | 30 | 1.0926 | 1.0586 | 0.3195 | 0.7793 | 1.5142 | 1.0000 | 1.7781 | 5.9846 |
| injected-clean | 30 | 1.2090 | 1.2404 | 0.1649 | 0.9704 | 1.3549 | 1.0000 | 1.5115 | 6.6220 |

Target 的 single-epoch conditional signal 通常约 `1.09 sigma`，最大 `1.78 sigma`，不足以单 epoch
跨越 DoF=4/5 omnibus threshold；但相同正号连续出现 30 次，未归一 cumulative sum 最终为
`32.7790`，除以 `sqrt(30)` 后为 `5.9846`。clean 的对应终值为 `-0.6374`。逐 epoch 1--30 的
clean/injected/paired cumulative sequence 完整保存在 `pl_persistent_cumulative.csv`，未对该序列设置
detector threshold，也未产生 support。

Raw 与 marginal signal 更弱：target injected `nu` 平均 `0.17365 m`、median `0.16075 m`、正号比例
93.33%；相对 clean 的 paired mean 仅 `0.22836 m`。marginal z 的 injected mean 为 `0.28336 sigma`，
paired `Z_delta=2.0856`；利用同组相关性后的 conditional paired `Z_delta=6.6220`。

## 5. Healthy-anchor specificity

下表只比较相同 30 个 affected groups 内实际存在的 healthy observations；因此不同 anchor 的 N 可不同。

| anchor | N | injected mean conditional z | injected positive fraction | injected Z_sum | paired delta Z_sum |
|---:|---:|---:|---:|---:|---:|
| target 20276 | 30 | 1.0926 | 1.0000 | 5.9846 | 6.6220 |
| 7475 | 29 | 0.2071 | 0.8621 | 1.1152 | -0.7073 |
| 10548 | 23 | -0.1621 | 0.2609 | -0.7772 | -3.7271 |
| 15155 | 30 | -1.1529 | 0.0333 | -6.3149 | -4.9962 |
| 9524 | 29 | -1.2264 | 0.0000 | -6.6045 | -4.0092 |

Target 是唯一具有强正向 paired shift 的 anchor。healthy anchors 确实出现较大的反向 shift，最大绝对
paired `|Z_sum|=4.9962`；这说明同组相关/导航响应会把注入效应重新分配到其他 anchors，不能把它们描述为
完全不变。但在预先由 PL sign convention 锁定的“positive excess”方向，healthy 最大 paired evidence
为 0，而 target 为 6.6220，所以本数据中的 persistent positive signal 具有 anchor specificity。

## 6. 为什么 group T 只有 4.86

### Raw amplitude 与 variance

30 个 target injected rows 的 `|nu|` mean/median/P90/max 分别为
`0.19435/0.16549/0.34987/0.43741 m`；实际 `+0.5 m` 注入没有一比一留在 raw innovation 中。
平均 measurement variance `R_mm=0.05854 m^2`，平均 projected prior variance
`H_m P H_m^T=0.35899 m^2`，平均 total `S_mm=0.41752 m^2`。按逐 row fraction 再平均，measurement
占 15.72%，prior 占 84.28%；marginal normalization 主要受 prior uncertainty 支配。

### Correlation 与 contribution

平均 `S_m|-m / S_mm=0.30826`：conditioning on healthy anchors 明显减小而非增大 target variance，
这就是 conditional z 比 marginal z 更清晰的原因。30 个 affected groups 的总 `T=64.97845`；可加的
`nu_m*(S^-1 nu)_m` 分解中，target 合计 `18.08508`（27.83%），healthy rows 合计 `46.89337`
（72.17%）。这不是“正常 rows 把 T 数值除小”，而是 omnibus T 的多数背景 contribution 并非 target，
且每一 epoch 的 modest target evidence 不会跨 epoch 累积。

最大 T group 的具体分解为：target `nu=0.39498 m`，`R=0.04601 m^2`，prior variance
`1.22596 m^2`，`S_mm=1.27198 m^2`，marginal z 仅 `0.3502`；conditioning 后
`nu_cond=0.52052 m`、`S_cond=0.09405 m^2`、conditional z=`1.6973`。其 additive target/healthy
contributions 为 `2.18605/2.67448`，合计正好为 group `T=4.86052`，仍远低于 30.85619。
Conditional quadratic increments 是 leave-one-out 差值、不可跨 row 相加；artifact 中另列，未冒充
additive decomposition。

## 7. 五个问题的直接回答

1. **`P_FA=1e-5` 是否比 IE candidate detection 需要的严格？** 是，作为 integrity operating point 它
   极严；但本失败不只是“稍严”。最大 T 要求约 `P_FA=0.4331` 才刚触发。
2. **合理降低 group threshold 能否检测 +0.5 m？** 不能。`P_FA<=0.05` 全部 0/30，额外的 0.10
   也为 0/30，同时 clean 是 0/224。
3. **Target single-epoch conditional innovation 多大？** injected 平均约 `1.09 sigma`，median
   `1.06 sigma`，范围统计的 P10/P90 为 `0.78/1.51 sigma`，最大 `1.78 sigma`。
4. **30 次后 evidence 是否明显累积？** 是。injected `Z_sum=5.9846`，paired
   `Z_delta=6.6220`，clean 为 `-0.6374`。
5. **是否区别于 clean/healthy？** 在预先锁定的正 excess 方向上是：target 30/30 正号、paired
   `Z_delta=6.6220`；healthy 最大正向 paired value 为 0，最高 injected 正向 `Z_sum` 仅 1.1152。
   同时保留 healthy 大幅反向 shift 的限制说明。

## 8. Artifacts、测试与停止边界

证据根：`/home/mint/ws_fusion_uwb/res/pl_threshold_signal_audit_20260912_01`

- `pl_group_threshold_sweep.csv`
- `pl_per_anchor_innovations_clean.csv`
- `pl_per_anchor_innovations_injected.csv`
- `pl_conditional_innovations_clean.csv`
- `pl_conditional_innovations_injected.csv`
- `pl_clean_injected_paired.csv`
- `pl_persistent_cumulative.csv`
- `pl_persistent_signal_summary.json`
- `sealed_statistics_hashes.sha256`
- `clean_file_access.trace` / `injected_file_access.trace`
- `test_pl_conditional_raim.log` / `evaluator.log`

Build target `pl_conditional_preflight` 与 `test_pl_conditional_raim` exit 0；direct GTest 13/13，post-seal
evaluator exit 0。首次 evaluator criterion transcription 错把 healthy 反向绝对值当作正向竞争 evidence，
以及最终 trace gate 补齐时一次手写 truth 路径错误，均保存在 `attempts/`；二者都没有修改或重跑 sealed
statistics。最终 evaluator 先验证 truth-blind status 与 successful forbidden open count=0，再读取正确
locked truth。

依据 verdict B，后续若要评估 one-sided sequential CUSUM/GLR，必须另立任务和合同。本轮没有实现、
调参或授权该步骤，到此停止。
