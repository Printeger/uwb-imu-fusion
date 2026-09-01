
# Part VIII — Mathematical Baseline: Snapshot UWB Slope-Based RAIM

## 25. Measurement models

### 25.1 TWR range

对已知锚点 `a_i` 和标签位置 `p`：

\[
y_i=\|p-a_i\|+b_i^{cal}+v_i+f_i.
\]

若 calibration 已移除，可将 `b_i^{cal}` 设为零。若 UWB tag 与 IMU body origin 有 lever arm `\ell`，且姿态已知：

\[
h_i(p,R)=\|p+R\ell-a_i\|.
\]

纯 snapshot position baseline 应明确姿态与 lever arm 是否已知；否则 orientation 也可能进入 state。

### 25.2 One-way TOA

若 anchor 已同步但接收端 clock bias 未知：

\[
y_i=\|p-a_i\|+c\,\delta t_r+v_i+f_i.
\]

snapshot state 至少为 `[p, cδt_r]`。若还估计 drift，则需要多历元或额外模型。

### 25.3 TDoA

设原始 TOA 向量为 `y^{toa}`，差分矩阵为 `D`：

\[
y^{tdoa}=D y^{toa},\qquad
R_{tdoa}=D R_{toa}D^T.
\]

不能把所有差分当独立同方差。若以某个 anchor 为 reference，该 anchor 的物理故障会同时进入多条差分；fault incidence 应写为

\[
A_S^{tdoa}=D A_S^{toa}.
\]

正确 whitening 后，更换 reference anchor 应只改变坐标表示，不应显著改变最终估计与完整性结论。

## 26. Linearization and whitening

在当前点 `p_0`：

\[
y-h(p_0)\approx H_{raw}\delta p+v_{raw}+A_{raw,S}d_S.
\]

令

\[
R=LL^T,\qquad W=L^{-1},
\]

得到

\[
z=H\delta p+v+A_Sd_S,
\]

\[
z=W(y-h(p_0)),\quad H=WH_{raw},\quad A_S=WA_{raw,S},\quad v\sim\mathcal N(0,I).
\]

**白化必须同时作用于 measurement residual、Jacobian 和 fault selector。** 只对白化 residual 而不对白化 `A_S` 会使 slope 失去物理意义。

对 TWR：

\[
\frac{\partial h_i}{\partial p}
=\frac{(p_0-a_i)^T}{\|p_0-a_i\|}.
\]

若使用残差定义 `y-h(p)` 或 `h(p)-y`，Jacobian 符号会改变，但两边必须一致。

## 27. WLS and residual projector

若 `H` 满列秩：

\[
\delta\hat p=H^\dagger z,
\qquad
H^\dagger=(H^TH)^{-1}H^T.
\]

\[
P_\perp=I-HH^\dagger.
\]

\[
r=P_\perp z=P_\perp(v+A_Sd_S).
\]

数值实现不应显式计算 inverse；baseline 可以通过 QR/Cholesky solve 获得相同结果。显式 `P_⊥` 仅用于小型单元测试和公式验证。

## 28. Chi-square detector

令

\[
T=r^Tr.
\]

在 H0、固定 `H`、正确白化且 `rank(H)=m` 时：

\[
T\sim\chi^2_\nu,
\qquad \nu=n-m.
\]

阈值：

\[
T_D=F^{-1}_{\chi^2_\nu}(1-P_{FA}).
\]

故障下：

\[
T\sim\chi^2_\nu(\eta),
\qquad
\eta=\|P_\perp A_Sd_S\|^2.
\]

这里推荐统一记号：

- `η`：noncentrality parameter；
- `λ=sqrt(η)`：残差空间 fault magnitude。

给定分配的 `P_MD,S`，求 `η̄_S` 满足：

\[
F_{\chi^2_\nu(\bar\eta_S)}(T_D)=P_{MD,S},
\qquad \bar\lambda_S=\sqrt{\bar\eta_S}.
\]

## 29. State dimension and redundancy

设空间维数为 `d`、anchor 数为 `M`。

| Measurement | measurement dimension | snapshot state dimension | residual DOF | 至少有 1 个 residual DOF |
|---|---:|---:|---:|---:|
| TWR | `M` | `d` | `M-d` | `M≥d+1` |
| TOA with receiver clock bias | `M` | `d+1` | `M-d-1` | `M≥d+2` |
| TDoA from `M` anchors | `M-1` independent differences | `d` | `M-d-1` | `M≥d+2` |

这些是**局部线性冗余条件**，不保证全局唯一解或良好几何。在 3D 中，锚点共面、接近共线或用户位于不良几何区域时，即使 DOF 正数，PL 也可能极大。

对 `q` 维 fault subspace，除了 `ν≥q`，还要求：

\[
\operatorname{null}(P_\perp A_S)
\]

中不存在会改变被保护状态轴的 fault direction。

## 30. Failure-mode slope

对 axis `j`：

\[
g_{S,j}(d)=
\frac{|e_j^TH^\dagger A_Sd|}
{\|P_\perp A_Sd\|}.
\]

令

\[
q_{S,j}=A_S^TH^{\dagger T}e_j,
\qquad
G_S=A_S^TP_\perp A_S.
\]

则

\[
g_{S,j}^2(d)=\frac{(q_{S,j}^Td)^2}{d^TG_Sd}.
\]

若 `G_S≻0`：

\[
\bar g_{S,j}^2=q_{S,j}^TG_S^{-1}q_{S,j},
\]

即

\[
\bar g_{S,j}^2
=e_j^TH^\dagger A_S
(A_S^TP_\perp A_S)^{-1}
A_S^TH^{\dagger T}e_j.
\]

### 30.1 Singular detector Gram 的处理

不能为了得到有限数值而无条件使用 pseudoinverse。应检查：

- 若存在 `d≠0`，使 `P_⊥A_Sd=0` 且 `e_j^TH†A_Sd≠0`，则该 axis 的 slope 为 `+∞`；
- 若 detector-null direction 对被保护轴也无影响，可在 quotient space 上使用 `G_S†`；
- 对 HPL/VPL 或多轴联合保护，应检查整个 protected subspace，而不只是某一轴。

**工程政策：** 默认把任何“残差不可见但保护量受影响”的 hypothesis 标记为 `UNMONITORABLE`，PL 取无穷。

## 31. Fault hypotheses

### 31.1 Single-anchor fault

TWR 中：

\[
A_{\{i\}}=\frac{1}{\sigma_i}e_i
\]

（在 diagonal covariance 下）。其 fault dimension 为 1。

### 31.2 Multiple-anchor fault

对 `S={i_1,...,i_q}`，`A_S` 选择相应物理测量列。假设数为

\[
\sum_{s=1}^{s_{max}}{M\choose s}.
\]

第一版只做 single-anchor；multiple simultaneous faults 先保留接口和 `P_NM`，不要过早实现组合爆炸。

### 31.3 TDoA physical fault map

不要按“差分行”定义故障，应按物理 anchor 定义，再通过 `D` 映射。reference-anchor fault 影响所有包含 reference 的差分。

### 31.4 Anchor-map fault

anchor position uncertainty 不等同于 range bias。线性化后：

\[
\delta y_i\approx H_{p,i}\delta p + H_{a,i}\delta a_i.
\]

可将 `δa_i` 作为独立 bounded/probabilistic uncertainty，或将 anchor 加入 state；不能默默忽略。

## 32. Protection-level construction

Nominal covariance：

\[
\Sigma_p=(H^TH)^{-1}.
\]

轴向 nominal sigma：

\[
\sigma_j=\sqrt{e_j^T\Sigma_p e_j}.
\]

对每个故障模式分配风险，得到 `k_{0,j}` 和 `λ̄_S`。baseline：

\[
PL_{S,j}=k_0\sigma_j+\bar g_{S,j}\bar\lambda_S,
\]

\[
PL_j=\max_{S\in\mathcal S_{mon}}PL_{S,j}.
\]

如果所有假设使用相同 detector、相同 conditional `P_MD`，`λ̄_S` 可以相同；更一般地应按假设先验和风险分配分别求解。

### 32.1 HPL/VPL

最简单但保守的构造：

\[
HPL=\sqrt{PL_x^2+PL_y^2},\qquad VPL=PL_z.
\]

更正规的二维水平 PL 应基于二维误差椭圆、方向扫描或主轴 worst case，而不是把两个独立轴界机械相加。第一版可以同时输出：

- `axis_PL = [PL_x,PL_y,PL_z]`；
- conservative `HPL=sqrt(PL_x²+PL_y²)`；
- nominal covariance ellipse；
- 后续实现 direction-wise HPL。

## 33. Positive NLOS bias 能否收紧 slope

### 单个标量 TWR fault

若 `d≥0`，而 PL 保护的是 `|e_j|`，slope 对幅值和符号齐次；把 `d∈R` 限制为 `d≥0` 通常不会改变 single-scalar worst-case absolute slope。

### 多锚或一侧风险

对多个正 bias：

\[
\max_{d\ge0}\frac{|q^Td|}{\sqrt{d^TGd}}
\]

可能比无约束解更小。若只保护某一方向的一侧误差，也可进一步收紧。但是：

- TDoA fault 可以有符号；
- calibration residual/clock/timestamp fault 可以有符号；
- 经过 bias correction 后的残差不一定非负。

因此只应对“未校正传播延迟型 TWR NLOS”启用正锥约束。

## 34. Nominal UWB noise model

### 第一版

- per-anchor Gaussian；
- 支持 distance/received-power-dependent `σ_i`；
- 独立 TWR baseline；
- TDoA 使用 full covariance。

### 论文级版本

1. 对明确 LOS 数据做 calibration；
2. 剔除确定的设备/同步错误作为 fault，而不是 nominal；
3. 验证 standardized residual 的中心、方差、skewness、autocorrelation；
4. 构造 upper-tail overbound；
5. 在独立数据和不同轨迹验证；
6. 将 domain-out-of-calibration 标记为 unavailable。

GMM、Student-t 或 robust loss **不是自动 overbound**。需要证明/验证其尾概率不低估真实尾部。

## 35. Linearization error protection

对 range function `h_i(p)=||p-a_i||`：

\[
\nabla^2 h_i(p)=\frac{1}{\rho_i}(I-u_iu_i^T),
\qquad \|\nabla^2h_i\|_2\le\frac{1}{\rho_i}.
\]

在一个不靠近 anchor 的 trust region 内，可以构造二阶余项界：

\[
|r_{lin,i}|\le \frac{1}{2\rho_{min,i}}\|\delta p\|^2.
\]

可选处理：

- 把 remainder 作为额外 bounded measurement term；
- 将其映射为 axis-wise `PL_lin`；
- 迭代收缩 trust region 直到自洽；
- 若估计步长/故障边界超出 region，返回 unavailable。

第一版至少记录：

- NLS final increment；
- max standardized residual；
- minimum anchor distance；
- Jacobian condition number；
- finite-difference linearization error。

## 36. Snapshot baseline 的最终判定

### 正确之处

- canonical whitened model 正确；
- `P_⊥` detector 正确；
- fault-to-state / fault-to-residual ratio 正确；
- worst-case closed form 正确；
- `fault term + nominal term` 是合理的 residual-based RAIM baseline。

### 必须补上的条件

1. full-rank/gauge；
2. correct whitening；
3. physically correct `A_S`；
4. `A_S^TP_⊥A_S` 可监测性；
5. detector threshold 与 `P_FA`；
6. noncentral chi-square inversion 与 `P_MD`；
7. hypothesis prior / unmonitored risk；
8. nonlinear remainder；
9. TDoA correlation；
10. calibration/nominal overbound。

### 作为 baseline 的推荐范围

- 先做 3D TWR；
- known anchors、known tag-body relation；
- single-anchor additive bias；
- Gaussian LOS；
- axis PL + conservative HPL；
- no FDE；
- 所有高级功能保留接口。

---

# Part IX — Extension to UWB/IMU Incremental Smoothing

## 37. State and factor graph

### 37.1 Navigation state

建议每个 keyframe：

\[
X_k=(R_k,p_k,v_k,b^a_k,b^g_k),
\]

可选增加：

- tag/IMU extrinsic；
- receiver clock state（TOA）；
- anchor states（若 anchor map 不确定）；
- temporal NLOS bias state（研究扩展）。

### 37.2 Factors

- prior factor；
- IMU preintegration factor；
- bias random-walk factor；
- UWB range/TOA/TDoA factor；
- optional zero-velocity/static factor；
- optional anchor prior；
- fixed-lag marginal prior。

## 38. Frozen-linearization graph model

在 MAP 解和给定 retraction tangent coordinates 上：

\[
b=J\delta X+v+A_Sd_S.
\]

若 `J` 满列秩：

\[
\delta\hat X=J^\dagger b,
\quad
\delta\hat x_k=E_kJ^\dagger b.
\]

对 axis `j`，令

\[
c_{k,j}=E_k^Te_j.
\]

则：

\[
g_{S,k,j}(d)=
\frac{|c_{k,j}^TJ^\dagger A_Sd|}
{\|(I-JJ^\dagger)A_Sd\|}.
\]

其 worst-case 公式与 snapshot 同构。

### 38.1 数学成立的条件

- 以白化残差构造 `J`；
- 线性化点冻结；
- estimator 与 integrity 使用完全相同的 factor graph、ordering 和噪声模型；
- gauge 被物理 prior/anchors 固定；
- LM damping 不被误当成真实信息；
- robust weights 的影响被明确建模；
- `A_S` 覆盖 fault 影响的所有 factor rows；
- marginal prior 的历史 fault provenance 未丢失；
- protected coordinate 在同一 tangent/frame 下定义。

### 38.2 Nonlinear interpretation

该 PL 首先是**local linearized PL**：

- position axis 在选定导航 frame 中解释；
- orientation axis 在 `so(3)` tangent 中解释；
- 大 fault、大 relinearization 或多峰 posterior 可能使 Laplace/local WLS 近似失效；
- 应配套 linearization-validity gate 或 remainder term。

## 39. Gauge freedom and rank deficiency

### 常见问题

- UWB-only 对 orientation 不敏感；
- IMU gravity 约束 roll/pitch，但 yaw 可能弱可观；
- anchor map 若未固定会产生全局平移/旋转 gauge；
- clock state 与 range bias 可耦合；
- fixed-lag marginal prior 可能数值退化。

### 工程政策

- 不用 arbitrary tiny prior 把无穷 PL“压成有限”；
- 通过 rank-revealing QR、pivoted Cholesky 或 SVD 检查 protected subspace；
- 若 protected axis 不可观，PL=∞；
- LM damping 只用于求解，最终 sensitivity/covariance 使用 undamped information；
- 输出 rank、condition estimate、weak directions。

## 40. Avoiding explicit `J†` and `P_⊥`

令

\[
\Lambda=J^TJ,
\qquad
B_S=J^TA_S.
\]

求解

\[
Y_S=\Lambda^{-1}B_S.
\]

则：

### 40.1 Fault-to-state sensitivity

\[
\delta X_f=Y_Sd,
\qquad
\delta x_{k,f}=E_kY_Sd.
\]

### 40.2 Fault-to-residual Gram

\[
G_S=A_S^TP_\perp A_S
=A_S^TA_S-B_S^TY_S.
\]

无需显式构造 `P_⊥`。

### 40.3 Current-axis numerator

求解

\[
q_{k,j}=\Lambda^{-1}c_{k,j}.
\]

令

\[
s_{S,k,j}=B_S^Tq_{k,j}.
\]

则

\[
\bar g_{S,k,j}^2=s_{S,k,j}^TG_S^{-1}s_{S,k,j}.
\]

### 40.4 Current marginal covariance

\[
\Sigma_k=E_k\Lambda^{-1}E_k^T.
\]

可通过 Bayes-tree marginal query 或对 current-state basis 做多右端 triangular solves 得到。

## 41. Numerical formulation choices

| 方法 | 推荐用途 | 优点 | 风险 |
|---|---|---|---|
| Normal equations + sparse Cholesky | SPD、条件良好的 baseline | 与 GTSAM 信息矩阵/Bayes tree 接近 | 条件数平方 |
| Square-root QR | rank/gauge 诊断、数值基准 | 稳定、直接对应 residual space | 可能更慢 |
| Bayes tree conditional solves | incremental online backend | 复用 iSAM2 factorization | API/ordering 依赖，局部性非保证 |
| SVD/pseudoinverse | 小型 oracle/unit test | 清楚识别 nullspace | 不适合大图在线 |
| Sherman–Morrison/Woodbury | SS/rank-k hypothesis update | 小 fault mode 快 | 不适合所有 factor deletion，稳定性需验证 |

推荐：

- **dense SVD oracle**：测试用；
- **sparse square-root/Cholesky backend**：第一版 graph slope；
- **Bayes-tree cache/rank update**：确认瓶颈后再做。

## 42. What can be reused after an incremental update

可复用：

- variable ordering 和 key-index map；
- unaffected Bayes-tree cliques；
- factor metadata / fault incidence；
- current factorization handle；
- unchanged hypothesis `A_S^TA_S`；
- grouped multi-RHS solves；
- prior risk allocations；
- cached current-state extraction basis。

必须失效/重算的情况：

- affected factors relinearized；
- robust weights changed；
- variable reordering；
- marginalization changed prior；
- anchor calibration/noise model version changed；
- hypothesis support crosses window boundary；
- rank/condition status changed。

缓存键至少包含：

`graph_version + linearization_version + ordering_version + noise_model_version + hypothesis_signature`。

## 43. Marginalization is the central research trap

将 old variables `x_o` 与 retained variables `x_r` 的 normal equations 分块：

\[
\begin{bmatrix}
\Lambda_{oo} & \Lambda_{or}\\
\Lambda_{ro} & \Lambda_{rr}
\end{bmatrix}
\begin{bmatrix}x_o\\x_r\end{bmatrix}
=
\begin{bmatrix}\eta_o\\\eta_r\end{bmatrix}.
\]

Schur complement prior：

\[
\bar\Lambda_{rr}=\Lambda_{rr}-\Lambda_{ro}\Lambda_{oo}^{-1}\Lambda_{or},
\]

\[
\bar\eta_r=\eta_r-\Lambda_{ro}\Lambda_{oo}^{-1}\eta_o.
\]

若 fault 对 normal-equation RHS 的影响为

\[
\Delta\eta=
\begin{bmatrix}B_o\\B_r\end{bmatrix}d,
\]

则边缘化后的 fault-to-prior RHS sensitivity 为

\[
\bar B_r=B_r-\Lambda_{ro}\Lambda_{oo}^{-1}B_o.
\]

仅有 `\bar B_r` 还不够保存 detector 的 noncentrality。令原始 fault-only 二次项为

\[
\min_{x_o,x_r}
\left(
\begin{bmatrix}x_o\\x_r\end{bmatrix}^{T}
\Lambda
\begin{bmatrix}x_o\\x_r\end{bmatrix}
-2
\begin{bmatrix}x_o\\x_r\end{bmatrix}^{T}
\begin{bmatrix}B_o\\B_r\end{bmatrix}d
+d^{T}Cd
\right),
\qquad C=A_S^{T}A_S.
\]

消去 `x_o` 后，除 Schur information 和 `\bar B_r` 外，还必须保留

\[
\bar C=C-B_o^{T}\Lambda_{oo}^{-1}B_o.
\]

约化系统对 fault 的最小残差能量为

\[
G_S
=\bar C-\bar B_r^{T}\bar\Lambda_{rr}^{-1}\bar B_r
=C-B_S^{T}\Lambda^{-1}B_S
=A_S^{T}P_\perp A_S.
\]

因此：

- `\bar\Lambda_{rr}` 保存 nominal retained-state information；
- `\bar B_r` 保存 historical fault 对 retained-state normal-equation RHS 的作用；
- `\bar C` 保存被消除部分对 fault detectability/residual energy 的贡献。

**只保存普通 nominal marginal prior 会同时丢失历史 fault 的 state sensitivity 与 residual-energy provenance。** 对每个低维 hypothesis basis，应传播 `(\bar B_r,\bar C)`，或者保留可精确重构这些量的 square-root/原始因子表示。若只需要 conditional current-UWB detector，可把历史 fault 进一步压缩成 current-prior bias map `B_h`；但必须用 full-window oracle 验证该压缩不会低估 current-state slope。

更困难的是 detector residual sensitivity：标准 Schur prior 能保存 retained-state likelihood，却不会自动暴露原始 parity/residual directions。可行研究路线：

1. 保存低维 fault-to-prior state sensitivity `B_h`；
2. 采用 conditional current-UWB detector，而非试图恢复整个历史 residual；
3. 对必须监测的旧 fault，维护压缩的 detector Gram/innovation contribution；
4. 用 full-window oracle 验证压缩表示没有低估 slope。

## 44. Persistent fault hypothesis design

### 44.1 Event definitions

- `H(a,k)`：anchor `a` 在 epoch `k` fault；
- `H(a,k-L:k)`：连续 fault；
- `H(a,t_on,t_off,basis_type)`：物理 event；
- `H(region,r)`：一组 anchor 被共同遮挡（后续）。

### 44.2 Low-dimensional basis

令被影响的 measurement rows 为 `S_a`，temporal basis 为 `F`：

\[
f_S=A_{rows}F\theta.
\]

`θ` 维数保持 1–3，而不是每个 epoch 一个自由 bias。这样：

- 假设更符合 persistent obstruction；
- slope 计算保持小矩阵；
- 可检测性更强；
- hypothesis 数可控。

### 44.3 Hypothesis pruning

仅当以下任一成立时监测：

- prior probability 超过阈值；
- geometry-sensitive upper bound 可能导致 HMI；
- anchor quality/NLOS classifier 提高 prior；
- event 与当前窗口相交或其 sensitivity 尚未衰减到阈值以下。

未监测质量必须计入 `P_NM`。

## 45. Relation to existing incremental ARAIM / solution separation

### Residual-slope route

- 使用 all-in solution；
- denominator 是 detector 可见 fault component；
- numerator 是 fault 对 protected state 的线性敏感度；
- 通常一次 factorization + 多 RHS solve。

### Solution-separation route

- 对每个 fault mode 计算 full solution 与 fault-excluded/subset solution 的 separation；
- PL 使用 separation covariance 与 risk allocation；
- 可通过 rank update 避免完整重优化。

### 与 2026 scheduled abstract 的实质区别应写成

- 它公开描述的是 GNSS/IMU、ARAIM、solution separation、single-satellite/constellation fault 和 rank updates；
- 你的候选贡献应是 UWB-specific conditional residual slope、persistent physical-anchor fault、prior contamination cancellation 和 marginalization provenance；
- 两者可以互为 benchmark，而不是宣称完全没有 prior art。

## 46. Graph PL 推荐实现层次

### Level 0 — Nominal covariance bound

仅用于 debug，不称 RAIM PL。

### Level 1 — Current-epoch UWB faults on frozen graph

fault rows 只在当前 UWB factors；graph slope + global detector。用于数学验证。

### Level 2 — Conditional current-UWB detector

history+IMU prior + current UWB detector；推荐作为第一个 fusion PL baseline。

### Level 3 — Historical/persistent fault sensitivity

引入 `B_h/A_c`，监测 prior contamination。

### Level 4 — Provenance-preserving marginalization

窗口滑动后仍保持 relevant historical modes。

### Level 5 — Nonlinear/remainder/error-overbound closure

用于论文强版本。

---
# Part X — Detector Design

## 47. Why the whole-graph chi-square can dilute a local UWB fault

对全图 frozen linear model，nominal statistic：

\[
T_g=\|P_\perp b\|^2\sim\chi^2_{\nu_g}.
\]

当窗口增长时，`ν_g` 增大。对固定 `P_FA`，大自由度阈值近似：

\[
T_D\approx \nu_g+z_{1-P_{FA}}\sqrt{2\nu_g}.
\]

若一个局部 UWB fault 只产生近似固定 noncentrality `η_f`，则它相对于 nominal 标准差的分离度约为：

\[
\frac{\eta_f}{\sqrt{2\nu_g}},
\]

会随 `ν_g` 增大而下降。这就是“更多历史信息提高 accuracy，却稀释 detection statistic”的一种明确机制。

并非所有情况下都会恶化：历史信息也可能改变 `J`，减少 fault 被状态吸收的部分，从而提高 `η_f`。因此应通过两个独立量分析：

- state sensitivity：fault 对 current error 的影响；
- detector sensitivity：fault 的 noncentrality。

不能只看 RMSE 或 covariance。

## 48. Detector alternatives

| Detector | 优点 | 缺点 | 推荐角色 |
|---|---|---|---|
| Whole-graph residual | 简单，检查全局模型健康 | 稀释局部故障；DOF 大；robust/marginalization 分布复杂 | 非正式 global health gate |
| UWB-only post-fit residual | 聚焦 UWB | UWB residual 已受全图状态拟合影响，covariance 非单位 | 可做，但需推导 covariance |
| Conditional UWB innovation | prior 与当前 UWB 清晰分离；标准分布易定义 | prior 必须不含当前监测 UWB；历史 fault 可能污染 prior | **正式主 detector** |
| Parity-space UWB | 直接对应 fault detectability | 需要构造条件化 parity；少观测时维数低 | 与 conditional detector等价实现 |
| Marginalized current-state detector | history 压缩，计算小 | 压缩必须保留 correlation/provenance | 推荐数学结构 |
| Factor-local standardized residual | 可定位 anchor | 多重检验、相关性、post-fit shrinkage | diagnostic/FDE candidate |
| Solution separation | 解释直观，fault-specific | 多假设计算较重 | benchmark/后续 FDE |
| Global + local hierarchy | 覆盖模型失效与具体 UWB fault | 风险分配复杂 | 最终系统架构 |

## 49. Recommended conditional UWB detector

### 49.1 Construct a prior excluding the monitored current UWB batch

通过 history + IMU（以及允许的其他 trusted factors）得到：

\[
x_k\sim\mathcal N(\bar x_k^-,P_k^-).
\]

当前 UWB batch：

\[
z_{u,k}=h_u(x_k)+v_{u,k}+A_cd.
\]

线性化：

\[
\nu_k=z_{u,k}-h_u(\bar x_k^-),
\]

\[
S_k=H_kP_k^-H_k^T+R_{u,k}.
\]

在 H0、prior 与 current UWB 独立且模型正确时：

\[
T_{u,k}=\nu_k^TS_k^{-1}\nu_k\sim\chi^2_{m_u}.
\]

注意 DOF 是 UWB innovation dimension `m_u`，不是 `m_u-state_dim`；prior 本身提供了状态约束。

### 49.2 Current-only fault slope

Kalman-like gain：

\[
K_k=P_k^-H_k^TS_k^{-1}.
\]

当前 fault 引起的 posterior state error：

\[
e_f^+=K_kA_cd.
\]

detector-visible fault：

\[
S_k^{-1/2}A_cd.
\]

轴向 slope：

\[
g_{c,k,j}(d)=
\frac{|e_j^TK_kA_cd|}
{\|S_k^{-1/2}A_cd\|}.
\]

令

\[
q=A_c^TK_k^Te_j,
\qquad G=A_c^TS_k^{-1}A_c,
\]

则 `G≻0` 时：

\[
\bar g_{c,k,j}^2=q^TG^{-1}q.
\]

## 50. Historical and persistent fault extension

这是最值得发展的核心公式。

设 prior estimate error 在 hypothesis `H_S` 下包含：

\[
e_k^-=B_hd+n^-.
\]

同一物理 fault 对当前 UWB measurement 的直接影响为：

\[
A_cd.
\]

### 50.1 Innovation fault map

\[
D_h=A_c-H_kB_h.
\]

因为 prior 已偏移，当前观测与 prior 的差异并不等于原始 fault；两者可能相消。

### 50.2 Posterior current-state fault map

\[
C_h=(I-K_kH_k)B_h+K_kA_c.
\]

### 50.3 Conditional historical-fault slope

\[
g_{h,k,j}(d)=
\frac{|e_j^TC_hd|}
{\|S_k^{-1/2}D_hd\|}.
\]

令

\[
q_h=C_h^Te_j,
\qquad
G_h=D_h^TS_k^{-1}D_h.
\]

若 `G_h≻0`：

\[
\bar g_{h,k,j}^2=q_h^TG_h^{-1}q_h.
\]

### 50.4 最危险的 persistent-fault 模式

若

\[
A_c\approx H_kB_h,
\]

则 innovation 中 fault 接近相消：

\[
D_h\approx0,
\]

但 `C_h` 可能仍不为零，slope 会很大或无穷。这解释了为什么“IMU/history 越多越安全”不成立：一个持续 NLOS bias 可能已经把 smoother prior 拖向错误状态，而当前同一 anchor 的 bias 与错误 prior 自洽。

这个 cancellation metric 应作为论文关键图：

\[
\sigma_{min}(S^{-1/2}D_h)
\quad\text{vs}\quad
\|E_kC_h\|.
\]

## 51. How to obtain the current prior from a factor graph

推荐两种实现：

### Method A — Explicit factor partition

每个 epoch 更新时：

1. 先加入 IMU/history factors，不加入当前 UWB batch；
2. 调用 incremental update；
3. 查询 current state prior mean/covariance；
4. 构造 conditional UWB detector；
5. detector pass 后再加入 UWB factors；
6. 更新正式 all-in estimate。

优点：统计语义清楚。缺点：每 epoch 可能两次 update；需要控制 timing。

### Method B — Leave-current-UWB-out linear algebra

从 all-in graph 利用 factor removal/rank downdate 得到等效 prior，而不重复完整优化。优点快；实现和数值验证更复杂。第一版先用 Method A，后续再优化。

## 52. Asynchronous UWB handling

单个异步 range 的 innovation detector 有 1 DOF，可以检测与 prior 的不一致，但不能提供 snapshot spatial redundancy。建议配置：

- `epoch_bin`: 将短时间窗内 ranges 成组；
- `max_time_skew`: 超出则不合并；
- 使用状态插值/连续时间或为每个 range 精确插入 factor；
- detector 使用与 estimator 一致的时间模型；
- 报告 group size 对 P_FA/P_MD 和 latency 的影响。

## 53. Hierarchical detector architecture

推荐最终结构：

1. **Model-validity gate：** rank、clock、anchor map、linearization、calibration domain；
2. **Global graph-health detector：** 检查严重模型失效，不直接用于主 PL；
3. **Formal conditional UWB group detector：** 与 slope/PL 共用 statistic；
4. **Anchor-local diagnostics：** standardized innovation、leave-one-anchor score；
5. **Temporal detector：** CUSUM/GLR 或 persistent hypothesis posterior；
6. **FDE/availability decision：** 独立 policy 层。

多 detector 同时使用时，必须分配 total P_FA/continuity risk；不能每个都用完整预算。

---

# Part XI — Software Architecture Handbook

## 54. Recommended technology stack

| Layer | Recommendation | Reason |
|---|---|---|
| Core | C++20（若环境限制则 C++17） | 强类型、性能、GTSAM 接口 |
| Optimization | GTSAM 4.x, iSAM2；fixed-lag adapter | 成熟增量图与 Bayes tree |
| Linear algebra | Eigen；必要时 SuiteSparse through GTSAM | dense oracle + sparse backend |
| Configuration | YAML + schema validation | 实验可复现 |
| Data | CSV first；ROS bag adapter；可选 Parquet | 先简单、再扩展 |
| Analysis | Python 3.11+, NumPy/SciPy/Pandas/Matplotlib | Monte Carlo 与论文图 |
| Tests | GoogleTest + CTest | C++ 单元/集成 |
| Benchmark | `std::chrono::steady_clock` + Linux perf/Tracy 可选 | 低侵入 timing |
| Logging | CSV/JSONL + structured metadata | 人可读、脚本友好 |
| Build | CMake presets + dependency lock | 可复现 |
| CI | GitHub Actions/local CI | 编译、单测、格式检查 |

第一阶段不要依赖 ROS 运行主实验；把 ROS bag 作为 dataset adapter，核心算法使用离线 replay API。

## 55. Repository tree

```text
uwb-imu-pl/
├── CMakeLists.txt
├── CMakePresets.json
├── cmake/
├── apps/
│   ├── snapshot_replay/
│   ├── incremental_replay/
│   ├── run_experiment/
│   ├── inspect_epoch/
│   └── benchmark_integrity/
├── include/uwb_imu_pl/
├── src/
│   ├── common/
│   ├── data_model/
│   ├── io/
│   ├── datasets/
│   ├── simulation/
│   ├── calibration/
│   ├── estimation/
│   │   ├── snapshot/
│   │   ├── uwb_incremental/
│   │   ├── uwb_imu_incremental/
│   │   ├── fixed_lag/
│   │   └── reference_batch/
│   ├── factors/
│   ├── imu/
│   ├── uwb/
│   ├── linearization/
│   ├── integrity/
│   │   ├── contracts/
│   │   ├── noise/
│   │   ├── fault_models/
│   │   ├── hypotheses/
│   │   ├── detection/
│   │   ├── sensitivity/
│   │   ├── protection_level/
│   │   ├── fde/
│   │   ├── risk/
│   │   └── availability/
│   ├── experiment/
│   ├── evaluation/
│   ├── profiling/
│   └── logging/
├── configs/
│   ├── datasets/
│   ├── sensors/
│   ├── estimators/
│   ├── integrity/
│   ├── faults/
│   └── studies/
├── tests/
│   ├── unit/
│   ├── numerical/
│   ├── property/
│   ├── integration/
│   └── regression/
├── python/
│   └── uwb_imu_pl_analysis/
├── scripts/
├── docs/
├── examples/
├── third_party/
└── results/              # gitignored
```

## 56. Module responsibility table

| Module | Responsibility | Inputs | Outputs | Dependencies | Must not do |
|---|---|---|---|---|---|
| `data_model` | 定义不可变领域对象、IDs、timestamps | raw parsed fields | typed records | common only | 文件读取、估计、PL policy |
| `io` | CSV/JSON/ROS bag 序列化 | streams/files | records | data_model | 解释传感器数学 |
| `datasets` | 数据集适配、时间排序、GT/anchor map | files/config | `DatasetSequence` | io/data_model | 注入 faults、运行 estimator |
| `simulation` | 轨迹/anchors/measurement generation | scenario + seed | synthetic sequence | data_model/math | 修改 estimator 内部 |
| `calibration` | UWB bias/noise/extrinsic calibration | calibration data | versioned models | uwb/data_model | 完整性决策 |
| `estimation/snapshot` | 单帧 WLS/NLS | UWB batch + anchors | state/cov/linearization | uwb/linearization | PL/risk/FDE policy |
| `estimation/uwb_incremental` | UWB-only graph bridge baseline | UWB stream | incremental state | GTSAM/factors | integrity policy |
| `estimation/uwb_imu_incremental` | tightly coupled iSAM2 | IMU/UWB | nav state + snapshot handle | GTSAM/imu/factors | fault injection、PL |
| `fixed_lag` | marginalization/window policy | factors/state | fixed-lag posterior | GTSAM | 丢弃 provenance without notice |
| `reference_batch` | offline oracle | full sequence | batch solution | GTSAM | 被当作最终在线算法 |
| `factors` | GTSAM factors/Jacobians | state/measurement | residual/Jacobian | GTSAM | detector thresholds |
| `linearization` | 统一白化、factor row metadata、sparse views | estimator graph | immutable linear system view | Eigen/GTSAM | 风险分配 |
| `integrity/noise` | nominal overbound/correlation models | residual features | covariance/overbound | calibration | outlier deletion |
| `fault_models` | 物理 fault parameterization | anchor/time/measurement type | fault basis | data_model | 枚举所有 hypotheses |
| `hypotheses` | 生成/prune monitored set | fault model + priors | hypothesis list | fault_models/risk | 求解 estimator |
| `detection` | statistics/thresholds/decision | integrity snapshot | detector result | linearization/noise | 修改 graph weights |
| `sensitivity` | slope ingredients、sparse solves | linear view + hypothesis | sensitivity result | estimator contract | allocation/availability policy |
| `protection_level` | nominal + fault terms | sensitivity + risk | PL result | risk/detection | 估计 state |
| `fde` | exclusion candidates and post-FDE contract | detector + hypotheses | exclusion request | integrity modules | 直接更改 estimator；应返回请求 |
| `risk` | budgets, priors, allocation | requirements/config | allocations | common | detector implementation |
| `availability` | compare PL/AL/model validity | PL + status | available/alert reason | integrity outputs | 重算 PL |
| `experiment` | Cartesian-product run orchestration | study config | run manifests | all via interfaces | 算法特例硬编码 |
| `evaluation` | metrics/aggregation | outputs + GT | summaries | data_model | 改算法 |
| `profiling` | scoped timing/memory counters | execution events | timing records | common | 日志 I/O 混入 timed region |
| `logging` | structured outputs | records | CSV/JSONL | io | 业务逻辑 |

## 57. Core design principles

1. estimator 不负责 integrity policy；
2. integrity 通过只读 contract 获取数学对象；
3. integrity 不修改 estimator state；
4. FDE 返回 `ExclusionRequest`，由 orchestration 层决定是否重跑；
5. fault injection 位于 simulation/replay decorator，不写入 estimator；
6. dataset adapter 不知道算法；
7. noise model、fault model、risk model 分开；
8. `FactorId` 与 `MeasurementId` 全程可追踪；
9. marginalization 必须产生 provenance event；
10. robust loss/weight 必须在 snapshot 中可见；
11. 所有结果绑定 config hash、git SHA、seed 和 dependency version；
12. batch solver 仅作 oracle/reference。

## 58. End-to-end data flow

```text
DatasetAdapter / Simulator
        │
        ├── raw measurements ──> FaultInjectionDecorator ──> ReplayScheduler
        │                                                     │
        │                                                     v
        │                                             Estimator Interface
        │                                                     │
        │                                           EstimationSnapshot
        │                                                     │
        └── ground truth ───────────────────────────────┐      v
                                                       │ IntegrityMonitor
                                                       │      │
                                                       v      v
                                                  Evaluation + Logging
```

Estimator 与 IntegrityMonitor 之间只传递 `EstimationSnapshot` 和 capability-based solver handle，不传可修改的 GTSAM graph 指针。

---

# Part XII — Data Structures and Interfaces

## 59. Fundamental identifiers

### `Timestamp`

- `int64 nanoseconds`；
- `ClockDomain`：sensor/raw/system/synchronized；
- 不用 `double seconds` 作为唯一时间键；
- 提供显式转换与比较。

### IDs

- `TrajectoryId`；
- `SequenceId`；
- `AnchorId`；
- `SensorId`；
- `MeasurementId`；
- `FactorId`；
- `StateKey`；
- `HypothesisId`；
- `RunId`。

## 60. Navigation and sensor objects

### `NavigationState`

- timestamp；
- position `p_WB`；
- velocity `v_WB`；
- orientation `q_WB` / `R_WB`；
- accelerometer bias；
- gyroscope bias；
- optional clock bias/drift；
- state frame and tangent convention；
- estimate status；
- source graph version。

### `UwbMeasurement`

- measurement ID；
- timestamp / transmit / receive timestamps if available；
- type: TWR / TOA / TDoA；
- tag ID；
- anchor ID；
- reference anchor ID for TDoA；
- raw range/time；
- corrected range；
- calibration version；
- nominal sigma/covariance group ID；
- received power；
- first-path power；
- CIR quality features；
- LOS/NLOS label if ground truth exists；
- quality flags；
- sequence/order counter；
- original dataset row provenance。

### `ImuMeasurement`

- timestamp；
- specific force；
- angular velocity；
- covariance or calibrated noise density；
- saturation/temperature/quality flags；
- sensor frame；
- calibration version。

### `AnchorRecord`

- anchor ID；
- position and frame；
- covariance or bounded set；
- clock/synchronization metadata；
- antenna-delay calibration；
- active interval；
- map version。

### `AnchorMap`

- frame；
- anchor records；
- map version/hash；
- coordinate transform provenance。

## 61. Dataset and truth objects

### `TrajectoryGroundTruth`

- timestamped pose/velocity；
- covariance/quality if GT not exact；
- interpolation policy；
- frame transforms；
- valid intervals。

### `DatasetSequence`

- sequence metadata；
- ordered UWB stream；
- ordered IMU stream；
- anchor map；
- ground truth；
- calibration references；
- known NLOS/occlusion intervals；
- replay start/end；
- data-quality report。

## 62. Estimation outputs

### `EstimatorOutput`

- epoch/time；
- current navigation state；
- selected marginal covariance；
- optimizer status；
- nonlinear cost；
- number of iterations/relinearized variables；
- active variable/factor count；
- rank/condition diagnostics；
- timing summary；
- graph/linearization version。

### `ResidualBlock`

- factor ID；
- factor type；
- measurement IDs；
- anchor IDs；
- timestamp span；
- raw residual；
- whitened residual；
- row range in linear system；
- covariance/whitener ID；
- robust loss and final weight；
- Jacobian block metadata；
- marginalization provenance。

## 63. Fault and risk objects

### `FaultHypothesis`

- hypothesis ID；
- physical fault type；
- affected anchor IDs；
- onset/end or timestamp set；
- temporal basis type and dimension；
- affected measurement/factor IDs；
- fault incidence/basis descriptor；
- sign/cone constraints；
- prior probability or conservative upper bound；
- monitored/unmonitored flag；
- parent event/group；
- required detector；
- pruning reason；
- validity interval。

### `FaultScenario`

用于注入，和 hypothesis 分开：

- truth fault event；
- exact amplitude/time profile；
- seed；
- injection point: raw timestamp/range/anchor position；
- no access to estimator internals。

### `RiskBudget`

- required integrity risk；
- continuity/P_FA budget；
- nominal H0 allocation；
- monitored-fault allocation；
- unmonitored risk；
- model-validity allocation；
- per-axis or horizontal/vertical budgets；
- allocation method/version。

## 64. Integrity objects

### `DetectorResult`

- detector type；
- statistic；
- threshold；
- DOF；
- P_FA allocation；
- pass/fail；
- innovation/residual dimension；
- conditioning/rank status；
- local anchor scores；
- model-validity flags；
- timing。

### `SensitivityResult`

- hypothesis ID；
- protected axis/frame；
- fault-to-current-state map summary；
- detector Gram；
- worst-case direction；
- max slope；
- finite/infinite status；
- numerical condition；
- solver calls/flops proxy；
- timing。

### `ProtectionLevelResult`

- timestamp；
- axis PL；
- HPL/VPL and construction type；
- nominal component；
- fault component；
- linearization/model component；
- maximizing hypothesis；
- detector statistic/threshold reference；
- alert limits；
- availability；
- HMI-risk requirement；
- unmonitored risk；
- assumption validity；
- formal/heuristic label；
- compute time。

### `IntegrityOutput`

聚合 estimator state、detector、PL、FDE candidate、availability、timing，供 logging/evaluation 使用。

## 65. Timing and experiment objects

### `TimingRecord`

- run/epoch/thread ID；
- stage name；
- wall duration；
- CPU duration optional；
- cold/warm flag；
- active factors/variables/anchors/hypotheses；
- window length；
- solver version；
- success/failure status。

### `ExperimentConfig`

- dataset/sequence；
- estimator config；
- sensor/noise config；
- integrity config；
- fault scenario；
- seeds；
- alert limits；
- output policy；
- profiling policy；
- config hash。

### `RunManifest`

- timestamp；
- git SHA / dirty flag；
- compiler/build flags；
- OS/CPU/RAM；
- GTSAM/Eigen versions；
- config paths and resolved config；
- dataset hashes；
- random seeds；
- result schema version。

## 66. Estimator interface

推荐抽象能力，而不是强迫所有 estimator 实现所有方法。

### Common lifecycle

- `initialize(context)`；
- `ingestImu(measurement)`；
- `ingestUwb(measurement)`；
- `update(request)`；
- `currentState()`；
- `timing()`；
- `reset()`。

### Optional capabilities

- current marginal covariance；
- selected joint marginals；
- factor residual blocks；
- immutable linearized system snapshot；
- sparse information solve；
- factor removal/temporary exclusion；
- pre-UWB current-state prior；
- Bayes-tree/factorization version metadata；
- marginalization provenance。

Snapshot WLS 实现 dense linear model；iSAM2 实现 sparse solver handle；batch reference 只用于 oracle。

## 67. Estimator ↔ IntegrityMonitor contract

### `EstimationSnapshot` 必须提供

1. epoch/current state key；
2. current linearization point；
3. protected-state coordinate/frame；
4. factor IDs、row blocks、timestamps、anchor IDs；
5. whitened residual blocks；
6. noise/whitening model IDs；
7. variable ordering/index map；
8. read-only `applyJ(x)` / `applyJT(y)` 或 equivalent block access；
9. `solveInformation(rhs)`，复用当前 factorization；
10. selected marginal query；
11. graph/factorization/linearization versions；
12. numerical rank/condition diagnostics；
13. robust weights/loss/damping flags；
14. prior/marginalization provenance；
15. factor-selection scope，如 pre-current-UWB or all-in。

### Contract must not expose

- mutable graph；
- mutable Values；
- raw optimizer internals without lifetime/version guarantee；
- “一个 covariance”而没有 state ordering/frame；
- residual without whitening metadata。

### Capability negotiation

Integrity backend 先查询：

- `supportsDenseSnapshot()`；
- `supportsSparseSolve()`；
- `supportsPreMeasurementPrior()`；
- `supportsFactorProvenance()`；
- `supportsTemporaryFactorRemoval()`。

缺能力时显式降级或返回 unsupported，不能静默换成 covariance-only PL。

## 68. Integrity/RAIM architecture

### Shared modules

- `FaultHypothesis`；
- `RiskBudget` / `RiskAllocator`；
- `NoiseOverbound`；
- `DetectorResult`；
- `ProtectionLevelResult`；
- `AvailabilityMonitor`；
- `IntegrityLogger`；
- experiment/evaluation framework。

### Backend-specific modules

#### `SnapshotUwbRaimBackend`

- dense `H`；
- explicit/small QR；
- snapshot residual projector；
- single/multi-anchor row selector。

#### `GraphSparseRaimBackend`

- factor row provenance；
- sparse information solves；
- current-state extraction；
- graph fault basis；
- marginalization handling。

#### `ConditionalPriorRaimBackend`

- pre-UWB prior `P^-`；
- innovation covariance `S`；
- current/historical `B_h,A_c`；
- conditional slopes。

#### Future `SolutionSeparationBackend`

- factor removal/subgraph solution；
- rank update/downdate；
- post-exclusion covariance。

PL calculator 不直接依赖 GTSAM；它只接收标准化 sensitivity 和 risk allocation。

---
# Part XIII — Development Milestones

## 69. Milestone overview

| Milestone | Deliverable | Target |
|---|---|---|
| M0 | semantics, repository, replay, config, logging, timing skeleton | Day 1–3 |
| M1 | pure-UWB snapshot WLS/NLS | Week 1 |
| M2 | snapshot chi-square detector | Week 1–2 |
| M3 | snapshot single-anchor slope PL | Week 2 |
| M4 | analytical + Monte Carlo integrity validation | Week 2 |
| M5 | UWB-only incremental graph bridge | Week 3 start |
| M6 | IMU preintegration validation | Week 3 |
| M7 | tightly coupled UWB/IMU iSAM2 baseline | Week 3 |
| M8 | conditional UWB detector for fusion | Week 4 |
| M9 | sparse graph-slope backend | After baseline demo |
| M10 | historical/persistent fault PL + provenance | Main research phase |
| M11 | full benchmark, multiple datasets, paper pipeline | continuous |

## 70. M0 — Research contract and experimental skeleton

### Complete when

- repository builds in Release/Debug；
- one YAML resolves into a `RunManifest`；
- replay emits ordered IMU/UWB events；
- all random generators require explicit seed；
- timing stages and CSV/JSON schema exist；
- result folder contains config hash/git SHA。

### Mathematical validation

- units, frames, quaternion convention, timestamp policy written in `docs/conventions.md`；
- integrity definitions and HMI event written in `docs/integrity_semantics.md`。

### Unit tests

- timestamp ordering；
- frame transforms；
- YAML validation and missing-field failure；
- deterministic replay under same seed。

### Integration test

- synthetic 10-second sequence replayed twice yields byte-identical measurement log。

### Expected plots/tables

- event-rate timeline；
- dataset quality summary。

### Common bugs

- seconds vs nanoseconds；
- body/world quaternion direction；
- anchor coordinates in different frame；
- hidden default random seed。

### Gate

No estimator coding until conventions and manifest are fixed.

## 71. M1 — Pure UWB snapshot WLS/NLS

### Complete when

- supports 2D/3D TWR；
- supports heteroscedastic covariance；
- outputs state, covariance, whitened `H`, residual, rank/condition；
- converges on multiple initial guesses or reports failure；
- can replay multiple epochs independently。

### Mathematical validation

- analytic Jacobian vs finite difference；
- WLS solution vs dense reference；
- covariance vs Monte Carlo under small-noise linear regime；
- scale/translation invariance of coordinate frame。

### Unit tests

- noiseless geometry；
- one bad geometry case；
- diagonal and full covariance；
- rank-deficient anchors returns unavailable；
- TDoA correlation test reserved if TDoA enabled。

### Integration test

- straight/circle/figure-eight synthetic trajectories, no faults；
- compare epoch-wise WLS to ground truth。

### Expected plots

- trajectory + anchors；
- position error vs time；
- condition number/GDOP vs time；
- standardized residual histogram。

### Common bugs

- range residual sign；
- whitening only residual not Jacobian；
- covariance using raw `H` after whitening；
- confusing local rank with global ambiguity。

### Gate

No RAIM until standardized nominal residual is approximately consistent with the configured model.

## 72. M2 — Snapshot residual chi-square detector

### Complete when

- computes `T=r^Tr` and correct DOF；
- threshold derives from configured `P_FA`；
- outputs pass/fail and numerical validity；
- supports per-epoch measurements with changing anchor set。

### Mathematical validation

- projector symmetry/idempotence in dense oracle；
- `rank(P_⊥)=n-rank(H)`；
- empirical H0 CDF matches chi-square within confidence bounds；
- injected fault follows predicted noncentral chi-square in linear model。

### Unit tests

- exact small matrices with known projector；
- orthogonal fault direction；
- fault in column space；
- whitening invariance under invertible measurement scaling。

### Integration test

- Monte Carlo over geometry/noise seeds；
- measured false alarm rate with binomial confidence interval。

### Expected plots

- empirical/theoretical chi-square CDF；
- statistic and threshold over time；
- detection probability vs fault magnitude。

### Common bugs

- using residual norm threshold while computing squared statistic；
- wrong DOF after dropped measurements；
- reusing threshold when rank changes；
- detector on robust-weighted residual while still assuming standard chi-square。

### Gate

H0 false-alarm rate must agree with target within statistical uncertainty before PL implementation.

## 73. M3 — Snapshot single-anchor failure-slope PL

### Complete when

- single-anchor hypotheses generated from physical anchor IDs；
- computes `G`, slope, worst-case direction；
- detects infinite/unmonitorable hypotheses；
- solves noncentrality boundary；
- outputs nominal/fault components, maximizing anchor and axis PL。

### Mathematical validation

- closed-form slope vs direct constrained optimization；
- explicit `P_⊥` vs solve-based formula；
- slope invariant to fault amplitude scaling；
- sign invariance for two-sided scalar fault；
- poor geometry makes PL grow。

### Unit tests

- hand-designed 2D matrix；
- exactly unobservable fault；
- nearly singular detector Gram；
- multiple axes；
- hypothesis mapping after anchor dropout。

### Integration test

- injected single-anchor bias sweep；
- comparison of predicted `P_MD` and empirical missed detections；
- PL vs PE across trajectories。

### Expected plots

- PE and PL time series；
- max slope and responsible anchor；
- PL vs fault threshold/risk allocation；
- geometry heatmap of PL。

### Common bugs

- `λ` vs `λ²`；
- using `pinv(G)` to hide infinite slope；
- neglecting no-fault tail；
- physical anchor fault mapped incorrectly in TDoA。

### Gate

Dense analytical tests and Monte Carlo must both pass.

## 74. M4 — Snapshot integrity validation harness

### Complete when

A standardized sweep exists over:

- trajectory；
- anchor geometry/count；
- noise；
- fault magnitude/anchor/time；
- random seed；
- risk budget。

### Required validation

1. analytical sanity check；
2. H0 false-alarm calibration；
3. noncentral `P_MD` calibration；
4. single-anchor bias injection；
5. geometry sweep；
6. anchor-number sweep；
7. noise sweep；
8. LOS vs NLOS scenarios；
9. PL coverage；
10. operational HMI and availability。

### Expected tables

- empirical/theoretical P_FA；
- empirical/theoretical P_MD by bias；
- RMSE/PL/availability by geometry；
- timing baseline。

### Gate

This milestone is the mathematical baseline to show the advisor before fusion PL is attempted.

## 75. M5 — UWB-only incremental estimator

### Purpose

A debugging bridge between snapshot and UWB/IMU. Use position/velocity states with a simple motion/smoothness factor and UWB factors.

### Complete when

- incremental state keys are deterministic；
- asynchronous UWB supported；
- marginal covariance query works；
- batch reference and incremental result agree on small graph；
- residual metadata are exported。

### Tests

- batch vs iSAM2；
- delayed measurement insertion；
- changing anchor set；
- relinearization settings regression。

### Common bugs

- double-counting prior；
- incorrect state timestamp association；
- covariance query after stale update；
- factor IDs lost during incremental update。

### Gate

Integrity contract can consume both snapshot WLS and UWB incremental estimator without algorithm-specific casts.

## 76. M6 — IMU preintegration

### Complete when

- static initialization estimates gravity direction and biases；
- preintegration reset/keyframe policy fixed；
- bias Jacobians verified；
- unit/frame conventions documented；
- synthetic motion recovers ground truth without UWB within expected drift。

### Mathematical tests

- constant acceleration；
- constant angular velocity；
- zero-motion；
- bias perturbation finite differences；
- covariance propagation Monte Carlo。

### Integration test

- compare GTSAM preintegration with independent numerical integration on short trajectories。

### Expected plots

- preintegrated delta error；
- bias convergence；
- IMU-only drift。

### Common bugs

- gravity sign；
- deg/s vs rad/s；
- accelerometer specific force interpretation；
- body/world convention；
- timestamps duplicated or gaps。

### Gate

All synthetic preintegration tests pass before UWB factors are added.

## 77. M7 — Tightly coupled UWB/IMU incremental smoother

### Complete when

- state `(R,p,v,b_a,b_g)`；
- IMU preintegration + bias RW + UWB factors；
- iSAM2 update at UWB/keyframe policy；
- initialization and failure recovery；
- current state/marginal/residual metadata；
- multiple trajectories replay；
- timing breakdown。

### Mathematical validation

- batch vs incremental small graph；
- marginal covariance vs dense inverse toy graph；
- UWB factor Jacobian including lever arm；
- observability/rank diagnostics。

### Integration tests

- good geometry；
- poor geometry；
- anchor outage；
- UWB delay/drop；
- IMU gaps；
- injected NLOS not yet used for formal PL but visible in residuals。

### Expected plots

- UWB-only vs UWB/IMU trajectory；
- position/velocity/orientation error；
- covariance vs actual error；
- active factors/relinearization/time。

### Common bugs

- overconfident IMU noise；
- UWB and IMU timestamp mismatch；
- incorrect tag lever arm；
- hidden robust kernel；
- arbitrary priors masking gauge。

### Gate

Accuracy baseline stable and deterministic; estimator contract complete.

## 78. M8 — Conditional UWB detector

### Complete when

- produces a pre-current-UWB prior；
- computes `ν,S,T`；
- handles current UWB covariance/correlation；
- compares global vs conditional detector；
- logs group size and latency。

### Mathematical validation

- synthetic Gaussian H0 chi-square；
- current-only fault noncentrality；
- detector power vs graph window length；
- no double counting of current UWB in prior。

### Expected key figure

Detection probability vs window length for:

- global graph residual；
- UWB-only post-fit residual；
- conditional UWB innovation。

### Gate

Conditional detector achieves calibrated P_FA and does not exhibit artificial degradation solely from added unrelated history factors.

## 79. M9 — Sparse graph-slope backend

### Complete when

- dense oracle and sparse formula agree；
- no explicit global pseudoinverse/projector；
- multi-RHS solves and current marginal query instrumented；
- fault hypothesis row maps validated；
- numerical rank failures explicit。

### Tests

- random sparse toy graphs；
- full graph vs snapshot limit；
- current-only UWB hypotheses；
- timing vs hypotheses/window size。

### Gate

All relative errors are below a condition-number-aware tolerance and no silent regularization occurs.

## 80. M10 — Historical/persistent fault PL

### Complete when

- low-dimensional persistent basis；
- `B_h,A_c,D_h,C_h` computed/propagated；
- cancellation/unmonitorable modes detected；
- fixed-lag boundary crossing tested；
- full-window oracle validates compressed method；
- risk allocation includes onset/duration/unmonitored events。

### Required experiments

- fault onset before/inside/after window；
- fault persists across marginalization；
- same anchor vs changing anchor；
- constant vs ramp bias；
- window length and IMU quality；
- current anchor outage after prior contamination。

### Gate

No historical fault mode is dropped merely because its original factor was marginalized.

## 81. M11 — Paper-grade benchmark and reproducibility

### Complete when

- one command expands study matrix；
- all results have manifests；
- all figures regenerated from raw logs；
- timing is repeatable；
- real-data calibration/test split fixed；
- rare-event validation plan documented；
- ablations and negative results included。

---

# Part XIV — Experiment Design

## 82. Why multiple trajectories are mandatory

Integrity depends on geometry, motion excitation, UWB visibility and fault persistence—not just sensor noise. A single trajectory can accidentally remain in good geometry, never excite IMU biases, or align faults with easy-to-detect directions. Multiple trajectories are required to sample:

- changing anchor geometry；
- low/high speed；
- turning and attitude excitation；
- repeated visits to NLOS regions；
- window history composition；
- anchor outage transitions；
- observability of vertical/yaw components。

## 83. Trajectory library

### Synthetic minimum set

1. straight line；
2. circle；
3. figure-eight；
4. sharp-turn/piecewise linear；
5. stop-and-go；
6. 3D helix or ascent/descent；
7. poor-geometry corridor；
8. NLOS region crossing；
9. anchor outage segment；
10. repeated loop through same obstruction。

### Geometry variants

- anchors surrounding trajectory；
- anchors on one side；
- coplanar 3D anchors；
- one distant anchor；
- variable anchor count；
- temporary reference-anchor loss for TDoA。

## 84. Dataset abstraction

A dataset adapter must implement conceptually：

- `metadata()`；
- `anchorMap()`；
- `imuStream()`；
- `uwbStream()`；
- `groundTruth()`；
- `calibration()`；
- `knownEvents()`；
- `qualityReport()`。

Adapters convert source formats only. Synchronization policy is configured in replay/estimator, not hard-coded in dataset parser.

## 85. Unified experiment Cartesian product

\[
\text{dataset}
\times\text{trajectory}
\times\text{anchor configuration}
\times\text{noise model}
\times\text{fault scenario}
\times\text{estimator}
\times\text{integrity backend}
\times\text{seed}.
\]

每个 resolved run 生成唯一 config hash；重复 run 不覆盖旧结果。

## 86. Fault scenarios

### Snapshot

- no fault；
- single anchor step bias；
- magnitude sweep；
- near-threshold fault；
- anchor outage；
- anchor coordinate error；
- TDoA reference fault。

### Temporal

- persistent constant bias；
- ramp/slowly growing bias；
- intermittent NLOS；
- fault onset near marginalization boundary；
- current measurement healthy but prior contaminated；
- prior and current fault cancellation；
- two anchors simultaneously（后续）。

## 87. Accuracy metrics

- ATE；
- RPE；
- position RMSE/median/P95/max；
- velocity error；
- orientation geodesic error；
- bias error；
- convergence time；
- failure/reinitialization count。

## 88. Integrity metrics

### Per epoch

- position error `PE`；
- axis PL/HPL/VPL；
- `PL/PE` ratio（`PE≈0` 时单独处理）；
- detector statistic/threshold；
- maximizing hypothesis；
- max slope；
- availability state/reason；
- model validity。

### Aggregate

- empirical `P_FA` under H0；
- empirical `P_MD` by fault type/magnitude；
- coverage `P(PE≤PL)`；
- operational HMI `P(PE>AL, PL≤AL, detector pass)`；
- availability `P(PL≤AL, detector pass, model valid)`；
- continuity loss events；
- time-to-detect persistent fault；
- FDE correctness（future）。

## 89. Efficiency metrics

- update time；
- PL time；
- total latency；
- memory/RSS；
- active variables/factors；
- relinearized variables；
- number/dimension of hypotheses；
- sparse solve count；
- affected Bayes-tree cliques if accessible。

## 90. Paper-grade figures

1. trajectory + anchors + NLOS regions；
2. PE vs axis PL/HPL/VPL over time；
3. detector statistic vs threshold；
4. injected fault profile and detected interval；
5. max slope and maximizing anchor；
6. geometry condition vs PL；
7. nominal covariance term vs fault term；
8. snapshot vs graph/conditional PL；
9. UWB-only vs UWB/IMU accuracy；
10. UWB-only vs UWB/IMU integrity/availability；
11. PL vs anchor count；
12. PL vs window length；
13. global-detector P_D vs conditional-detector P_D；
14. PL vs fault persistence length；
15. historical sensitivity vs time since fault；
16. cancellation metric `||S^-1/2(A-HB)||` vs current error sensitivity；
17. runtime stacked breakdown；
18. runtime vs hypotheses/window/factors；
19. availability vs alert limit；
20. empirical HMI upper confidence bound vs requirement；
21. accuracy–integrity–runtime Pareto plot；
22. LOS residual Q-Q/tail overbound plot；
23. real-data anchor-wise residual/autocorrelation heatmap；
24. linearization predicted error vs nonlinear reoptimized error。

## 91. Required tables

- literature/claim matrix；
- dataset/trajectory/anchor summary；
- estimator accuracy；
- detector P_FA/P_MD；
- PL coverage/HMI/availability；
- runtime mean/median/P95/P99/max；
- ablation: no IMU/no history/global detector/conditional detector；
- ablation: Gaussian vs overbound；
- ablation: current-only vs persistent hypotheses；
- failure cases and assumption violations。

## 92. Calibration and evaluation split

必须分开：

- calibration set：UWB bias/noise/overbound；
- development set：调参数；
- test set：最终 coverage/HMI；
- stress set：domain shift/未知环境。

不能用同一轨迹拟合 residual distribution 再声称在该轨迹上获得 calibrated integrity。

---

# Part XV — Timing and Complexity Benchmark

## 93. Mandatory timing stages

每 epoch 至少记录：

1. UWB parsing/preprocessing；
2. UWB factor construction；
3. IMU preintegration；
4. incremental graph update；
5. nonlinear relinearization/solve if separable；
6. current-state query；
7. marginal covariance；
8. linearized snapshot extraction；
9. detector statistic；
10. fault-hypothesis generation/pruning；
11. fault-incidence construction；
12. slope/sensitivity solve；
13. PL composition/risk；
14. availability decision；
15. total algorithm latency。

I/O、CSV flush 和 plotting 不在核心 timed region 内。

## 94. Summary statistics

- count；
- mean；
- median；
- standard deviation；
- P90/P95/P99；
- max；
- warm/cold split；
- timeout/failure count。

横轴：

- trajectory length；
- number of anchors；
- window size；
- active factors/variables；
- hypothesis count；
- fault basis dimension；
- relinearized clique count；
- UWB group size。

## 95. Fair and reproducible benchmark protocol

1. Release build，记录 optimization flags；
2. 固定 CPU/线程数；
3. 记录 CPU、RAM、OS、compiler、GTSAM/Eigen；
4. warm-up 后再统计；
5. 同一数据、同一 seed、同一 estimator settings；
6. 避免日志 flush 和 visualization；
7. 每配置至少重复多次；
8. 报告 cold-start 与 steady-state；
9. 不把 batch oracle 时间混进 incremental baseline；
10. 对并行代码记录线程数和调度策略；
11. 检查 CPU frequency/turbo 状态，无法固定时明确报告；
12. 同时记录 problem size，避免只给毫秒没有规模。

## 96. Expected complexity trends

### Snapshot

状态维数小，主要随 anchor 数线性或低阶增长。single-anchor slopes 可在一次 WLS factorization 后批量计算。

### Incremental estimator

复杂度由图稀疏性、ordering、relinearization 和 affected cliques 决定，不仅由窗口长度决定。

### Graph PL

对 hypothesis `S` 的 fault basis 维数 `q_S`：

- 构造 `B=J^TA`：与受影响 factor rows 成正比；
- solve：`q_S` 个 RHS，可批处理；
- detector Gram：`q_S×q_S`；
- slope solve：小矩阵。

若 persistent fault 用 1–3 维 basis，可保持可控；若每历元独立 bias，复杂度与病态性都会迅速上升。

## 97. Timing acceptance targets

第一版不应先设不现实的绝对毫秒目标。建议用相对门槛：

- total mean < sensor update period；
- P99 < 2× update period 或明确队列策略；
- PL overhead < estimator update 的 20–30% 作为初期目标；
- 优化后再争取 <10%；
- 任何缓存优化都必须通过 numerical regression。

---
# Part XVI — Testing and Validation

## 98. Testing pyramid

### 98.1 Unit tests

- measurement models/Jacobians；
- whitening；
- TDoA differencing/covariance；
- risk allocation；
- chi-square/noncentral inversion；
- fault selector/basis；
- timestamp/frame conversions；
- result serialization。

### 98.2 Numerical oracle tests

小矩阵上比较：

- explicit SVD pseudoinverse；
- QR least squares；
- normal-equation sparse solve；
- explicit `P_⊥` vs `A^TA-B^TΛ^-1B`；
- dense covariance inverse vs GTSAM marginal；
- closed-form slope vs numerical maximization。

### 98.3 Property tests

随机生成 full-rank systems，验证：

- projector symmetry/idempotence；
- slope 对 fault amplitude 齐次；
- whitening coordinate invariance；
- permutation of measurement rows 不改变结果；
- adding an exactly redundant correctly modeled measurement does not worsen nominal covariance；
- unobservable protected fault gives infinite PL；
- TDoA reference change invariance（正确 covariance 下）。

### 98.4 Integration tests

- complete replay；
- snapshot estimator + PL；
- iSAM2 + integrity contract；
- factor marginalization and provenance；
- config matrix runner；
- deterministic seed/results；
- result-to-plot pipeline。

### 98.5 Regression tests

保存小型 gold datasets：

- expected state；
- expected PL components；
- expected detector statistic；
- expected timing only as broad guard, not exact；
- numerical tolerance tied to condition number。

## 99. Analytical sanity checks

1. zero fault：fault term不应凭空出现；
2. fault exactly in detector subspace：P_D 增加；
3. fault in state column space：detector不可见、PL趋于无穷；
4. more/better-distributed anchors：通常 nominal sigma 和 slope下降；
5. poor geometry：PL上升或不可用；
6. noise scale乘 `α`：在同类模型下 PL 近似线性缩放；
7. tighter risk requirement：quantile/λbar 增大，PL 不减；
8. larger P_FA budget：threshold降低，continuity变差但 detectability改善；
9. persistent fault cancellation：detector sensitivity 可下降而 state sensitivity仍高；
10. marginalization前后：compressed historical sensitivity 不应低估 full-window oracle。

## 100. Why “coverage rate = 100%” is not a probabilistic integrity proof

设 `N` 个**独立**试验中零次失败。失效率 `p` 的单侧 95% 上界近似“rule of three”：

\[
p_{95}\approx\frac{3}{N}.
\]

因此：

- 要支持 `p<10^{-3}`，需约 `3×10^3` 个独立试验；
- `p<10^{-5}`，需约 `3×10^5`；
- `p<10^{-7}`，需约 `3×10^7`。

轨迹历元高度相关，实际有效独立样本数远小于 epoch 数。故一条或几十条轨迹上的 100% coverage 只能说明“在这些样本中没有观察到失包络”，不能证明航空级风险。

## 101. Recommended integrity validation stack

1. **解析验证：** linear Gaussian toy model；
2. **普通 Monte Carlo：** 中等概率 P_FA/P_MD；
3. **fault magnitude sweep：** 对 detector operating curve；
4. **rare-event simulation：** importance sampling/subset simulation；
5. **real-data replay：** model mismatch；
6. **stress/domain shift：** 未校准环境；
7. **confidence reporting：** Clopper–Pearson/one-sided bound；
8. **effective sample size：** 处理 temporal correlation；
9. **negative result logging：** 不隐藏 unavailable 和 infinite PL。

## 102. Robust kernels and formal integrity

### Allowed engineering use

- 防止 estimator 数值发散；
- 做 robustness baseline；
- 作为 FDE candidate generator；
- 提供 quality features。

### Not allowed inference

- “使用 Huber 后 residual 更小，因此 PL 有保证”；
- “switch variable 自动完成 risk allocation”；
- “M-estimator covariance 自动覆盖 faults”。

若 formal detector 使用 robust-weighted residual，必须重新推导/校准 statistic distribution。推荐第一版：

- estimator 可以有一个 robust variant 用于 accuracy 对比；
- formal RAIM baseline 使用明确 Gaussian WLS linearization；
- 两者输出不能混标。

---

# Part XVII — Publication Strategy

## 103. Publication sequence

### Stage A — Baseline/内部里程碑

**内容：** snapshot UWB slope PL + tightly coupled iSAM2 + multiple trajectories + timing。  
**定位：** 导师展示、technical report、开源 baseline。  
**不建议：** 把它包装成主要 journal novelty。

### Stage B — 第一篇主论文

**题目方向：** conditional current-state integrity for incremental UWB/IMU under persistent anchor faults。

**必须有：**

- 清楚的 prior art differentiation；
- conditional detector dilution analysis；
- historical/persistent fault slope；
- sparse implementation；
- real data + simulation；
- risk/coverage semantics；
- runtime。

**适合 venue：** ION ITM / ION GNSS+ / IEEE-ION PLANS；成熟后 NAVIGATION、RA-L 或 T-AES。

### Stage C — 第二篇方法论文

**内容：** provenance-preserving marginalization + Bayes-tree/local sparse update + hypothesis management。

**关键对比：**

- per-hypothesis full reoptimization；
- solution separation/rank update；
- static full-window oracle；
- 2026 incremental ARAIM work。

### Stage D — 误差模型/系统论文

**内容：** LOS overbound + NLOS fault prior + temporal correlation + PL-aware anchor selection。

## 104. Claim wording for a safe paper

推荐：

> “To the best of our literature search, existing UWB integrity work has not jointly addressed current-state protection under historical persistent anchor faults, conditional UWB detection after smoothing-based prior compression, and preservation of fault sensitivity through fixed-lag marginalization.”

不要写：

> “This is the first integrity monitoring system for UWB/IMU factor graphs.”

## 105. Minimum evidence for a strong submission

- at least one public/real UWB-IMU dataset or released own dataset subset；
- multiple trajectories and anchor geometries；
- statistical detector calibration；
- analytical/dense oracle validation；
- full runtime distribution；
- code/config release or enough implementation detail；
- failure cases；
- explicit publication-status table for close prior art；
- comparison with snapshot RAIM、global graph residual、conditional detector、solution-separation oracle if feasible。

## 106. Likely reviewer questions

1. “Bhamidipati already did failure-slope Graph-SLAM; what is new?”
2. “Tanil/Arana already handled historical faults; why is this not a sensor substitution?”
3. “Hafez already did fixed-lag integrity; why not solution separation?”
4. “Hu et al. already use incremental ARAIM/rank updates; what is different?”
5. “How are UWB fault probabilities calibrated?”
6. “Why is LOS Gaussian?”
7. “Does robust weighting invalidate chi-square?”
8. “What happens after marginalization?”
9. “Can persistent bias be completely absorbed and undetected?”
10. “How do you validate a 10^-5 or lower risk with finite data?”
11. “Is 100% coverage merely conservative?”
12. “Does the method remain real-time as hypotheses grow?”

本报告推荐的主线正是围绕这些问题设计。

---

# Part XVIII — Risks, Failure Modes, and Open Questions

## 107. Mathematical risks

### R-M1 — Fault subspace not detectable

`A^TP_⊥A` 或 conditional `D^TS^-1D` 奇异；应返回 infinite PL。

### R-M2 — Linearization invalid

故障导致解离开 local chart；first-order slope 低估真实 error。

### R-M3 — Prior double counting

conditional detector 的 prior 已包含当前 UWB，导致 innovation covariance 错误和过度自信。

### R-M4 — Marginalization loses fault history

旧 factor 被消除后无法表示 historical fault；产生虚假有限 PL。

### R-M5 — Robust/data-dependent weights

统计量不再服从预设 chi-square。

### R-M6 — Incorrect TDoA covariance

差分相关被忽略，阈值和 PL 失真。

### R-M7 — Risk budget not exhaustive

未监测多故障/模型失效未计入 `P_NM/P_model`。

## 108. UWB physical-model risks

- antenna-delay drift；
- received-power-dependent bias；
- anchor clock/synchronization fault；
- reference-anchor TDoA common fault；
- anchor coordinate/map error；
- moving/loose anchor；
- channel switching/packet association error；
- NLOS bias persistence and region dependence；
- inter-anchor/common-environment correlation；
- packet loss not missing at random；
- tag/IMU lever-arm uncertainty。

## 109. Engineering risks

- GTSAM version-specific internal APIs；
- stale factorization handles；
- factor ID/row mapping changes after reordering；
- timing contaminated by logging；
- non-deterministic multi-threading；
- result schema drift；
- unit/frame mismatch；
- numerical regularization hiding unobservability；
- huge YAML Cartesian product without run registry；
- plotting from manually edited CSV。

## 110. Open research questions

1. 边缘化后最小充分的 historical-fault integrity summary 是什么？
2. conditional detector 与 full graph parity detector 在何种条件下等价？
3. persistent fault basis 选得过低维会漏掉什么风险？
4. 如何对 NLOS onset/duration prior 做 conservative calibration？
5. 如何把 learned LOS/NLOS score 转换为有置信保证的 prior/likelihood？
6. robust estimator 与 formal detector 如何并行使用而不双重计数？
7. nonlinear PL 是采用 remainder bound、iterative self-consistency 还是 solution-separation oracle？
8. UWB anchor selection 如何同时优化 availability、continuity 和通信负担？
9. 当前状态 position PL 是否需要同时保护 orientation/bias，避免间接危险？
10. 对低风险 HMI，最有效的 rare-event sampler 是什么？

---

# Part XIX — Recommended Reading Order

## 111. Reading sequence

### Phase 1 — Integrity semantics and classical math

1. Parkinson & Axelrad (1988), residual RAIM；
2. Joerger, Chan & Pervan (2014), RB vs SS；
3. Blanch et al. (2015), baseline ARAIM；
4. Bruvik et al. (2026), nonlinear PnP slope PL。

### Phase 2 — Sequential/history integrity

5. Tanil et al. (2018), innovation-sequence failure slopes；
6. Arana et al. (2020), time-window KF localization integrity；
7. Meng & Hsu (2021), KF solution separation；
8. Lee et al. (2023), sequential IMU fault PL。

### Phase 3 — Graph/fixed-lag integrity

9. Bhamidipati & Gao (2019), distributed GPS/UWB Graph-SLAM failure slope；
10. Bhamidipati & Gao (2020), Graph-SLAM PL；
11. Hafez et al. (2020), fixed-lag solution separation；
12. Xia et al. (2024), integrity-constrained FGO；
13. Hu et al. (2026 abstract), incremental ARAIM/rank update—投稿前必须跟踪全文。

### Phase 4 — UWB estimators and fault models

14. Ascher et al. (2011), UWB/INS IM；
15. Kang et al. (2020), incremental UWB/IMU smoothing；
16. Song & Hsu (2021), tightly coupled UWB/INS FGO；
17. Li et al. (2023), pure-UWB MHSS integrity；
18. Shalaby et al. (2023), TWR calibration；
19. DB-SMF-UWB (2025), deterministic UWB uncertainty。

### Phase 5 — Deterministic/remainder/error overbounding

20. Calafiore (2005), bounded nonlinear remainder；
21. Rohou & Jaulin (2023), guaranteed dynamic interval localization；
22. Zhu et al. (2026 preprint), on-manifold deterministic LIO PL；
23. Langel et al. (2021), uncertain Gauss-Markov overbound；
24. Jada et al. (2025), time-correlation modeling；
25. Gallon et al. (2026), nonstationary noise overbounding。

## 112. How to take notes

每篇只回答八个问题：

1. protected state 是什么？
2. HMI event 如何定义？
3. nominal noise 假设是什么？
4. fault hypothesis 是什么？
5. detector statistic/threshold 是什么？
6. PL 如何由 detector 和 fault sensitivity 得到？
7. temporal history/marginalization 如何处理？
8. 哪个 claim 已占据、哪个 assumption 未闭合？

---

# Part XX — Immediate 2–4 Week Action Plan

## 113. Week 1 — Pure UWB and infrastructure

### Day 1–2

- repository/CMake/config/result schema；
- frame/time/unit conventions；
- synthetic trajectory + anchor generator；
- scoped timing；
- run manifest。

### Day 3–5

- 3D TWR snapshot WLS/NLS；
- whitening/Jacobian/rank；
- straight/circle/figure-eight；
- dense numerical oracle；
- first plots。

**Friday deliverable：** trajectory + anchors、PE、condition number、runtime。

## 114. Week 2 — Snapshot RAIM baseline

### Day 6–7

- residual chi-square；
- `P_FA` calibration；
- noncentral Monte Carlo。

### Day 8–10

- single-anchor fault hypotheses；
- worst-case slope；
- `λbar` inversion；
- axis PL/HPL；
- geometry/noise/fault sweeps。

**Week-2 deliverable：** PE vs PL、detector vs threshold、slope vs geometry、P_FA/P_MD table。此时已经有一个可以给导师展示的数学 baseline。

## 115. Week 3 — Incremental UWB/IMU estimator

### Day 11–12

- UWB-only incremental bridge；
- batch vs iSAM2 test；
- estimator-integrity snapshot contract。

### Day 13–15

- IMU preintegration tests；
- tightly coupled state/factors；
- initialization/synchronization；
- current marginal covariance；
- update timing。

**Week-3 deliverable：** UWB-only vs UWB/IMU trajectories、accuracy、marginals、timing。

## 116. Week 4 — Fusion detector and complete advisor demo

### Day 16–18

- pre-current-UWB prior；
- conditional UWB innovation detector；
- global detector comparison；
- window-length dilution experiment。

### Day 19–20

- multiple trajectories batch runner；
- runtime P50/P95/P99；
- consolidated report；
- list of next research modules。

**Week-4 deliverable：**

1. multiple trajectories；
2. pure UWB snapshot slope PL；
3. UWB/IMU incremental smoothing；
4. conditional detector baseline；
5. timing statistics；
6. clear roadmap to graph persistent-fault PL。

## 117. What to implement immediately, what to leave as interface

### Must implement now

- TWR snapshot WLS；
- correct whitening/rank；
- residual chi-square；
- single-anchor slope PL；
- Monte Carlo；
- IMU preintegration；
- UWB/IMU iSAM2；
- marginal covariance；
- conditional UWB detector；
- multiple trajectories；
- detailed timing/logging。

### Interface now, implementation later

- multiple simultaneous faults；
- persistent temporal basis；
- FDE；
- solution separation；
- heavy-tail overbound；
- adaptive priors；
- anchor selection；
- nonlinear remainder；
- provenance-preserving marginalization；
- TDoA；
- anchor-map uncertainty。

### Do not spend time on now

- custom Bayes-tree implementation；
- GPU acceleration；
- full ARAIM with constellation-like UWB modes；
- deterministic SMF integration；
- sophisticated learned NLOS network；
- ROS online visualization stack；
- multi-robot UWB；
- multiple fault FDE；
- end-to-end rare-event certification。

## 118. The direct answer

**从明天开始，最快得到正确 baseline 的顺序：**

`replay/config/profiling → snapshot WLS → chi-square → single-anchor slope PL → Monte Carlo → IMU preintegration → UWB/IMU iSAM2 → marginal/linearized snapshot contract → conditional UWB detector → multiple trajectories + timing`。

**最值得升级成论文贡献的模块：**

1. historical/persistent physical-anchor fault 对 current state 的 PL；
2. conditional UWB detector 与 prior/current fault cancellation；
3. fault-provenance-preserving marginalization + sparse online slope；
4. LOS overbound + NLOS explicit fault；
5. nonlinear remainder/validity gate。

---

# Appendix A — Configuration and Result Conventions

## A.1 Configuration hierarchy

```text
base defaults
  ├── dataset config
  ├── sensor/calibration config
  ├── anchor-map config
  ├── estimator config
  ├── integrity config
  ├── fault-scenario config
  └── study/runner config
```

Resolution rules：

- later layer overrides earlier layer；
- resolved config is saved verbatim；
- unknown fields are errors；
- all units are explicit in field names or schema；
- seed has no implicit default in Monte Carlo mode；
- risk probabilities use scientific notation and range validation。

## A.2 Recommended result hierarchy

```text
results/
└── <study_name>/
    └── <UTC-date>_<git-short-sha>/
        └── <dataset>_<sequence>/
            └── <config-hash>/
                └── seed_<seed>/
                    ├── manifest.json
                    ├── resolved_config.yaml
                    ├── estimator_states.csv
                    ├── residuals.csv
                    ├── integrity.csv
                    ├── timing.csv
                    ├── events.jsonl
                    ├── summary.json
                    └── plots/
```

## A.3 Experiment naming

`<dataset>-<sequence>__<estimator>__<integrity>__<fault>__s<seed>__<hash>`

不要把所有参数塞进文件名；完整参数在 manifest 中。

## A.4 CSV/JSON schemas

### `integrity.csv` minimum columns

- epoch/time；
- state key；
- PE_x/y/z if GT；
- PL_x/y/z；
- HPL/VPL；
- nominal/fault/model component；
- detector statistic/threshold/DOF；
- detector pass；
- maximizing hypothesis；
- max slope；
- available；
- alert reason；
- model validity；
- hypothesis count；
- PL compute time。

### `timing.csv`

Long format：一行一个 stage，便于 groupby 和 percentile。

### `events.jsonl`

记录：

- anchor outage；
- fault injection；
- detector alert；
- marginalization；
- reinitialization；
- rank failure；
- model-domain violation；
- FDE request。

## A.5 Plotting pipeline

1. raw logs are immutable；
2. Python reads manifests and schemas；
3. aggregation produces versioned summary tables；
4. figures cite run IDs/config hashes；
5. no manual spreadsheet edits；
6. paper figures generated by one script/study config；
7. plotting code never recomputes estimator/PL logic。

---

# Appendix B — Claim and Assumption Checklist

投稿前逐项回答：

- [ ] 是否明确区分 deterministic 与 probabilistic PL？
- [ ] 是否定义 HMI、AL、availability、continuity？
- [ ] nominal error 是否被 conservative overbound？
- [ ] fault priors 和 unmonitored risk 是否公开？
- [ ] detector 与 PL 是否使用同一个 fault model？
- [ ] robust weights 是否改变 detector distribution？
- [ ] TDoA covariance 是否完整？
- [ ] gauge/rank failure 是否给 infinite PL？
- [ ] linearization error 是否有界或 gate？
- [ ] marginalization 是否保留 historical fault sensitivity？
- [ ] persistent hypothesis 是否低维且物理合理？
- [ ] 100% coverage 是否附带置信上界？
- [ ] runtime 是否包含 P95/P99 和 problem size？
- [ ] 是否和 Bhamidipati、Hafez、Tanil、Arana、Hu 2026 做了直接区分？
- [ ] 是否使用“目前检索中未发现”而非“世界首创”？

---

# Appendix C — Curated Bibliography and Publication Status

> DOI/official links are supplied where confidently identified. “Abstract only” means the present report did not have a full paper to verify detailed claims.

1. Parkinson, B. W.; Axelrad, P. “Autonomous GPS Integrity Monitoring Using the Pseudorange Residual.” *NAVIGATION*, 1988. Journal.
2. Brown, R. G. “A Baseline GPS RAIM Scheme and a Note on the Equivalence of Three RAIM Methods.” *NAVIGATION*, 1992. Journal.
3. Joerger, M.; Chan, F.-C.; Pervan, B. “Solution Separation Versus Residual-Based RAIM.” *NAVIGATION*, 61(4), 273–291, 2014. Journal.
4. Blanch, J. et al. “Baseline Advanced RAIM User Algorithm and Possible Improvements.” *IEEE TAES*, 51(1), 713–732, 2015. Journal.
5. Tanil, C.; Khanafseh, S.; Joerger, M.; Pervan, B. “Sequential Integrity Monitoring for Kalman Filter Innovations-Based Detectors.” ION GNSS+ 2018. DOI: https://doi.org/10.33012/2018.15975.
6. Arana, G. D. et al. “Integrity Monitoring for Kalman Filter-Based Localization.” *IJRR*, 2020. DOI: https://doi.org/10.1177/0278364920960517.
7. Meng, Q.; Hsu, L.-T. “Integrity Monitoring for All-Source Navigation Enhanced by Kalman Filter-Based Solution Separation.” *IEEE Sensors Journal*, 2021. DOI: https://doi.org/10.1109/JSEN.2020.3026081.
8. Lee, J. et al. “Navigation Safety Assurance of a KF-Based GNSS/IMU System: Protection Levels Against IMU Failure.” *NAVIGATION*, 2023. DOI: https://doi.org/10.33012/navi.612.
9. Ascher, C.; Zwirello, L.; Zwick, T.; Trommer, G. “Integrity Monitoring for UWB/INS Tightly Coupled Pedestrian Indoor Scenarios.” IPIN 2011. DOI: https://doi.org/10.1109/IPIN.2011.6071948.
10. Li, J.; Sun, Y.; Deng, Z. “Integrity Monitoring for Extreme Indoor Environment Positioning Based on Ultrawideband.” ION ITM 2023, pp. 535–547. DOI: https://doi.org/10.33012/2023.18610.
11. Salimpour, S. et al. “Exploiting Redundancy for UWB Anomaly Detection in Infrastructure-Free Multi-Robot Relative Localization.” *Frontiers in Robotics and AI*, 2023. DOI: https://doi.org/10.3389/frobt.2023.1190296.
12. Zhou, B.; Zhu, Y.; Rui, C.; Luo, J.; Pan, Y. “Safety-Critical Ultra-Wideband 3D Localization With Set-Membership Uncertainty Representation.” *IEEE RA-L*, 10(9), 8826–8833, 2025. DOI: https://doi.org/10.1109/LRA.2025.3589806. GitHub: https://github.com/Zhu-YQ/DB-SMF-UWB.
13. Kang, J. et al. “Ultra-Wideband Aided UAV Positioning Using Incremental Smoothing with Ranges and Multilateration.” IROS 2020. DOI: https://doi.org/10.1109/IROS45743.2020.9341439.
14. Song, Y.; Hsu, L.-T. “Tightly Coupled Integrated Navigation System via Factor Graph for UAV Indoor Localization.” *Aerospace Science and Technology*, 108, 106370, 2021. DOI: https://doi.org/10.1016/j.ast.2020.106370.
15. Fang, X.; Wang, C.; Nguyen, T.-M.; Xie, L. “Graph Optimization Approach to Range-Based Localization.” *IEEE TSMC: Systems*, 51(11), 6830–6841, 2021. DOI: https://doi.org/10.1109/TSMC.2020.2964713.
16. Fan, G. et al. “RFG-TVIU: Robust Factor Graph for Tightly Coupled Vision/IMU/UWB.” *Frontiers in Neurorobotics*, 2024. DOI: https://doi.org/10.3389/fnbot.2024.1343644.
17. Bhamidipati, S.; Gao, G. X. “Distributed Cooperative SLAM-Based Integrity Monitoring Via a Network of Receivers.” ION GNSS+ 2019, pp. 2023–2034. DOI: https://doi.org/10.33012/2019.16882.
18. Bhamidipati, S.; Gao, G. X. “Integrity Monitoring of Graph-SLAM Using GPS and Fish-Eye Camera.” *NAVIGATION*, 67(3), 583–600, 2020. DOI: https://doi.org/10.1002/navi.381.
19. Hafez, O. A.; Arana, G. D.; Joerger, M.; Spenko, M. “Quantifying Robot Localization Safety: A New Integrity Monitoring Method for Fixed-Lag Smoothing.” *IEEE RA-L*, 5(2), 3182–3189, 2020. DOI: https://doi.org/10.1109/LRA.2020.2975769.
20. Hafez, O. A. et al. “On Robot Localization Safety for Fixed-Lag Smoothing: Quantifying the Risk of Misassociation.” IEEE/ION PLANS 2020. DOI: https://doi.org/10.1109/PLANS46316.2020.9110126.
21. Meng, F.; Sun, Y.; Deng, Z. “A Novel Integrity Monitoring Algorithm for FGO-Based GNSS Positioning System.” ION ITM 2023. DOI: https://doi.org/10.33012/2023.18609.
22. Xia, X.; Wen, W.; Hsu, L.-T. “Integrity-Constrained Factor Graph Optimization for GNSS Positioning in Urban Canyons.” *NAVIGATION*, 71(3), 2024. DOI: https://doi.org/10.33012/navi.660.
23. Maharmeh, E.; Alsayed, Z.; Nashashibi, F. “PL-RAS: A Robust Localization System with Real Time Protection Level Calculation and Adaptive Kernel for Enhanced Integrity.” IEEE IV 2025. DOI: https://doi.org/10.1109/IV64158.2025.11097747.
24. Maharmeh, E.; Alsayed, Z.; Nashashibi, F. “PL-RAS++: Real-Time Integrity Assurance for Robust Localization via Risk-Adaptive Protection Level.” IEEE ITSC 2025. DOI: https://doi.org/10.1109/ITSC60802.2025.11423212.
25. Bruvik, O. B. et al. “Protection Levels for Vision-Based Pose Estimation.” arXiv:2608.10023, 2026; accepted for DASC 2026 according to the manuscript. https://arxiv.org/abs/2608.10023.
26. Hu, J.; Wang, X.; Wen, W. “Extending ARAIM to Incremental Smoothing-Based GNSS/IMU Fusion: Efficient Protection Level Computation via Rank Updates.” ION GNSS+ 2026 scheduled presentation, **public abstract only as of 2026-08-31**. Official abstract: https://www.ion.org/gnss/abstracts.cfm?paperID=16840.
27. Calafiore, G. “Reliable Localization Using Set-Valued Nonlinear Filters.” *IEEE TSMC-A*, 35(2), 189–197, 2005. DOI: https://doi.org/10.1109/TSMCA.2005.843383.
28. Rohou, S.; Jaulin, L. “Brunovsky Decomposition for Dynamic Interval Localization.” *IEEE TAC*, 68(11), 6937–6943, 2023. DOI: https://doi.org/10.1109/TAC.2023.3246943.
29. Zhu, Y. et al. “Safety-Critical LiDAR-Inertial Odometry with On-Manifold Deterministic Protection Level.” arXiv:2605.09383v2, 2026. Preprint in the supplied manuscript.
30. Langel, S. et al. “Overbounding the Effect of Uncertain Gauss-Markov Noise in Kalman Filtering.” *NAVIGATION*, 68(2), 259–276, 2021. DOI: https://doi.org/10.1002/navi.419.
31. Jada, S. et al. “Measurement Error Time-Correlation Modeling for Safety-Critical Navigation.” *NAVIGATION*, 72(4), 2025. Official article identifier: navi.721.
32. Gallon, E. et al. “High-Integrity Modeling of Nonstationary Noise Processes for Safety-Critical Navigation.” *NAVIGATION*, 73(1), 2026. Official article identifier: navi.729.
33. Shalaby, M. A.; Cossette, C. C.; Forbes, J. R.; Le Ny, J. “Calibration and Uncertainty Characterization for Ultra-Wideband Two-Way-Ranging Measurements.” ICRA 2023. DOI: https://doi.org/10.1109/ICRA48891.2023.10160769.
34. Maharmeh, E.; Alsayed, Z.; Nashashibi, F. “A Comprehensive Survey on the Integrity of Localization Systems.” *Sensors*, 2025. Survey; useful for landscape, but its broad PL terminology should not replace aviation RAIM semantics.
35. Kaess, M. et al. “iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree.” *IJRR*, 2012. Foundational incremental optimization reference.

---

## Final project decision

**DB-SMF-UWB does not eliminate the research space.** It eliminates the weak claim “UWB + deterministic protection set is new.” The strongest remaining path is not a generic UWB/IMU factor graph and not a direct `H→J` substitution. It is a formally defined, computationally efficient integrity framework in which:

- the protected quantity is the **current state**；
- faults may originate in **historical UWB factors**；
- physical anchor faults can be **persistent and temporally structured**；
- history+IMU are used as a **conditional prior** rather than allowed to dilute the detector；
- marginalization preserves **fault provenance**；
- PL uses a defensible **nominal overbound + explicit fault model + risk allocation**；
- all results include **availability and timing**, not only accuracy and 100% sample coverage。


# Part XXI — GTSAM 落地补强(Engineering Reinforcement)

> 本部分补全审阅中标记的三处最高优先级工程缺口。它们都位于"数学正确但落地会静默破坏假设"的区域,因此必须写进 estimator↔integrity contract,而不是留给实现者临场决定。三节分别对应:(§120)frozen linearization 与 iSAM2 relinearization 的自洽;(§121)Method A 双更新 + `marginalCovariance` 的实时代价与 Method B 提前验证;(§122)UWB-only bridge 中 smoothness factor 对 detector DOF 的语义。§123 给出可直接落地的 `EstimationSnapshot` C++ 接口草案。

## 119. 为什么这三处必须进 contract

§38.1 要求 estimator 与 integrity 使用"完全相同的 factor graph、ordering、噪声模型和线性化点"。iSAM2 的三个默认行为会在不报错的情况下破坏这个前提:

- **选择性重线性化**(wildfire relinearization)使 Bayes tree 中缓存的 `GaussianFactor` 分布在**不同的线性化点**,而非当前估计点;
- **conditional prior 的提取**若用两次完整 update 实现,其代价与时序耦合可能超过 detector 本身;
- **UWB-only bridge 的 smoothness/motion factor** 会给信息矩阵加行,同时改变 detector 的 DOF 与 nominal 协方差,而其"可信度"从未被显式声明。

这三点都不会抛异常,只会让 §26 白化、§28 chi-square DOF、§32 nominal 协方差**静默地基于不自洽的量**。因此它们必须成为契约级约束。

## 120. 自洽 relinearization 策略

### 120.1 问题

iSAM2 内部的 `ISAM2::calculateEstimate()` 返回当前非线性估计,但 Bayes tree 各 clique 的 `GaussianConditional` 是在**各自上次被 relinearize 时的点**上生成的。若 integrity monitor 直接复用这些缓存因子拼装 \(J\) 与白化残差,则:

$$
b = J\,\delta X + v + A_S d_S
$$

中的 \(J\)、\(b\) 可能对应**多个互不相同的展开点**,违反 §26 的"同一点白化"前提,使 slope 的物理意义失效。

### 120.2 政策

**PL 计算不复用 iSAM2 缓存的线性因子。** 契约要求:integrity monitor 拿到一个"上报的当前估计点" \(\hat X^\star\)(通常即 `calculateEstimate()`),然后对**受 hypothesis 影响的子图 + 被保护状态所在的相关 clique**,在 \(\hat X^\star\) 处**统一重新** `linearize()` 一次,得到自洽的 \((J,b,A_S)\)。

形式化约束:

- integrity 消费的所有 factor row 必须共享同一 `linearization_version`;
- `linearization_version` 与 \(\hat X^\star\)、`graph_version`、`ordering_version`、`noise_model_version` 一一绑定(§42 缓存键的子集);
- 若某 factor 无法在 \(\hat X^\star\) 处重线性化(例如已被 marginalize,仅剩 Schur prior),则走 §43 的 provenance 通道 \((\bar B_r,\bar C)\),而不是复用旧线性点。

### 120.3 三种一致性级别

| 级别 | 含义 | 何时可接受 |
|---|---|---|
| **L-Strict** | 对整个相关子图在 \(\hat X^\star\) 处重线性化,ordering 冻结 | M8 之前的数学验证、oracle、论文关键图 |
| **L-Cached-Checked** | 复用 iSAM2 因子,但校验每个 factor 的缓存线性点与 \(\hat X^\star\) 的偏差 \(\|\hat X^\star \ominus X_{lin,i}\|\) 低于阈值,否则强制重线性化 | steady-state 在线,追求速度且已验证偏差可控 |
| **L-Cached-Blind** | 直接复用缓存,不校验 | **禁止**用于 formal PL,仅可用于 debug |

**Gate:** 任何 formal PL 输出必须声明其一致性级别;L-Cached-Checked 的偏差阈值必须通过 §98.5 的数值回归与 L-Strict oracle 对齐后才允许启用。

### 120.4 伪代码

```text
function buildConsistentLinearView(isam2, X_star, hypothesis_S, protected_keys):
    ordering        = isam2.getOrdering()                 # 冻结
    affected_factors = graph.factorsTouching(
                          hypothesis_S.affected_keys ∪ protected_keys)

    linear_rows = []
    for f in affected_factors:
        if f.isMarginalPrior():                            # 已被消元
            attachSchurProvenance(f, hypothesis_S)         # 用 (B̄_r, C̄), §43
            continue
        # 关键:统一在 X_star 处展开,不复用缓存
        gf = f.linearize(X_star)                           # GTSAM linearize()
        (Hraw, braw) = gf.jacobian()
        W  = whitener(f.noiseModel())                      # L^{-1}, §26
        linear_rows.append( row(W*Hraw, W*braw,
                                 W*faultIncidence(f, hypothesis_S),
                                 factor_id = f.id,
                                 lin_version = X_star.version) )

    assert allSame(r.lin_version for r in linear_rows)     # 自洽性断言
    return ImmutableLinearView(linear_rows, ordering,
                               lin_version = X_star.version)
```

契约层新增能力:`supportsConsistentRelinearization()`。缺失时,integrity backend 必须显式降级为 L-Cached-Checked 并在 `ProtectionLevelResult` 标注,不得静默使用 L-Cached-Blind。

## 121. Method A 双更新代价评估与 Method B 提前验证

### 121.1 Method A 的真实开销

§51 Method A(先加 IMU/history 求 prior,detector 通过后再加 UWB)每 epoch 的成本近似:

$$
\text{cost}_A \approx
\underbrace{c_{upd}^{(\text{no-UWB})}}_{\text{第一次 update}}
+\underbrace{c_{marg}(\text{current key})}_{\text{prior query}}
+\underbrace{c_{upd}^{(\text{+UWB})}}_{\text{第二次 update}}.
$$

关键点:

- **两次 `ISAM2::update()`**——第二次通常较轻(只加 UWB factors),但第一次可能触发与正式 update 相当的 relinearization/backsubstitution;最坏情况接近正式 update 的两倍。
- **`P_k^-` 只需当前状态的 marginal**——用 `ISAM2::marginalCovariance(currentKey)` 或对 current-state basis 做多右端 triangular solve(§40.4),**绝不整图求逆**。查联合 marginal(多键)比单键贵约一个数量级,必须在 §93 timing stage 里单列区分。

### 121.2 代价预算表

| 阶段 | 期望复杂度 | 是否可复用 factorization | timing stage(§93) |
|---|---|---|---|
| 第一次 update(no-UWB) | 与受影响 clique 数成正比 | 否(新 IMU/prior 因子) | stage 4 变体 |
| current-state prior query \(P_k^-,\bar x_k^-\) | 单键 marginal ≈ \(O(\text{clique path})\) | 是(复用当前 Bayes tree) | stage 7(单列 single-key) |
| innovation \(\nu_k,S_k\) 构造 | \(O(m_u \cdot \dim x)\) | — | stage 9 |
| 第二次 update(+UWB) | 通常小于第一次 | 部分 | stage 4 |

**验收目标**(承接 §97):Method A 的双 update 总开销不得使 PL overhead 超过 estimator 单次 update 的 20–30%;若超出,必须转 Method B。

### 121.3 Method B 提前验证(不再"后续再说")

审阅结论:**Method B 应从 §51 的"后续优化"提前到 M8 阶段做可行性验证**,否则 M8 demo 的 timing 会被 Method A 拖累。

Method B 思路:从 all-in graph 出发,用 factor removal / rank downdate 得到"排除当前 UWB batch"的等效 prior,而非重跑第一次 update。

$$
\Lambda^{-} = \Lambda^{\text{all-in}} - H_{u,k}^{T} R_{u,k}^{-1} H_{u,k},
\qquad
\eta^{-} = \eta^{\text{all-in}} - H_{u,k}^{T} R_{u,k}^{-1} z_{u,k}.
$$

当前 UWB batch 维数 \(m_u\) 小,这是一个低秩 downdate,适用 Sherman–Morrison/Woodbury(§41 表):

$$
P_k^{-} = P_k^{\text{all-in}}
+ P_k^{\text{all-in}} H_{u,k}^{T}
\big(R_{u,k} - H_{u,k}P_k^{\text{all-in}}H_{u,k}^{T}\big)^{-1}
H_{u,k} P_k^{\text{all-in}}.
$$

**风险与验证**(§41 已警告 downdate 稳定性):

- 中括号内 \(R_{u,k} - H_{u,k}P_k^{\text{all-in}}H_{u,k}^{T}\) 必须保持正定;当当前 UWB batch 对该状态贡献接近饱和时会病态,须监测条件数。
- Method B 必须与 Method A 在小图上做**数值等价回归**(§98.2),相对误差低于 condition-number-aware 容差方可上线。
- 若 downdate 病态或涉及非当前 UWB 的 factor 删除,回退 Method A。

**决策 gate:** M8 同时实现 Method A(语义基准)与 Method B(速度候选),以 Method A 为 oracle 验证 Method B;在线默认用通过验证的 Method B,失败时自动回退 Method A 并记 event。

## 122. UWB-only bridge 中 smoothness factor 的 detector DOF 语义

### 122.1 问题

§75 的 UWB-only bridge 用 position/velocity + 简单 motion/smoothness factor 桥接 snapshot 与融合。该 factor 给信息矩阵加行、给 \(J\) 加行,因此**同时改变**:

- detector 的残差 DOF \(\nu\)(§28);
- nominal 协方差 \(\Sigma_p=(H^TH)^{-1}\)(§32)。

但它的"可信度"从未声明。这直接决定 chi-square 分布是否成立。

### 122.2 两种语义,必须二选一并声明

| 语义 | smoothness factor 角色 | 对 detector 的处理 | DOF 影响 | 风险 |
|---|---|---|---|---|
| **S-Trusted**(可信运动模型) | 视为真实先验信息,进 H0 模型 | 其残差行计入 \(T=r^Tr\),计入 \(\nu\) | \(\nu\) 增加对应行数 | 若运动模型与真值不符,产生**模型故障**,须进 §107 R-M 风险,且 nominal residual 会偏离 chi-square |
| **S-Regularizer**(软正则) | 仅为数值稳定/gauge 固定 | **不计入** detector statistic;仅用于求解 | detector \(\nu\) 不含这些行 | 有 §39 的"用 tiny prior 把无穷 PL 压成有限"风险,须在 §39 政策下检查 protected subspace 可观性 |

### 122.3 政策

1. bridge 的 smoothness/motion factor 默认标记为 **S-Regularizer**,因为它只是过渡产物,不是被验证过的物理运动模型。
2. detector 的 DOF 计算必须显式排除 S-Regularizer 行:

$$
\nu = n_{\text{measurement rows}} - \operatorname{rank}(H),
\qquad
n_{\text{measurement rows}} \; \text{不含 S-Regularizer 行}.
$$

3. 若要将其升级为 S-Trusted,必须先通过 §71/§98.3 的 nominal residual 一致性验证(标准化残差中心/方差/尾部),并进入 §34 的 overbound 流程。
4. 无论哪种语义,`ResidualBlock`(§62)与 `EstimationSnapshot`(§123)必须携带每行的 `RowRole ∈ {Measurement, TrustedPrior, Regularizer}`,detector 据此决定是否计入 statistic 与 DOF。

**Gate(承接 §75):** bridge 上线前,S-Regularizer 配置下的 H0 false-alarm rate 必须与仅 UWB measurement 行的理论 \(\chi^2_\nu\) 在统计不确定度内一致(§72 gate 的扩展)。

## 123. `EstimationSnapshot` C++ 接口草案

> 该草案落地 §66/§67 的抽象契约,并把 §120–§122 的三条政策变成编译期/运行期可检查的接口。它不暴露可变 GTSAM graph,只暴露 capability + 只读线性视图 + 一致重线性化 + prior 提取。PL calculator 不直接依赖 GTSAM(§68)。

```cpp
// ============================================================
// uwb_imu_pl/integrity/contracts/EstimationSnapshot.hpp
// 只读契约:estimator -> integrity。不暴露可变图 / 可变 Values。
// ============================================================
#pragma once
#include <cstdint>
#include <optional>
#include <vector>
#include <Eigen/Dense>

namespace uwb_imu_pl::integrity {

// ---- 版本键:构成 §42 缓存键,任何不一致都禁止跨界复用 ----
struct LinearizationVersion {
  uint64_t graph_version      = 0;
  uint64_t ordering_version   = 0;
  uint64_t noise_model_version= 0;
  uint64_t linpoint_version   = 0;   // 绑定 X_star
  bool operator==(const LinearizationVersion&) const = default;
};

// ---- §122:每行的角色决定是否计入 detector statistic 与 DOF ----
enum class RowRole { Measurement, TrustedPrior, Regularizer };

// ---- §120:一致性级别,必须随 PL 结果一并上报 ----
enum class LinearizationConsistency { Strict, CachedChecked, CachedBlind };

// ---- 白化后的单个 factor 行块(§62 ResidualBlock 的线性视图)----
struct WhitenedRowBlock {
  uint64_t      factor_id      = 0;
  RowRole       role           = RowRole::Measurement;
  Eigen::MatrixXd H;                 // W * H_raw   (已白化 Jacobian)
  Eigen::VectorXd b;                 // W * (y - h(X_star)) (已白化残差)
  std::vector<int> col_index;        // 映射到全局 ordering 的列
  int row_offset = 0;                // 在线性系统中的行区间起点
  double robust_weight = 1.0;        // §102:必须可见
};

// ---- prior 提取结果(§49 / §121)----
struct CurrentStatePrior {
  Eigen::VectorXd mean;              // x_bar_k^-        (排除当前 UWB batch)
  Eigen::MatrixXd cov;               // P_k^-           (仅当前状态 marginal)
  bool excludes_current_uwb = false;// 契约保证:无 double counting (§107 R-M3)
  LinearizationVersion version;
};

// 提取方式:A=显式双 update;B=leave-current-out downdate (§51 / §121)
enum class PriorExtractionMethod { ExplicitDoubleUpdate_A,
                                   LeaveCurrentOutDowndate_B };

class EstimationSnapshot {
 public:
  virtual ~EstimationSnapshot() = default;

  // -------- 基本状态与版本 --------
  virtual uint64_t             epochKey()     const = 0;
  virtual Eigen::VectorXd      currentState() const = 0;   // X_star (tangent)
  virtual LinearizationVersion version()      const = 0;
  virtual int                  stateDim()     const = 0;

  // -------- Capability negotiation (§67) --------
  virtual bool supportsDenseSnapshot()            const = 0;
  virtual bool supportsSparseSolve()              const = 0;
  virtual bool supportsPreMeasurementPrior()      const = 0;
  virtual bool supportsFactorProvenance()         const = 0;
  virtual bool supportsTemporaryFactorRemoval()   const = 0;
  // §120 新增:能否在指定点做自洽重线性化
  virtual bool supportsConsistentRelinearization()const = 0;
  // §121 新增:能否只取当前状态 marginal(避免误用全协方差)
  virtual bool supportsCurrentStateMarginalOnly() const = 0;

  // -------- §120 自洽线性视图 --------
  // 对受 hypothesis 影响的子图 + protected_keys,在 X_star 处统一重线性化。
  // 返回的所有行共享同一 LinearizationVersion(实现须断言);
  // 无法重线性化的 marginal-prior 行走 schurProvenance()。
  virtual std::vector<WhitenedRowBlock> consistentLinearView(
      const std::vector<uint64_t>& affected_factor_ids,
      const std::vector<int>&      protected_state_cols,
      LinearizationConsistency     level = LinearizationConsistency::Strict
  ) const = 0;

  // §43:被 marginalize 部分的 (B̄_r, C̄) provenance,保存残差能量来源。
  struct SchurFaultProvenance {
    Eigen::MatrixXd B_bar_r;   // fault -> retained RHS sensitivity
    Eigen::MatrixXd C_bar;     // eliminated 部分对 residual energy 的贡献
  };
  virtual std::optional<SchurFaultProvenance> schurProvenance(
      uint64_t hypothesis_id) const = 0;

  // -------- §49 / §121 prior 提取 --------
  // 排除"当前被监测 UWB batch"的当前状态 prior。
  // Method A: 语义基准; Method B: downdate 加速(须先通过等价回归)。
  virtual std::optional<CurrentStatePrior> currentStatePrior(
      const std::vector<uint64_t>& current_uwb_factor_ids,
      PriorExtractionMethod method =
          PriorExtractionMethod::ExplicitDoubleUpdate_A
  ) const = 0;

  // -------- §40 无显式 J†/P⊥ 的稀疏解 --------
  // 复用当前 factorization: 解 (J^T J) x = rhs;多右端批处理。
  virtual Eigen::MatrixXd solveInformation(
      const Eigen::MatrixXd& rhs) const = 0;

  // §40.4:仅当前状态 marginal(禁止整图求逆)。
  virtual Eigen::MatrixXd currentStateMarginalCovariance() const = 0;

  // -------- §120.3 数值诊断 --------
  virtual int    numericalRank()      const = 0;
  virtual double conditionEstimate()  const = 0;
  virtual std::vector<int> weakDirections() const = 0; // §39 输出弱方向
};

}  // namespace uwb_imu_pl::integrity
```

### 123.1 使用约定(把政策变成断言)

```text
# detector 组装时(§122):DOF 只数 Measurement 行
nu = count(rows where role == Measurement) - rank(H_measurement)
assert TrustedPrior/Regularizer 行不进入 T = r^T r  除非 role == TrustedPrior

# conditional detector(§49):必须确认无 double counting(§107 R-M3)
prior = snapshot.currentStatePrior(current_uwb_ids, method)
assert prior.excludes_current_uwb == true
S_k   = H_k * prior.cov * H_k^T + R_u_k     # §49.1
assert S_k is SPD                            # 恒成立,见审阅 §49.1 论证

# Method B 上线前(§121.3):与 Method A 等价回归
priorA = currentStatePrior(ids, ExplicitDoubleUpdate_A)
priorB = currentStatePrior(ids, LeaveCurrentOutDowndate_B)
assert relError(priorA.cov, priorB.cov) < tol(conditionEstimate())

# PL 结果必须携带一致性级别(§120.3),缺失能力时显式降级、不静默
result.linearization_consistency = level_actually_used
assert not (formal_PL and level == CachedBlind)
```

### 123.2 与里程碑的对接

| 接口能力 | 对应政策 | 里程碑 |
|---|---|---|
| `consistentLinearView` + `LinearizationConsistency` | §120 自洽 relinearization | M7/M9 |
| `currentStatePrior(Method A)` | §121 语义基准 | M8 |
| `currentStatePrior(Method B)` + 等价回归 | §121 speed 候选 | M8(验证)/M9(在线) |
| `RowRole` + DOF 排除 Regularizer | §122 smoothness 语义 | M5 bridge / M8 detector |
| `schurProvenance` | §43 provenance | M10 |
| `currentStateMarginalCovariance` 单键 | §121 禁止整图求逆 | M7/M8 |

**总 gate:** 上述任一 capability 返回 false 时,integrity backend 必须走 §67 的"显式降级或返回 unsupported",严禁静默替换为 covariance-only PL 或 L-Cached-Blind 线性视图。

---

需要的话,我可以接着把 §123 的抽象接口对应到一个具体的 `GtsapIsam2Snapshot` 实现骨架(如何用 `ISAM2::getFactorsUnsafe()`、`linearize()`、`marginalCovariance()` 和 Woodbury downdate 分别落地 Method A / Method B),或者补一份针对 §120–§122 三条 gate 的 GoogleTest 用例清单。
