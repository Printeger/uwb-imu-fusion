# T10-A16 唯一 solver amendment 草案

状态：`PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED`。本轮只交草案；不更新生效方法/实验合同、不改变任何生产接受或停止判据。需要后续指挥单独授权。T10 IN_PROGRESS，C1–C3 不升级。

## 证据与范围

固定 A15 call17 trial1 原生 binary64 端点的参考下降为 `2.59734275148281994e-13`，50/100 位有向区间均严格正。原 linked 线性总目标相减为负；A15 稳定 GN 下降为正，但小于原 `epsilon * oldLinearizedError = 5.163241103110679e-13`。仅改线性相减仍会被旧分辨率门拦截。另一方面，native 白化残差恒等式把下降高估 `4.0095328652322331e-13`，只改为 long-double 累加也不够。

证据支持修复“以大目标的相减/固定相对机器分辨率代理 trial 下降可信性”的数值判定。它不证明全局导数正确、A14 可收敛、所有 trial 可比较。参考与 GN 的剩余差 `1.0490113853023698e-13` 尚未细分为线性模型误差、二阶以上项、原生端点舍入/非正交表示的贡献；本方案不把它设为零，也不以它调参。

唯一提案为默认关闭、仅 paper development 的 `PAPER_CERTIFIED_PAIR_REDUCTION_V1`：以完整实际 trial 的稳定增量和显式误差区间替换两个下降相减及 resolution 门；现有 GTSAM 线性系统/实际 delta、模型、priors、初始化、lambda 设置、接受后的 lambda 更新公式、容差、预算和 generic AND stationarity 不变。缺省/legacy 完全保持。不得改 `/usr/local`；后续实施必须在仓库内隔离策略中传递实际 trial，不用 shadow optimizer 或另求方向。

## 唯一数值规则（待实施）

1. 保留同一次原求解得到的完整 binary64 `delta`；以原 native retract 生成 trial Values。固定该端点，禁止高精度重新 retract/正交化。
2. 对原 live graph 交付的固定 binary64 白化 `J,r,delta`，计算 `v=J delta`，预测下降 `P=-r^T v-0.5 v^T v`，不减两个线性化总目标，不计 damping 为目标项。固定 **333-bit MPFR 有向区间**传播乘加，得到 `[Plo,Phi]`，误差对象是该原生线性模型的算术，不声称给 analytic Jacobian 的正确性证书。原 `-g^T delta` 只作交叉核验，不能换方向。
3. 复用已审计五类 factor 的固定常量/PIM/full R/公式，以固定333-bit有向区间计算两端残差，直接累加 `D=0.5 sum_f,k (r0-r1)(r0+r1)`，得到 `[Dlo,Dhi]`。严禁仅对已有 binary64 残差做补偿求和就宣称覆盖残差求值误差。每个 factor 的公式版本、分支与区间域必须可核对。不给通用任意 factor 开后门，不添加精度搜索。
4. 新 resolution 门由证书替代：两个区间有限、分支确定且半宽均 `<=1e-15`，并有 `Plo>0`。这项 `1e-15` 是 trial 下降证书精度门，不是 stationarity/relative/absolute stopping tolerance；没有证书不接受。取消此 opt-in 策略中的 `P > epsilon * oldLinearizedError` 人工绝对分辨率下限。
5. 只有 **`Dlo>0` 且 `Dlo/Phi > 原 minModelFidelity`** 才接受。阈值保持当前实际 `minModelFidelity=1e-3`，不把“接近”写成通过。原 lambda 更新公式输入保守 fidelity `Dlo/Phi`，原目标/梯度/停止字段继续原算法；另存 D/P 区间供审查。用保守 ratio 是本草案明确的数值语义变化，不能伪称与旧接受规则等价。
6. 明确非下降（`Phi<=0` 或 `Dhi<=0`）/明确低 fidelity：记录可证拒绝，依原 lambda 规则继续至原上限，不追加预算。区间跨零、ratio 门不确定、区间宽度超限：终止本 block 为 `NUMERIC_REDUCTION_UNRESOLVED`；不得当成功，也不得用更大 lambda 作为精度重试。未知 factor、近pi未审计分支、区间跨分支、非有限值为 `NUMERIC_REFERENCE_UNSUPPORTED`/`NUMERIC_REFERENCE_NONFINITE`，停止并完整保留。不能回退到无误差证书的接受。

以上不是“降低容差”或仅“换高精度”：核心变化是 paired reduction、覆盖残差求值的证书、明确 resolution/接受/失败语义；固定精度只为落实已审计的误差包络。它可能因新失败状态更早停止，运行成本也可能增加。只有后续实际回归和授权 pilot 能评价效果；本轮不得从固定端点比值推算一次真实接受、迭代轨迹或驻点。

## 身份失效与必要回归（本轮全部 NOT_RUN）

- 增加 solver numerical-policy/version、interval implementation/source SHA、MPFR/GMP 实际库身份、333-bit precision、factor formula/branch版本、错误状态schema到实际兼容检查。common准备、discovery/support、Stage2 producer/cache/request、final context 的消费必须拒绝混用；旧 checkpoint warm start 不允许进入新策略。未改变的 raw/GT、PIM 模型身份可保留；所有依赖 optimizer 轨迹/停态的旧结果不能复用为新策略结果。
- 缺省/legacy逐trial分支、原A14/A15初始图/Values、默认停止行为回归；新策略只改 reduction policy，actual delta/端点和同lambda的线性系统需逐位核对。
- 固定本端点：复现A16两端371factor的误差区间及D；A15原J/r/delta的P证书独立计算，核对其与原稳定GN数值。先通过静态门，再提出单独固定预算运行协议，不能直接沿本封存末态 warm start。
- 有界单元反例：正/负/零 reduction、大目标相消、binary64残差误差造成假下降、模型预测与真实下降异号、ratio跨门、区间宽度超限、非有限/未知factor/分支不确定。检验未证实不接受，日志不丢失败。
- 原IMU/Pose prior/native retract/GraphBuilder/config/paper/cache/stationarity回归；确认damping不混入信息量/目标、四项AND不变；MPFR有向舍入/精确binary64导入/区间矩阵运算对可解析有理数和超越函数域回归。
- 运行成本、完整trial计数、身份冲突拒绝与默认关闭回归；任何失败即停止后续科学试跑。此草案不授权正式validation/test、scheduler、gate lock、C1–C3升级。
