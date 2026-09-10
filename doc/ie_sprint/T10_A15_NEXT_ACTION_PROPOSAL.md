# T10-A15 唯一最小后续方案（PROPOSED / NOT_RUN）

仅对封存 call17 trial1 做一次零 optimizer iterate 的非线性残差求值精度审计。原因是本轮已证实该 trial 的 linked 线性化总目标相减为 -1.3642420526593924e-12，而同一 J/r/delta 的原生梯度公式和 long-double 增量公式均为 +1.54833137e-13；但该正值仍低于原 5.163241103110679e-13 分辨率门。非线性残差恒等式给出的 +6.606875616719874e-13 不能证明真实下降，更不能证明改算术就能达到驻点。

唯一待补缺口是：这个实际 trial 的非线性目标变化中，binary64 残差求值误差与真实非线性余项各占多少。直接放宽分辨率门、用 long double 累加结果代替接受判断或增加 lambda/预算，均没有足够依据。

建议后续授权的固定工作：

1. 只恢复 A15 封存的同一 graph、call17 base 和 trial1 实际完整 delta；先通过 A15 graph/Values/factor 内容门。原生 retract 生成的 binary64 端点保持，不把高精度重新 retract 后的另一端点混入比较。
2. 只构建该固定端点对的独立残差参考：按当前 Expression range、paper Pose3 prior、velocity/bias priors、CombinedImuFactor 的已记录数学公式、原 PIM 常数和完整白化矩阵计算。仅提高诊断求值精度，不生成数据、不优化状态、不改变模型、PIM 或 factor mask。与原公式/单位/局部参数化的对应必须逐项验证；无法对齐则报告证据不足。
3. 预登记使用 50/100 十进制有效位作参考自身稳定性对照，不做精度或参数搜索。要求目标变化参考的区间/保守误差估计半宽 <=1e-15，且两精度的变化差 <=1e-18；仅不同精度结果接近不视为严格误差界。若不能达到、或 SO(3) 数值语义无法对齐，明确停止为不确定。
4. 分别报告固定端点的残差求值误差、非线性目标变化与本轮 GN 增量的差，不把参考差直接接入接受决策。零 optimizer/solve，零新场景，Stage2/gate/held-out NOT_RUN。

本方案只补充静态诊断证据，不改变生产接受、停止或数值语义，因此本轮不提出可实施的 solver amendment。若后续证据支持改写 objective/linearized reduction 或替换原 resolution threshold，必须再交独立的具体 amendment，说明接受判据、误差证书、失败状态、版本/缓存失效和必要回归；不能把本方案视为该修改的授权。

T10 保持 IN_PROGRESS；本方案本轮不执行，不宣称能恢复收敛，不升级 C1–C3。
