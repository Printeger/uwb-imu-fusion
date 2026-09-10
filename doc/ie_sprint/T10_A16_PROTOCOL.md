# T10-A16 固定端点精度审计运行前协议

登记时间2026-09-09T16:02:52Z（本地2026-09-10），先于实现和端点/参考计算。
指挥接受 REVIEW_ACCEPTED_A15_TERMINAL_NUMERIC_DIAGNOSTIC_SCOPE，来自本用户指挥会话，非外部REVIEW.md。T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败及历史15/18等限制保留。

只用A15 P1 step seed10101 call17 base/trial1完整delta。复用A15 C++ graph恢复路径，initial/terminal graph和Values hashes严格匹配，371factor type/key/error、123key/615dim核对；仅此delta的615坐标读取。原生Values::retract一次生成binary64端点并封存，不使用高精度retract/正交化；原PIM/15维完整白化R、参数不变，零optimizer/solve/iterate，零新estimator进程。

全部常量/端点以IEEE binary64 hex及bits导出；参考精确导入double（二进制MPFR精度>=53bit）。固定50/100十进制有效位对应ceil(dps*log2(10))=167/333bit，分别最近舍入点计算及向外舍入区间，不增加第三精度/精度搜索。使用本机已有libmpfr.so.6 via ctypes（不安装/升级依赖），初查默认Python无mpmath，保留该查询失败。

定义D=E(base)-E(native binary64 trial)，D>0为下降。两档点D差<=1e-18；另以逐操作向外舍入自然区间包络计算同一公式，要求D区间半宽<=1e-15。区间上下界严格正才确认下降；跨零明确不确定。两档一致不能单独作为误差界。区间结论条件于已审计MPFR正确有向舍入基本/初等函数语义，不声称形式化验证整个工具链。

参考保持实际SO3 Exp/Log及Pose3 Log分支/近零近似，不把矩阵变成正交矩阵，不用正交性简化Ri*Ri^T；Coriolis/bodySensor等实际参数导出核对。每类factor给出源码对应、分支证据及中间量。若分支跨界、未知factor、公式不对应，停止为不确定。公式初步对应检查：逐未白化残差与native差<=1e-8、白化差<=1e-6（仅sanity，不是误差界/科学通过），并必须逐项追踪公式来源；原始native残差与A15封存值严格相同。

分解：参考D；用native未白化残差和固定R高精度计算D_u；用native白化残差精确累加平方得到D_w；native逐factor error精确求和D_f；native graph.error总差D_g。D_u-D为未白化残差求值影响；D_w-D_u为白化浮点影响；D_f-D_w为平方/因子求和影响；D_g-D_f为图级累计/相减影响，全部逐factor和总量保留。参考D与A15稳定GN下降1.54833136618045015513e-13的差独立报告，不能自动称全是残差误差。

若证据支持，只交唯一solver amendment草案：稳定下降计算/分辨率门/误差处理/接受条件/失败状态/身份失效/回归具体化，不实施。证据不足则给精确缺口并停止。生产solver/GTSAM/模型/容差/预算不改，truth/GT/其他trial/场景/chain/Stage2/gate/validation/test/T11 NOT_RUN，claim不升级。
