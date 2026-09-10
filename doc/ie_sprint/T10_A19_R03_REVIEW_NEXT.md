# R03 封存复审与下一轮任务

2026-09-10，指挥会话只读复核；未运行 estimator、未修改实现或实验参数。以下结论依据实际源码、封存
stdout/stderr、退出码、file-open trace 和身份，不把原 VERIFICATION 的概括当成独立测试结果。

## 判断

R03 增加了有限的 guard 与 final adapter 工程能力，但完整输入没有验证 handoff 是否能帮助最终收敛。
SIGSEGV 是当前必须定位的软件执行故障，不能用来支持或否定 eta、修正收益或 R02 outer15 的数值诊断。
R02 已完成 Stage1 的历史观察仍成立；R03 当前构建的完整路径发生了崩溃，原因及是否由新增修改引入未定。
目前没有证据说明研究问题遭遇不可跨越障碍，也没有证据证明只修崩溃就一定能完成 Stage2。

### 已独立核对的证据

以下 R03 证据路径均相对
`doc/ie_sprint/evidence/t10_a19_r03_inexact_handoff_20260910T061142Z/`。

- `attempts/attempt2/estimator_run/command.json`：exit139、2.096557408s。当前磁盘 runner、实际 mapped
  core/GTSAM/MPFR/GMP 五个文件 hash 均与该记录一致；这只确认身份，不代表 ABI 兼容。
- `attempts/attempt2/estimator_run/file_access.trace` 末尾：读取完整 raw 后 SIGSEGV，
  `si_code=SEGV_MAPERR, si_addr=0x39`。它给出故障访问地址，不给出指令位置或根因。
- `tools/paper/a19_r03_pipeline.cpp:166` 构造 A17Graph；`tools/paper/a17_graph_io.h:73-78` 包含
  config、raw load、plan、materialization、初始化、建图及内容身份检查。无原始调用栈，不能精确断言
  崩溃在 initializer 或 GraphBuilder 内。没有 trace 不能单独证明优化未开始。
- 当前系统 `core_pattern` 为 `|/wsl-capture-crash %t %E %p %s`；封存交付中未见调用栈/core 文件，
  `/var/crash` 仅发现无关旧 ROS crash。没有全机搜索 WSL 宿主侧转储，不能断言 core 从未生成。
- core 当前实际构建参数见
  `/home/mint/ws_fusion_uwb/build/uwb_imu_fgo/CMakeFiles/uwb_imu_fgo.dir/flags.make`，包含 `-march=native`；
  R03 `COMMANDS.md` 对手工 runner 只记录 C++17/O2，没有给出可复核的完整 compile argv。
  Eigen 对齐/对象布局/跨库 ABI 与编译选项一致性应优先核查，**尚未证实为崩溃原因**；O2/O3 差异本身
  也不能证明 ABI 错误。
- `commands/focused_tests_2/stdout.log` 确认正确过滤器 `SegmentRefitter.A19*` 实际通过3项。
  `commands/final_engineering_gate/stdout.log` 同时含 `T10A19R03.*` 的0项测试，以及另外实际通过的2项
  final/policy 测试。0项退出0不能计作有效测试，也不能因此抹掉另行真实执行的3项证据。

### 验证覆盖及后续接线缺口

1. `tools/paper/a19_r03_preflight.py:20-58` 的8项检查覆盖路径、动态库解析、隔离目录、日志与进程树，
   没有调用正确参数下的 runner 初始化/建图成功路径。因此通过8/8与随后崩溃并不矛盾。
   下轮应新增真实同runner的 prepare-only 正路径，在 Stage1 入口前明确停止，而非增加无关检查数量。
2. `tools/paper/a19_r03_authorize.py:30-36` 记录自动900s/final各900s/总5400s，但实际只用一个900s
   timeout 包住整个进程树。`tools/paper/a19_r03_pipeline.cpp:511-582` 在同树 fork 五个final、阻塞waitpid，
   没有每策略独立900s计时与终止。实际预算调度不等于协议；它比协议更短，未造成预算超支，却可能在
   Stage2成功后截断本应完成的比较。
3. `test/test_t09_evaluator.py:15` 开始自行构造临时轨迹/ledger/truth再调用评价器；其 exit0支持既有评价器
   回归，不证明已消费R03 mixed graph导出的真实final。`a19_r03_pipeline.cpp:480-482` 注释声明外部独立
   evaluator，但本次检查的授权器在runner结束后只写COMPLETION退出。完整evaluation外部编排与R03
   adapter产物消费尚无本轮连贯运行证据，不能写成完整score→decision→final→evaluation已打通。

## 下一步选择

唯一第一优先级：在新隔离诊断中取得真实崩溃栈，定位并修复唯一根因；随后验证同一runner完整raw的
prepare-only正路径。暂不改变handoff、lambda、驻点阈值、证书精度或输入。不能把编译选项线索当成已定案，
也不能用换一组flags后不崩溃替代根因证据。

在下一次科学运行前，仅补上述直接阻塞的分阶段预算与真实final→evaluator消费检查。成功后继续一次完整
raw→Stage1→Stage2→score及原五策略比较。下列prompt给出一次连贯任务，避免每修完一个启动问题就另起
一轮没有科学结果的“完成”。本文件只提出任务，没有消费任何新诊断或科学ticket。

## 可直接交给 Codex 的任务 prompt

执行 T10-A19-R04：定位 R03 SIGSEGV、修复最小根因，完成真实路径工程门，再进行一次完整评分与收益比较。
工作目录 `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie`。

先读 STATUS、冻结论文结构/roadmap、完整 METHOD/EXPERIMENT_CONTRACT、R03协议及本文件的复审发现。
保护dirty worktree和全部R03封存；登记新的R04协议、隔离输出、诊断预算与科学ticket，不复用R03 ticket。
本prompt授权以下有限诊断、根因修复及满足工程门后的完整运行，不停在计划或局部测试通过。

1. **先取得根因证据。** 任何重建前保存R03实际runner、core和加载库的可复现身份/构建配置，避免动态库
   被覆盖后旧runner无法复现。优先读已有core；若没有可用core，以原runner/原库/原raw在GDB下定向运行，
   设置SIGSEGV停止并采集所有线程backtrace、故障指令/寄存器、maps/build-id及可用局部变量。
   在Stage1入口设置已解析的停止断点；若正常走到入口便停止诊断，不让它悄悄变成完整估计。
   首次原栈诊断最多60s；仅栈无法定位时允许一次有针对性的带符号/ASan或阶段标记诊断，最多120s。
   不能盲目重跑、重建GTSAM、升级依赖或遍历优化参数。若仍不能定位，封存明确的栈和未知项后停止。

2. **只修证据支持的根因。** 优先核对手工runner与core/GTSAM的完整compile/link argv、Eigen对齐宏、
   相关跨库类型sizeof/alignof、对象布局与运行时库；这是假设，不是结论。检查实际栈指向的生命周期/
   越界/空指针/ABI问题。若证实构建配置不一致，让runner使用项目既有target/imported dependency的
   一致构建设置，避免继续手写一套偏离的编译规则。保存修复前复现、根因和修复后对照证据。
   不修改handoff规则、阈值、lambda、证书333bit、IMU模型、初始化数学定义或分段/评分定义。

3. **补真实正路径工程门。** 同一最终runner增加或使用显式prepare-only：完整冻结raw→load→plan→
   materialize→initialize→graph/Values/content identity，Stage1入口前停止，明确输出阶段标记及
   Stage1/conditional optimizer调用数为0；不要仅凭缺少日志作判断。最多一次修复后prepare验证/120s，
   失败则停止该任务，不能再调另一个方案。为真实成功argv、错误argv、原故障保护写必要回归。
   聚焦guard/mixed graph测试实际执行数必须>0；原R03已有证据身份未变的检查可复用。

4. **科学ticket前补齐直接下游缺口。** 保持原900s自动、每final900s、final共4500s、总5400s预算，
   真正分阶段计时并终止超时策略进程树，保留已完成结果。不能仅改ticket字段或用总900s覆盖全部比较。
   用同一构建的小型真实mixed graph产物执行score→decision→final，再由独立evaluator实际消费这些
   导出的轨迹/bias/ledger，获得可核验指标。已有自造文件的T09 evaluator回归不能替代这一检查。
   这不授权重做数据平台/调度器；只补最小adapter与控制逻辑。明确区分科学和工程fixture成本。

5. **工程门通过后直接执行一次fresh全流程。** 使用新ticket、新identity，从冻结P1 step seed10101
   完整raw原始初始化；Stage1 outer<=500、conditional<=50、Stage2 outer<=200，自动流程<=900s。
   第一处算法失败停止依赖分支，保存实际调用栈或梯度/四项AND；无retry/阈值或预算调整。
   若Stage2最终joint四项AND真正通过且有eligible分数，冻结所有decision后完成R03原五策略：
   suppress_all、structured_debias、fit_only(gamma<=1)、s_fit(再加s<=0.10m)、
   full_gate(再加eta>=0.10)，至多原合同一次fallback；同决策复用须满足原等价条件。
   保留全部failed/zero-coverage/unavailable/fallback。共享correctness故障停止受影响策略。

6. **取得实际收益或负结果。** 全部可运行decision/final冻结后，独立evaluator才读取evaluation-only
   truth。estimator全过程不得读取GT/oracle。报告完整记录及[3,6]s raw-frame ATE RMSE/P95/matches、
   相对suppress_all配对差值、decision/final bias-field error、accepted risk、good rejection、coverage、
   retained fraction、failure/fallback和成本，epsilon_bad=0.20m不变。零接受risk=UNDEFINED，gate同决策
   不宣称eta增量；全部不可评分则报告该负观察并按原协议停止final。

交付代码/构建最小改动、真实崩溃栈及根因、完整命令/退出码/实际身份、prepare-only和真实fixture证据、
分阶段预算验证、新完整运行结果或首次明确失败。更新STATUS/CLAIM_EVIDENCE，保持consumable=false，
不升级C1–C3。人话回答：软件崩溃是否被根因修复；是否真正越过outer15并最终收敛；真实分数是否产生；
修正改善还是恶化；下一项科学证据缺什么。到配对结果表或预登记停止边界结束，不继续求解器支线。
