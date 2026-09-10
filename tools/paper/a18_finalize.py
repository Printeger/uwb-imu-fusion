#!/usr/bin/env python3
import sys,json,csv,hashlib,shutil,subprocess,re
from pathlib import Path
E=Path(sys.argv[1]).resolve();root=Path.cwd();audit=json.loads((E/'RESULT_AUDIT.json').read_text());assert audit['passed'];summary=audit['summary']
# Actual mapped dependencies, including all observed versions of the rebuilt core.
identities=[];store=E/'runtime_libraries';store.mkdir(exist_ok=True)
for p in E.glob('*/command.json'):
 c=json.loads(p.read_text())
 for i in c.get('elf_identity',[]):
  if not i.get('mapped'):continue
  i=dict(i,command=str(p.relative_to(E)));source=Path(i['realpath']);h=i['sha256'];dest=store/h/source.name
  if not dest.exists():
   if hashlib.sha256(source.read_bytes()).hexdigest()!=h:
    assert source.name in ['a18_stage1','a18_tests'],(source,'unexpected library change')
    i['archive']=None;i['archive_status']='HISTORICAL_ENGINEERING_EXECUTABLE_REBUILT_IDENTITY_RETAINED';identities.append(i);continue
   dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
  i['archive']=str(dest.relative_to(E));identities.append(i)
(E/'ALL_MAPPED_IDENTITIES.json').write_text(json.dumps(identities,indent=2)+'\n')
# Sealed A17 regression inputs only, never any evaluation/GT sidecar.
old=E/'A17_regression_inputs';old.mkdir(exist_ok=True);oldids=[]
for line in (E/'43_pair_manifest.txt').read_text().splitlines():
 p=Path(line);dest=old/p.name;dest.mkdir(exist_ok=True)
 for name in ['exact_binary64.csv','factors.csv','linear.csv','certificate.json','factor_certificates.csv','branches.csv']:
  src=p/name;shutil.copy2(src,dest/name);oldids.append({'source':str(src),'copy':str((dest/name).relative_to(E)),'sha256':hashlib.sha256(src.read_bytes()).hexdigest()})
(E/'A17_INPUT_IDENTITIES.json').write_text(json.dumps(oldids,indent=2)+'\n')
# Map actual discovered IDs to raw metadata, no truth read.
raw=root/'doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/inputs/raw/step/uwb_observations.csv';obs=list(csv.DictReader(raw.open()));lookup={x['obs_id']:x for x in obs};parts=list(csv.DictReader((E/'pilot/partition.csv').open()));partsummary=[]
for p in parts:
 ids=[i for i in p['obs_ids'].split(';') if i];links={(lookup[i]['tag_id'],lookup[i]['anchor_id']) for i in ids};assert len(links)==1
 p=dict(p,tag_id=next(iter(links))[0],anchor_id=next(iter(links))[1]);partsummary.append(p)
(E/'PARTITION_AUDIT.json').write_text(json.dumps({'source':'actual Stage1 partition and raw obs_id association; no Stage2 grouping','segments':partsummary,'short_count':0,'boundary_count':None,'boundary_status':'NOT_RUN_STAGE2','group_count':None,'eligible_count':None},indent=2)+'\n')
libs={Path(x['realpath']).name:x['sha256'] for x in identities if x['command']=='pilot_run/command.json'}
strategy=json.loads((E/'pilot/diagnostic_manifest.json').read_text())['strategy_identity']
text=f'''# T10-A18 进程内 certified policy 与唯一完整 Stage1

本地限定交付完成，等待 review。指挥接受 `REVIEW_ACCEPTED_A17_CERTIFIED_FIRST_BLOCK_PROTOTYPE_SCOPE`，来源本指挥会话及封存的 `/tmp/t10-a17-commander-review-j5heubp8/REVIEW.md`。T10 **IN_PROGRESS**；不是validation准入或C1–C3升级。

唯一fresh P1 step seed10101 development运行 exit0，**Stage1在outer90满足原四项AND**，随后立即停止；Stage2/score/cache/final/gate/validation/test/T11/scheduler全部NOT_RUN。这仅支持本development输入的Stage1观察，不支持端到端评分/门控或held-out收益。

## 实现与数学边界

实施前[A18 amendment](../../T10_A18_AMENDMENT_PROTOCOL.md)及合同补充已登记，预登记原件`PREREGISTERED_PROTOCOL.md`逐hash保持。默认关闭，显式`--policy PAPER_CERTIFIED_PAIR_REDUCTION_V1 development-stage1`才运行；A14原config/raw字节不变。

`a18_interval.h`/`a18_certificate.h`为C++进程内MPFR333bit残差/证书计算，没有Python worker、shadow optimizer或额外方向。因当前系统缺mpfr.h，局部声明现有MPFR4 C ABI子集（x86_64结构32bytes、precision333实测），链接原MPFR4.0.2/GMP，无安装/升级和/usr/local改动。RAII管理固定精度数值；每个初始double用mpfr_set_d精确导入；加减乘除/超越函数端点有向舍入。sin/cos用1-Lipschitz半径包络，sqrt用非负平方包络，acos/tan先检查域；沿用A16/A17 SO3近零近似，近pi或跨分支显式失败。完整PIM/15维R和交叉项保留。信息/目标不加入damping。

P=`-rᵀ(Jδ)-0.5||Jδ||²`来自该次native未阻尼linear graph；D=`0.5Σ(r0-r1)(r0+r1)`来自原factor数学定义、固定native端点；range、paper Pose3 prior、velocity/bias prior、CombinedImuFactor的公式与A16/A17相同。只改变计算载体/运算组织和实现身份，不改变接受数值策略。固定333bit、半宽<=精确十进制1e-15、正P/正D和保守fidelity原1e-3门保持。fidelity下界RNDD、上界RNDU，返回double也RNDD且必须仍过门；输出保守十进制包络及精确二进制mantissa@exponent，独立有理审计不混淆打印误差。跨零/过宽/跨门UNRESOLVED，非有限NONFINITE，未知factor/域/未支持分支UNSUPPORTED；无无证书接受或精度重试。

`a18_optimizer.h`最小复用A17基于实际GTSAM LM的solve/native retract/state更新；每trial一解一retract，lambda/defaults/isotropic/SEQUENTIAL_CHOLESKY不变。A17异常分类覆盖NONFINITE的bug已修正；新路径用明确异常类型/前缀保留三类原因并记录unresolved，不算可证拒绝。

`AutomaticSupportProvider::RunDevelopmentStage1`是真实旧provider循环的新显式development请求入口，普通重载传nullptr，原chain、objective allowance、partition与停止代码复用。数值请求与原只读diagnostic observer区分；policy/role/implementation identity和context solver hash必须匹配，不能冒充普通observer。每次条件range工厂调用同步传递同一binary64 `fixed beta + current u`，不沿用首轮u=0。原图factor类型不变；新证书读取实际PIM/noise R/Values，range元数据与图的measurement/key核对。

成功或失败partition均标A18_STAGE1_DIAGNOSTIC_ONLY/development_nonconsumable，输出manifest consumable=false。实际旧Stage2 reader拒绝该schema；入口没有Stage2/cache/final调用。不是已经完成正式cache/scheduler集成。

## 工程门和全部失败

`ENGINEERING_GATE.json`门通过后才创建一次性ticket并运行，记录UTC顺序。gate规则未放宽。

- 全部43个A17固定pair：P/D区间重叠、中心差<=1e-18、半宽<=1e-15；43个P均包络独立精确有理J/r/delta求和，43个接受/拒绝及保守fidelity比较/转换核对通过。原20accept/23reject不变。
- 43对图/Values/371factor/常量/linearization/native端点严格核验（最终加强signed-zero逐位检查）；call17 trial1同base/lambda的一次原GTSAM solve，615维方向一致；**静态零iterate**，不作pilot warm start。
- 完整43-pair批次含input加载、求值、逐factor/branch/完整证书输出：外部 **4.783429s**、内部4.7794s，user4.69s/system0.08s/RSS10852KiB。首批次4.86497s也保留。环境见ENVIRONMENT.json；固定333bit，无并行摊薄/少算pair。
- C++最终104项断言通过：正负零/相消/假下降/不确定/非有限/未知factor/近pi/跨分支/fidelity及实际转换门；实际异常路径三类状态、1call/1trial/1solve/0accepted/0rejected/1unresolved以及CSV输出；vector fixture2calls原generic AND stationarity；真实provider chain后的非零bias、原生方向/端点、完整白化残差与新旧证书相容。
- 原discovery39项GTest全部通过。core、paper runner、discovery tests重建exit0；独立C++原型/tests编译链接通过，实际maps绑定重建core。缺省/非法policy/非development/warm-start入口均拒绝，禁止目录未产生；旧manifest reader拒绝。

失败全部保留于`KNOWN_ENGINEERING_FAILURES.md`及对应命令目录：一次测试编译名字/Eigen表达式错误；两次fixture因错误使用旧V2首轮而lambda耗尽（第二次仅增加失败原因定位），改为被测试的certified首轮后通过；一次fidelity后处理错误使用十进制打印包络再相除，改为精确binary字段校验。没有科学retry或预算追加。只读缺失路径/尚未结束编译的日志查询也列明，不当数值失败。

## 唯一development pilot实测

原A14 config SHA `479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f`；原raw cache `sha256:d0dca8d096f06bc9022408fde98e41f7c8bff02b9428e866d63bf4a4409e26c0`。从原raw重新Initializer，原初始graph `99cdfbc7…`、Values `7115e76d…`严格复现（完整见pilot/initial_identity.csv）。outer500/conditional50/refit200配置保持，实际仅Stage1；整进程树timeout KILL900s，一次无retry。

| 字段 | 实测 |
|---|---:|
| exit / 原Stage1判定 | 0 / CONVERGED: AUTOMATIC_SUPPORT_DISCOVERED |
| 完成outer / 尝试outer | 90 / 90 |
| 总conditional calls / trials（每trial一次native solve） | 327 / 468 |
| accepted / rejected / unresolved | 327 / 141 / 0 |
| external wall / certificate累计wall | 60.508297065s / 31.169742676s |
| user CPU / system CPU（含外部跟踪进程树）/ max RSS | 46.05s / 38.76s / 36432KiB |
| 首block | 20calls / 43trials / 20accepted / 23rejected |
| 首block末态 | 与A17所有call的objective/lambda/counter/gradient/驻点逐字段复现 |
| 首block证书累计wall | 2.752617178s |
| 最终conditional尺度gradient | 1.62949618243e-7 |
| 最终chain后navigation尺度gradient / roundoff | 9.52020803524e-7 / 2.40329281275e-10 |
| 原完整objective相对变化 | 1.13960160701e-15 <= 1e-8 |
| 组合step | 1.22651189205e-9 <= 1e-6 |
| 最大chain KKT violation | 9.87744013026e-9 <= 1e-8；原chain全部审计通过 |
| navigation stationarity | 原1e-6阈值通过 |
| 最终lambda | 1.0000000000000002e-6 |

最后一轮完整objective有3.4e-13级binary64上升，在**原**64epsilon allowance内；未放宽objective检查，四项AND均由原provider判定。首block证书提速和后续90轮是本机观察，不作跨模型objective性能比较，也不宣称通用收敛。

实际自动partition：2段、32个candidate observation；各16条、t=3到6秒、duration3秒，short=0。两段分别link `{partsummary[0]['tag_id']}:{partsummary[0]['anchor_id']}` 和 `{partsummary[1]['tag_id']}:{partsummary[1]['anchor_id']}`，Stage1 merge代表幅值分别0.5783041841m和0.8633718776m；不是truth、Stage2去正则幅值或误差。完整segment/parent/obs_id在pilot/partition.csv，raw关联在PARTITION_AUDIT.json。**Stage2 boundary、group、eligible、unavailable均NA/NOT_RUN，不能填0或称已可评分。**

`pilot/outers.csv`逐outer保留四项AND、目标/导航与bias步长/KKT/驻点/roundoff；每outer的calls.csv/trials.csv/block_status.json保留全部调用与接受拒绝/lambda/终止原因。468个trial目录保存完整binary64 base/trial/delta、实际图常量和PIM/full R、native J/r/delta及完整区间/逐factor/分支。`PILOT_CERTIFICATES.csv`与`RESULT_AUDIT.json`独立核验全部实际接受/拒绝证书、计数、原AND和无失败后继续。

## 身份、隔离和停止

策略身份 `{strategy}`，实现A18_CPP_MPFR_333_V1，绑定实际executable/config/core/GTSAM/MPFR/GMP；源码/输入在运行前封存，pilot后逐hash未变。core SHA `{libs.get('libuwb_imu_fgo.so')}`；GTSAM SHA `{libs.get('libgtsam.so.4.2.0')}`；MPFR SHA `{libs.get('libmpfr.so.6.0.2')}`。GTSAM/MPFR/GMP与A17原库一致；core变化来自显式development接线/接口重建，原初始图和实际首block数值仍复现。

所有实际maps、ELF SHA/build ID/实际动态库完整副本、命令argv/cwd/UTC/exit、raw/config和A17静态输入在包内。file_access.trace的truth/GT禁止open为0；无GT初始化、无生成数据或读取evaluation sidecar。旧A17/A14等证据目录未改。部分早期工程可执行文件在同一路径重建，旧binary副本未保留，maps/原SHA/build ID/命令及失败日志保留并明确标HISTORICAL_ENGINEERING_EXECUTABLE_REBUILT_IDENTITY_RETAINED；不声称这些旧工程binary可逐字恢复。最终pilot binary/source与实际库完整封存。归档脚本首次因这些旧binary与现路径hash不同而显式失败，修正为记录此证据缺口，未将新binary冒充旧版本。

A14pilot失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。此结果仅为一个development输入、默认关闭新策略下完整Stage1；正式validation/test准入仍否，未锁gate，不升级claim。交付后停止等待review，不自行运行Stage2或扩展实验。
'''
(E/'VERIFICATION.md').write_text(text)
brief='''A18限定交付完成待review：登记 REVIEW_ACCEPTED_A17_CERTIFIED_FIRST_BLOCK_PROTOTYPE_SCOPE，来源本指挥会话及独立复审 `/tmp/t10-a17-commander-review-j5heubp8/REVIEW.md`。默认关闭C++进程内333bit证书保留A17数值语义；真实AutomaticSupportProvider全Stage1接线覆盖实际beta+非零u，开发输出schema隔离且旧cache reader拒绝。43-pair全证书/有理P/保守fidelity通过，完整批次4.783429s；C++104项/原discovery39项通过，全部工程失败保留。
唯一fresh P1 step seed10101运行exit0、60.5083s：90outer/327calls/468trials，327接受141拒绝0unresolved；证书累计31.1697s。首block完整复现A17数值；Stage1最终原objective/step/KKT/navigation四项AND通过，chain后尺度梯度9.520208e-7。实际2段/32candidate/short0（每段16条、3–6s）；boundary/group/eligible/unavailable为NA，Stage2/score/cache/final/gate/validation/test/T11/scheduler NOT_RUN。
仅本development输入Stage1观察，非端到端/held-out证据；T10 IN_PROGRESS，A14失败/A15全部FD失败/A12负结果/A08历史15/18与C1–C3限制保留，不锁gate、不升级claim。完成归档后停止等待review。
'''
rel='evidence/'+E.name+'/VERIFICATION.md'
p=root/'doc/ie_sprint/STATUS.md';s=p.read_text();a=s.index('A18进行中：');b=s.index('### A17历史限定交付',a);s=s[:a]+brief+f'\n[完整A18交付]({rel})。\n\n'+s[b:];oldline=next(l for l in s.splitlines() if l.startswith('| T10 validation/gate |'));s=s.replace(oldline,f'| T10 validation/gate | `IN_PROGRESS` | A17首block指挥接受；A18进程内证书工程门通过，唯一fresh step完整Stage1于90outer/327calls/468trials满足原四项AND，60.5083s；2段/short0，Stage2及正式准入NOT_RUN。[A18交付]({rel})。历史限制/claim不变，待review |');p.write_text(s)
p=root/'doc/ie_sprint/T10_READINESS.md';s=p.read_text().replace('A17_LOCAL_FIRST_BLOCK_PROTOTYPE_COMPLETE_AWAITING_REVIEW','A17_REVIEW_ACCEPTED_BOUNDED_SCOPE / A18_LOCAL_STAGE1_COMPLETE_AWAITING_REVIEW');pos=s.index('## A17');s=s[:pos]+'## A18 当前交付与准入判定\n\n'+brief+f'\n[完整A18交付]({rel})。\n\n'+s[pos:].replace('## A17 当前交付与准入判定','## A17 历史交付（本轮指挥接受）',1);p.write_text(s)
p=root/'paper/CLAIM_EVIDENCE.md';s=p.read_text();pos=s.index('## T10-A17');s=s[:pos]+'## T10-A18 完整Stage1 development证据边界\n\n'+brief+f'\n[完整A18交付](../doc/ie_sprint/{rel})。C1–C3原状态不变。\n\n'+s[pos:];p.write_text(s)
for p in [root/'doc/ie_sprint'/n for n in ['METHOD_CONTRACT.md','EXPERIMENT_CONTRACT.md']]:
 with p.open('a') as f:f.write(f'\nA18限定实现及唯一development运行本地完成：[证据]({rel})。工程门通过，fresh step完整Stage1于90outer达到原四项AND；仅Stage1，Stage2/正式准入/claim不迁移，等待review。\n')
# Final documents/source (postprocessing is separate from the frozen pilot binary/source).
for p in list((root/'tools/paper').glob('a18*'))+[root/'doc/ie_sprint'/n for n in ['STATUS.md','T10_READINESS.md','METHOD_CONTRACT.md','EXPERIMENT_CONTRACT.md','T10_A18_AMENDMENT_PROTOCOL.md']]+[root/'paper/CLAIM_EVIDENCE.md']:
 dest=E/'closeout_source'/p.relative_to(root);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
failures=[]
for p in E.glob('*/command.json'):
 c=json.loads(p.read_text())
 if c['exit_code']!=0:failures.append({'record':str(p.relative_to(E)),'exit_code':c['exit_code'],'expected_exit_code':c.get('expected_exit_code'),'purpose':'expected rejection' if p.parent.name.startswith('negative_') else 'engineering failure retained'})
(E/'ALL_FAILURES.json').write_text(json.dumps(failures,indent=2)+'\n')
(E/'FINAL_CHECKS.json').write_text(json.dumps({'passed':True,'result_audit':audit['checks'],'mapped_dependency_records':len(identities),'A17_static_files':len(oldids),'Stage2':'NOT_RUN','validation_test_T11_scheduler':'NOT_RUN'},indent=2)+'\n')
print(json.dumps({'completed':True,'summary':summary,'actual_libraries':libs},indent=2))
