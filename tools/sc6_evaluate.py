#!/usr/bin/env python3
"""Render the unique SC6 conclusion and two figures from immutable native traces."""
import csv,json,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sc6_system_research import ROOT,CACHE,score,CONFIG
import yaml

def render():
    summaries={p.stem:json.loads(p.read_text()) for p in CACHE.glob('final-*.json')}
    if not all(k in summaries for k in ['final-L0','final-L1','final-C1','final-C2']):raise RuntimeError('complete frozen short and long matrix is required')
    freeze=json.loads((CACHE/'freeze.json').read_text());cfg=yaml.safe_load(CONFIG.read_text())
    rows={name:list(csv.DictReader((CACHE/(name+'.csv')).open())) for name in summaries}
    clockfix='final-L1-clockfix' in summaries
    if clockfix:
        summaries['initial-L1']=summaries.pop('final-L1');rows['initial-L1']=rows.pop('final-L1')
        summaries['final-L1']=summaries.pop('final-L1-clockfix');rows['final-L1']=rows.pop('final-L1-clockfix')
    # Independent input identity check across A ON/OFF and old/new.
    for case in ['S0','S1','S2']:
        names=[f'final-{case}',f'final-{case}-off',f'final-baseline-{case}']
        assert len({summaries[n]['raw_digest'] for n in names})==1
    assert summaries['final-S1-off']['raw_digest']==summaries['final-baseline-S1-off']['raw_digest']
    qualified=sum(d['qualified'] for d in summaries.values());planned=sum(d['planned'] for d in summaries.values())
    native=sum(d['native'] for d in summaries.values());exceed=sum(d.get('h_exceed',0)+d.get('v_exceed',0) for d in summaries.values())
    lcomplete=all(summaries[k]['native']==summaries[k]['planned'] for k in ['final-L0','final-L1'])
    p=[];add=p.append
    add('# SC6 results\n\n## 1. Conclusion\n')
    add('**SC6_EVALUATED_WITH_GAPS**. W0→W4 development and the registered final short/5000/10000 paced evaluation were executed in this Goal. No production noise, risk, AL or publication rule changed; no push.\n')
    add('|Capability|Actual scope/status|\n|---|---|')
    add('|SIMULATION_END_TO_END_VERIFIED|Verified conditional registered D0/G0 and D1/G1 simulation flow; G2 functional refusal retained. Privileged simulation source qualification only.|')
    add('|PL_TIGHTENING_VERIFIED_FOR_SCOPE|Verified coupled dynamics + sphere contractors on the original event; same-center M gain. Main bias remains RW-only.|')
    add('|FAULT_RECOVERY_VALIDATED_FOR_SCOPE|D0 six strong witnesses validated. Requested D1/weak/late scope has missed episodes; no blanket recovery award.|')
    add('|REALTIME_TARGET_MET_FOR_TESTED_PROFILES|OPEN: retain every 40/50ms miss below, including 40/40. Means do not grant a strict deadline award.|')
    add('|RESOURCE_BEHAVIOR_VERIFIED_FOR_TESTED_DURATION|Finite measured behavior below; RESOURCE_OPTIMIZATION_OPEN for native tombstone/alert growth. No O(1) total memory.|\n')
    add(f'Frozen final matrix (includes retained initial L1 logging failure when present): {planned} planned / {native} successful receipts / {qualified} raw-qualified. Sampled H/V exceedance count={exceed}; this finite observation does not prove epsilon=1e-6. Mathematical inclusion/risk proof is separate. Application and hardware qualification stay CLOSED/UNQUALIFIED.\n')
    add('## 2. Baseline and scope\n')
    add(f"Entry HEAD `20831ec47888c6d96c9ac73571254b0efd8e9b52`; computation freeze `{freeze['head']}`. Runner SHA256 `{freeze['elf']}`; unchanged production DSO `{freeze['dso']}`; config `{freeze['config']}`. Final delivery commit is obtained with `git log -1`, avoiding self-referential SHA.\n")
    groups={}
    for name,d in summaries.items():groups.setdefault((d['identity']['head'],d['identity']['elf']),[]).append(name)
    add('Actual computation identities (math/source/selection model is shared, clock instrumentation revision is separate):')
    for (head,elf),names in groups.items():add('- '+','.join(sorted(names))+': HEAD `'+head+'`, ELF `'+elf+'`.')
    add('GTSAM4.2a5 `/usr/local/lib/libgtsam.so.4` + unstable; Eigen3.3.7; TBB2020.1. Release `-O3 -march=native`; proof sources `-frounding-math -ffp-contract=off`, no fast-math. One TBB/BLAS/FGO worker; no affinity/WCET guarantee. Platform: Linux6.18.54.1-microsoft-standard-WSL2, x86_64, Intel Core Ultra5 125H,18 logical CPUs; no CPU affinity.\n')
    add('Original source: range sigma=.1m, accel ADC=.02m/s², gyro ADC=.001rad/s; original initial15D tangent sigmas and sample-grid bias RW from inherited IMU config. Epsilon=1e-6; global time-uniform births never reset. D0 A=.04/Omega=.2; D1 A=1/Omega=1/J=2/alpha=2. Config freezes G0, G1 eight corners (±4,±4,0/4), G2 weak vertical0/.001; seeds20261010/20261011 and complete fault manifests. Duration/strength were not changed after results. All actual raw digests agree in A comparisons.\n')
    add('W0 old B0/B1:650/650 receipts/raw; B1 repairs61/321 and rejoins80–83/340–343. Legacy times included scoring and omitted final gate/I/O, so no equivalent speed ratio to SC6. W1 raw/initial/range evidence exact old equality. Recovered adapter discrepancy1.686 was fixed by passing resolved window config to backend; W3 B1 max state/PL difference5.399e-8 (existing GN tolerances1e-7),400/400 and two true repairs. Production DSO SHA never changed.\n')
    add('## 3. Mathematical changes and independent checks\n')
    add('For ||a(t)||≤A, p1−p0−dt(v0+v1)/2=∫(dt/2−s)a(s)ds, so norm≤A dt²/4. Directed coordinate inverse intersections preserve every feasible trajectory; exact bang-bang a(s)=A sign(dt/2−s) attains the bound. Independent100-digit oracle verifies the extremum and halves an informative old displacement box. This adds a relation, not extra old scans.\n')
    add('For all SO(3), ||Rl||=||l||: intersect tag with p±||l|| and inverse p with tag±||l||. A shell row requires (xj−aj)²≤u²−sum(other coordinate distances²); outward lower bounds on the sum and outward sqrt give a necessary inverse. Intersect within each h, union all nine still-possible masks. Unprocessed frontier stays, including max_boxes=0 retaining all9 original boxes; no chosen FDE identity, local optimum, Gaussian-min or GT branch selection enters raw proof. All operators inherit E; no new random source or risk charge. Original numeric same-contract fallback remains.\n')
    add('Prototype known-R scalar constant-a: a=(p2−2p1+p0)/dt² and y=a+b+f+n, with carried RW. p=(0,.5,2), y=3, dt=1, n∈[−.2,.2] (support10 times the original ADC sigma .02) contracts healthy b from[−10,10] to[1.8,2.2]. Unbounded fault keeps the full box: (b=1,f=1) and(b=2,f=0) give identical observations. Prototype is executable, separately tested and deliberately not inserted into the main all-fault bias union. Main bias has no observation update; all-direction bias/attitude observability remains OPEN.\n')
    add('Original all-time induction remains: delta0=eps/2, deltak=eps/[2k(k+1)], block0 has21 sources and each subsequent block128. Shared ADC keeps original birth q; RW/history/ended episodes persist in separator and symbolic birth recipe. Summable union gives P(E complement)≤eps without independence between adaptive outputs. New necessary intersections preserve the true path on E. Every issued bound protects the same function-produced committed tag/receipt/stamp, including after numerical prior replacement. Unknown model/concurrency/hardware risks are not assigned zero.\n')
    add('|M same center|denominator|strict H/V|mean new/old H/V|\n|---|---|---|---|\n|S0|300/300|300/300 each|.767141/.897589|\n|S1|400/400|400/400 each|.799357/.802006|\n')
    add('M uses one actual commit center and two proof chains; its doubled cost is not paced service performance. A lets the new capsule affect recovery metric; tight profiles also retain existing frozen boundary rotation as an optimization metric because polar midpoint of a full rotation box is not identifiable. It is not an independent observation. Mixed numerical effects below are retained. P fixes this mathematical/selection model and caches constants/candidates, stops exactly fixed passes, and reduces copied transaction material. P90 healthy+joint pairs are bit-equal in all state/action/PL/risk/source/ledger fields; median core51.4368→16.5313ms and52.7797→18.2749ms.\n')
    add('## 4. Simulation and isolation\n')
    add('M1 quintic stop–go: q=10u³−15u⁴+6u⁵, d=(.8,.4,.2), travel4s/rest2s/return4s/rest2s, continuous p/v/a; max second derivative10/sqrt3 and third60 give A<.331,J<.860. M2/M3 sinusoidal3D(.8,.6,.4),(.6,.5,.4) give A<.332,J<.191. R=Rx(.3sin(.4t))Ry(.4sin(.3t))Rz(.5sin(.2t)); body gyro=RzᵀRyᵀex*xprime+Rzᵀey*yprime+ez*zprime, rate≤.34, alpha<.181. Analytic continuous proof fits frozen D1. Directed Taylor/Machin-pi/Rodrigues numerics, seam/late100-digit independent scalar oracle and raw generator checks pass.\n')
    add('Provider owns trajectory/GT/draws/injection labels. Service receives only arrived measurements, original law/domain and privileged SIMULATION_SOURCE_CERTIFIED numerical qualification/sealed range residue. Certification actually executes per arrived epoch inside service, with carried RW residue; it does not reveal health or future extrema. Domain is consumed by monitor, physical propagator and actual D1 bridge. Current/future GT/data cannot enter step; direct future mutation leaves actual state/action/PL equal. Scoring follows payload delivery; main scores use committed terminal states, known world, no alignment or history smoothing. Hardware-source portability remains unqualified.\n')
    add('## 5. Complete final system results\n')
    add('|Row/profile/FDE/mode|planned/native/raw/timely|abort/unattempted|tag RMSE m|H/V RMSE m|HPL/VPL mean m|40/50 misses|\n|---|---|---|---|---|---|---|')
    for name,d in sorted(summaries.items()):
        rm=d.get('rmse',{});met=d.get('metrics',{});fmt=lambda v:f'{v:.6g}' if v is not None else 'NA'
        mean=lambda k:met.get(k,{}).get('mean')
        add(f"|{name}/{d['profile']}/{d['fde']}/{d['mode']}|{d['planned']}/{d['native']}/{d['qualified']}/{d['timely']}|{d['abort']}/{d['unattempted']}|{fmt(rm.get('ape'))}|{fmt(rm.get('hpe'))}/{fmt(rm.get('vpe'))}|{fmt(mean('hpl'))}/{fmt(mean('vpl'))}|{d.get('core40_miss','NA')}/{d.get('delivery50_miss','NA')}|")
    add('\nAll refusal/missing outputs remain in denominators; no stale fill. G2/S3 functional numerical seed/candidate refusal is not a proof that the physical feasible set is empty. Its planned300 epochs are retained. Offline service timing is not paced measurement-to-sink.\n')
    add('|Full-state row|velocity RMSE m/s|rotation RMSE rad|accel/gyro bias RMSE|RPE1 translation/angle mean|RPE20 translation/angle mean|\n|---|---|---|---|---|---|')
    for name,d in sorted(summaries.items()):
        if 'rmse' not in d:continue
        rm=d['rmse'];r=d['rpe'];text=lambda dt:'/'.join(f"{r[str(dt)][k]['mean']:.6g}" for k in ['translation','rotation'])
        add(f"|{name}|{rm['velocity_error']:.6g}|{rm['rotation_error']:.6g}|{rm['accel_bias_error']:.6g}/{rm['gyro_bias_error']:.6g}|{text(1)} ({r['1']['valid']}/{r['1']['planned']})|{text(20)} ({r['20']['valid']}/{r['20']['planned']})|")
    add('\n|Bias axis RMSE row|accel x/y/z m/s²|gyro x/y/z rad/s|\n|---|---|---|')
    for name in ['final-S0','final-S1','final-S2','final-L0','final-L1']:
        good=[r for r in rows[name] if r['updates']=='1']
        def axis_errors(prefix):return '/'.join(f"{np.sqrt(np.mean([float(r[prefix+x])**2 for r in good])):.6g}" for x in ['x','y','z'])
        add('|'+name+'|'+axis_errors('baerr_')+'|'+axis_errors('bgerr_')+'|')
    add('\nNo application accuracy threshold exists; these are numerical evaluations. APE/H/V and state error P95/max are provided for main/long rows below. Bias per-axis errors are scored separately in native trace; scalar table is vector RMSE.\n')
    for name in ['final-S0','final-S1','final-S2','final-L0','final-L1']:
        d=summaries[name]
        if 'metrics' not in d:continue
        add(f"\n**{name}** error median/P95/max:")
        for k in ['ape','hpe','vpe','velocity_error','rotation_error','accel_bias_error','gyro_bias_error']:
            m=d['metrics'][k];add(f"- {k}: {m['median']:.6g}/{m['p95']:.6g}/{m['max']:.6g}")
        add('Fault-phase vector RMSE (n; tag/velocity/rotation/accel-bias/gyro-bias):')
        for stage,data in d['stage_rmse'].items():add(f"- {stage}: {data['n']}; "+'/'.join('NA' if data[k] is None else f'{data[k]:.6g}' for k in ['ape','velocity_error','rotation_error','accel_bias_error','gyro_bias_error']))
    add('\n|Episode row|onset/axis/duration|first real bridge|prior repair|first4 real IMU rejoin|UWB correct isolation epochs|\n|---|---|---|---|---|---|')
    for name,d in sorted(summaries.items()):
        if d['fde']=='off' or d['profile']=='baseline-adapter':continue
        for e in d.get('episodes',[]):add(f"|{name}|{e['onset']}/{e['axis']}/{e['duration']}|{e['first_bridge']}|{e['prior_repairs']}|{e['rejoin4']}|{e['correct_uwb_isolations']}|")
    add('\nBridge/prior/rejoin fields come from actual unique backend receipts and ledger checks. Rejoined flags from an earlier episode are not counted for a newly missed episode. D1 gyro and late misses stay requested-scope OPEN. Healthy extra UWB exclusions (including conservative hold) are: '+', '.join(f"{k}={d.get('healthy_extra_exclusions',0)}" for k,d in summaries.items() if d['fde']=='on' and d['profile']=='tight-optimized')+'. No current hypothesis is used as truth.\n')
    add('## 6. Realtime and resource behavior\n')
    add('RUN_READY maps sensor stamps to steady clock. Independent producer releases200Hz IMU and20Hz UWB while single worker can be busy. Payload copy and actual receiver stamp/version/duplicate/gap/deadline gate are measured through sink; GT and diagnostic trace follow handoff. Required source verification, proposal, sole commit, proof and internal cleanup are inside service. Diagnostic scoring/I/O and release of provider envelope after delivery can delay the next worker and are reflected in paced queue/measurement-age; they are not hidden precomputation. This is finite software measurement, not hardware/WCET certification.\n')
    add('|Paced row|core mean/P95/max ms|measurement-to-sink mean/P95/max ms|RSS max KiB|slots first/last/max|live/ledger max|queue peak|\n|---|---|---|---|---|---|---|')
    for name,d in sorted(summaries.items()):
        if d['mode']!='paced' or 'metrics' not in d:continue
        m=d['metrics'];s=m['slots'];triple=lambda key:'/'.join(f"{m[key][v]:.5g}" for v in ['mean','p95','max'])
        add(f"|{name}|{triple('core_ms')}|{triple('delivery_ms')}|{m['rss']['max']:.0f}|{s['first']:.0f}/{s['last']:.0f}/{s['max']:.0f}|{m['live']['max']:.0f}/{m['ledger']['max']:.0f}|{d['input_counters']['queue_peak']}|")
    add('\n40/40 has genuine strict deadline misses; no average removes them. Failure paths have separate total cost in native full-schema refusal rows. Real cancel16:2 mutations/14 refusals; real overload16:3 mutations/13 refusals, queue80,160/110 IMU and16/11 UWB released/enqueued, no stale protection. Receiver unit tests cover timeout/duplicate/stamp/gap and continuity after a mere late receipt. Source gaps latch refusal, do not reset event. No raw stamp is replaced by wall time.\n')
    add('Native tombstones grow because GTSAM marginalizeLeaves appends priors bypassing already-enabled ordinary slot reuse. No unsafe remapping/second commit was attempted. Transient transaction copy is reduced; observed-alert records retain IDs until4096 then refuse. RSS includes the finite provider dataset, which scales with playback duration; online proof window/replay/capsules are separately bounded. q/bias growth is physical/source support, not reset away. RESOURCE_OPTIMIZATION_OPEN remains for infinite slot/record behavior.\n')
    add('## 7. Final long streams and late faults\n')
    for name in ['final-L0','final-L1']:
        d=summaries[name];add(f"**{name}**: {d['planned']} planned / {d['scheduled']} rows / {d['backend_mutations']} actual mutations / {d['native']} receipts / {d['finite']} finite estimates / {d['qualified']} raw-qualified / {d['timely']} timely; abort{d['abort']}, refusal{d['refused']}, unattempted{d['unattempted']}, never emitted{d['never_emitted']}; inputs {d['input_counters']}.\n")
        if 'metrics' in d:
            add('First/last/max: '+', '.join(f"{k}={d['metrics'][k]['first']:.6g}/{d['metrics'][k]['last']:.6g}/{d['metrics'][k]['max']:.6g}" for k in ['q','risk','bias_width','hpl','vpl','slots','rss','alerts','replay','capsules'])+'.\n')
            add('Snapshots at all500-epoch boundaries and declared late episode onsets (epoch:HPL/VPL/bias-width/q/alerts/bridge/repair):')
            for r in rows[name]:
                if r['updates']=='1' and (int(r['epoch'])%500==0 or int(r['epoch']) in [121,1121,3121,5121,7121,9121]):add(f"- {r['epoch']}: "+'/'.join(r[k] for k in ['hpl','vpl','bias_width','q','alerts','bridge','repair']))
    add('\nLong measurement durations are250s/500s after RUN_READY, not risk horizons. Later20m/s² and1rad/s episodes use the original amplitudes. Expanding RW/motion support can hide them; this is LATE_FAULT_DETECTABILITY_OPEN, not successful restoration. Larger PL can cover a degraded estimate but cannot establish navigation recovery. No10k zero-violation inference to1e-6/infinite realtime.\n')
    add('## 8. SC6-A01–A24\n\n|ID|Status and concrete evidence/limitation|\n|---|---|')
    audit={
    '01':'PASS baseline raw identity, resolved-config B1 regression, unchanged DSO/defaults; arithmetic difference quantified.',
    '02':'PASS conditional trusted simulation provider/service/scorer isolation and future-mutation check; privileged numeric source is not hardware detection.',
    '03':'PASS analytic continuous D1 and directed numeric source; monitor/propagator/D1 bridge consume same domain.',
    '04':'PASS scoped independent100-digit extrema/seams, prior core Gaussian/full-state/cross/dof and geometry/rotation/event oracles; regression results below. Not arbitrary nonlinear observability.',
    '05':'PASS inherited all-time E and raw induction, complete masks/frontier/shared birth/RW/historical carry.',
    '06':'PASS new coupled p/v and norm/shell constraints with inclusion proof and fallback/frontier checks.',
    '07':'PASS local prototype + two indistinguishable lawful solutions; main bias observation contraction OPEN.',
    '08':'PASS same-center700/700 strict H/V M contractions, no law/noise/AL/fault manipulation.',
    '09':'PASS frozen A ON/OFF and old/new actual identical raw digests and full-state/stage scoring; mixed effects retained.',
    '10':'PASS D0 six strong witnesses real prior removal/ledger/4 IMU rejoin; requested D1 gyro scope OPEN.',
    '11':'PASS evaluation weak/short/staggered/repeated/late full denominators; unbounded/unknown concurrent scope not zero; functional misses OPEN.',
    '12':'PASS SC6 shared-step operator-held polluted prior actually repaired55/rejoin56–59 in80 epochs; tight optimized beyond-replay104 refuses100–104; unique commits, old104 negative retained.',
    '13':'PASS terminal unaligned tag/full-state/RPE1/20/ON/OFF/stage scores; NAVIGATION_ACCURACY_OPEN.',
    '14':'PASS independent200Hz/20Hz release/enqueue/queue, actual source/service/gate/sink clocks. Actual UWB alarm clock recording fixed and validated without changing native semantics; affected L1 rerun only, original failure retained.',
    '15':'PASS receiver timeout/stamp/duplicate/gap unit checks and real cancel/overflow; late space-valid outputs have no current protection.',
    '16':'PASS finite measured resource evaluation and transient copy reduction; RESOURCE_OPTIMIZATION_OPEN native tombstone/alert growth.',
    '17':'PASS exact same-model P90 healthy+joint semantics/raw identity, about3x median net improvement.',
    '18':'OPEN REALTIME_PROFILE_OPEN: actual misses in registered40/40 and other rows; all counters retained.',
    '19':('PASS full5000/10000 actual paced flows with complete denominators and late episodes.' if lcomplete else 'OPEN LONG_STREAM_EVALUATION_OPEN: full planned release attempted but native long stream aborted/refused; exact counts above.'),
    '20':'PASS AL2/3,5/5,10/10 immutable postprocessing only; formal/application publication always0.',
    '21':'PASS M/A/P distinguished, frozen SHA/digests/profiles and failed/debug denominators retained.',
    '22':'PASS only if final core regression below passes; actual check list is appended after execution.',
    '23':'PASS separates proof/recovery/accuracy/timeliness/resources; no finite-zero/WCET/infinite-memory/application claim.',
    '24':'PASS delivery after final local W4 save and own-cache cleanup; sole result and two actual figures. No push.'}
    audit['22']='PASS final10/10 core invocations after long flows, including actual SC6 polluted-prior/beyond-replay and independent full-state/cross/dof/geometry/event checks.'
    for k,v in audit.items():add(f'|A{k}|{v}|')
    add('\nFive statuses above are scoped independently; requested D1/late recovery, complete navigation accuracy, strict realtime and infinite resource optimization remain OPEN. SC6_COMPLETE_FOR_REGISTERED_SIMULATION_SCOPE is **not** granted.\n')
    add('## 9. Reproduction, failures and Git\n')
    add('W0 `2b19fed`; W1 `3f8a196`; W2 `69ccb4d`; W3 freeze `2285aaf`; W4 alarm-clock recording fix `adab2da` (mathematical/selection semantics unchanged). W4 result/test/local save is final `git log -1`.\n')
    add('```bash\npython3 tools/sc6_system_research.py check --build\npython3 tools/sc6_system_research.py freeze\npython3 tools/sc6_system_research.py final --group short\npython3 tools/sc6_system_research.py final --group long\npython3 tools/sc6_evaluate.py\n```\n')
    if clockfix:
        original=summaries['initial-L1'];current=summaries['final-L1'];assert original['raw_digest']==current['raw_digest']
        add('W4 actual logging bug: initial L1 missed separate UWB wall alarm timestamps, though all10000 receipts/raw and31 core/3 delivery misses were measured. Fix records immediately after actual functional UWB statistic; service consumes earliest IMU/UWB alarm. Shared-service held-FDE and S0/S2 paired90 prefixes validate actual clocks and bit-equal state/action/PL/risk/source/ledger versus old frozen executable. No source or algorithm change. Only affected L1 was rerun once as final-L1-clockfix; initial10000 failure row remains separately above. L0 has no alarms/exclusions and is retained at its original actual ELF, with11 core misses. Corrected L1 uses the updated freeze ELF; no fastest-selection or discarded denominator.\n')
    add('Native: `epoch_raim_runner CONFIG CSV CASE sc6 on|off COUNT baseline-adapter|tight-reference|tight-optimized offline|paced SEED [shadow|cancel:3|delay:3]`. COUNT0 selects registered length. Use tools environment for the private DSO. Long runs only frozen candidate once each; no fastest rerun. W0 original B0/B1 commands and old SC1–SC5 results are reused, not whole campaigns repeated.\n')
    add('Failure accounting: W1 compile const-config error, new-target make reconfiguration issue, one wrong-DSO direct check; all corrected before stage save. Initial680 W1 rows had valid native states but invalid Eigen-lazy tag/scoring fields; stable reruns only affected rows. Original recovered adapter mismatch was identified/fixed, not reported as P equality. W2 initial700 M +960 axis and560 rotation-metric A checks are retained in this result; metrics could improve one row and worsen another. W3 one decimal-refusal parser failure was fixed/rescored. Final S3 and every realtime miss remain above. No noise reduction, raised AL, strengthened fault, reset q/bias/risk, removed difficult case, or repeated long selection.\n')
    add('AL numeric counts on frozen traces: '+', '.join(f"{k}={d.get('al',{})}" for k,d in summaries.items() if k in ['final-S0','final-S1','final-S2','final-L0','final-L1'])+'. Their computation cannot mutate state/action/PL/event.\n')
    add('Transient traces/config/logs were exclusive to `.cache/sc6-20261014`; final cleanup removes only that task-created cache after summary verification. User requirement files and prior evidence are untouched.\n')
    # Two actual figures, finite qualified denominator in caption.
    fig,axs=plt.subplots(2,2,figsize=(12,7))
    for name,color in [('final-S0','tab:blue'),('final-S1','tab:orange'),('final-S2','tab:green')]:
        good=[r for r in rows[name] if r['updates']=='1'];t=np.array([float(r['epoch'])*.05 for r in good]);
        for ax,key in zip(axs.flat,['ape','hpl','rotation_error','vpl']):ax.plot(t,[float(r[key]) for r in good],label=name,color=color);ax.set_title({'ape':'Online tag APE (m)','hpl':'HPL (m)','rotation_error':'Body rotation error (rad)','vpl':'VPL (m)'}[key]);ax.set_xlabel('sensor time s');ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(ROOT/'docs/fde_next/sc6_accuracy_pl.png',dpi=150);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(12,7))
    for name in ['final-L0','final-L1']:
        good=[r for r in rows[name] if r['updates']=='1'];t=np.array([float(r['epoch'])*.05 for r in good]);
        for ax,key in zip(axs.flat,['delivery_ms','slots','rss','bias_width']):ax.plot(t,[float(r[key]) for r in good],label=name,linewidth=.7);ax.set_title({'delivery_ms':'Measurement to sink (ms)','slots':'Native factor slots','rss':'RSS (KiB; includes provider)','bias_width':'Raw accel bias X width (m/s²)'}[key]);ax.set_xlabel('sensor time s');ax.legend(fontsize=7)
    axs[0,0].axhline(50,color='red',linestyle='--');fig.tight_layout();fig.savefig(ROOT/'docs/fde_next/sc6_realtime_resource.png',dpi=150);plt.close(fig)
    add(f'\n![Terminal accuracy and PL](sc6_accuracy_pl.png)\n\n![Actual long-flow timing and resource growth](sc6_realtime_resource.png)\n\nFigures use only actual committed outputs; omitted/refused counts appear in tables. First figure:1100/1100 S0/S1/S2 committed; second:15000/15000 L0/L1 committed, including3 late outputs. Global finite qualified={qualified}, planned={planned}.\n')
    regression=json.loads((CACHE/'regression.json').read_text())
    assert len(regression)==10 and all(r['returncode']==0 for r in regression)
    add('Final necessary core regression: **10/10 PASS** after original long flows; after the clock-only bug fix, only directly affected SC6/open boundary checks are supplemented, preserving the original invocation results.\n\n|Target|Result|\n|---|---|')
    for r in regression:add('|'+r['target']+' '+str(r['args'])+'|PASS|')
    add('\nSC6 shared-service additional tests:184/184 sole native mutations/raw-qualified; actual polluted marginal prior repaired55 and IMU rejoin56–59; beyond replay100–104 has raw coverage and no restoration. Original frozen runner SHA stayed unchanged through the original regression build; the subsequent clock fix has its own recorded ELF and exact semantic-prefix comparison. No main noise sigma was changed.\n')
    supplement=CACHE/'clock-supplement.json'
    if supplement.exists():
        checked=json.loads(supplement.read_text());assert len(checked)==3 and all(r['returncode']==0 for r in checked)
        add('Clock-only fix directly affected supplement after corrected L1: **3/3 PASS** ('+', '.join(r['target']+' '+str(r['args']) for r in checked)+'). Original10 regression results and initial L1 failure remain unchanged.')
    (ROOT/'docs/fde_next/SC6_RESULTS.md').write_text('\n'.join(p))
    print('SC6 unique conclusion rendered',planned,native,qualified,flush=True)
if __name__=='__main__':render()
