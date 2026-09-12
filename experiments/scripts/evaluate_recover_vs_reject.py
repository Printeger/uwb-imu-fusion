#!/usr/bin/env python3
"""Independent RR metrics. GT/window data is never exported to estimator staging."""
from pathlib import Path
import math
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import run_recover_vs_reject as r


def union_intervals(intervals):
    out=[]
    for lo,hi in sorted(intervals):
        if not math.isfinite(lo+hi) or hi<lo: raise ValueError('INVALID_WINDOW')
        if out and lo<=out[-1][1]: out[-1][1]=max(hi,out[-1][1])
        else: out.append([lo,hi])
    return out


def events(rows):
    """All qualifying segments, not only the longest. Missing data breaks a run."""
    links={}; result=[]
    for row in rows: links.setdefault((row['tag_id'],row['anchor_id']),[]).append(row)
    for link, values in sorted(links.items()):
        active=[]
        def finish():
            if len(active)>=5 and active[-1]['time_s']-active[0]['time_s']>=2.:
                result.append(dict(tag_id=link[0],anchor_id=link[1],start_s=active[0]['time_s'],
                    end_s=active[-1]['time_s'],count=len(active),
                    mean_error_m=float(np.mean([v['error_m'] for v in active]))))
        for row in sorted(values,key=lambda x:x['source_row']):
            t,e=row['time_s'],row['error_m']
            good=math.isfinite(t) and math.isfinite(e) and e>.5
            if not good or (active and (t<active[-1]['time_s'] or t-active[-1]['time_s']>1.)):
                finish();active=[]
            if good: active.append(row)
        finish()
    return result


def grid(lo,hi):
    if not math.isfinite(lo+hi) or hi<lo: raise ValueError('INVALID_INTERVAL')
    return lo+np.arange(int(math.floor((hi-lo)*10))+1)/10.


def interpolate_gt(times, positions, query):
    times=np.asarray(times); positions=np.asarray(positions)
    if len(times)!=len(positions) or np.any(np.diff(times)<=0) or not np.isfinite(times).all():
        raise ValueError('NONCANONICAL_GT_TIMES')
    out=np.full((len(query),3),np.nan)
    for j,t in enumerate(query):
        k=np.searchsorted(times,t)
        if k<len(times) and times[k]==t: out[j]=positions[k]
        elif k>0 and k<len(times) and times[k]-times[k-1]<=.05:
            # A NaN endpoint deliberately keeps the whole bracket unavailable.
            u=(t-times[k-1])/(times[k]-times[k-1]);out[j]=(1-u)*positions[k-1]+u*positions[k]
    return out


def tag_trajectory(tum, lever):
    tum=np.asarray(tum,dtype=float)
    if tum.ndim!=2 or tum.shape[1]!=8 or not np.isfinite(tum).all(): raise ValueError('INVALID_TUM')
    if np.any(np.diff(tum[:,0])<=0): raise ValueError('NONCANONICAL_ESTIMATE_TIMES')
    if not np.allclose(np.linalg.norm(tum[:,4:],axis=1),1,atol=1e-6,rtol=0): raise ValueError('INVALID_QUATERNION')
    return tum[:,0], tum[:,1:4]+Rotation.from_quat(tum[:,4:]).apply(np.asarray(lever))


def nearest_estimate(times, positions, query, interval):
    times=np.asarray(times); positions=np.asarray(positions)
    out=np.full((len(query),3),np.nan)
    if not len(times):return out
    if np.any(np.diff(times)<=0):raise ValueError('NONCANONICAL_ESTIMATE_TIMES')
    for j,t in enumerate(query):
        k=np.searchsorted(times,t)
        choices=[i for i in (k-1,k) if 0<=i<len(times) and interval[0]<=times[i]<=interval[1]]
        if not choices:continue
        i=min(choices,key=lambda i:(abs(times[i]-t),times[i]))
        if abs(times[i]-t)<=.02:out[j]=positions[i]
    return out


def alignment_errors(est,gt):
    if len(est)<3:return None
    x,y=est-est.mean(0),gt-gt.mean(0)
    u,_,vt=np.linalg.svd(x.T@y)
    rot=u@np.diag([1.,1.,np.linalg.det(u@vt)])@vt
    return np.linalg.norm(x@rot-y,axis=1)


def rmse(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def matched_metrics(estimates,gt,times,windows):
    """Input arrays already associated on the frozen grid; align only once."""
    valid=np.isfinite(gt).all(1)
    for est in estimates.values(): valid &= np.isfinite(est).all(1)
    in_window=np.zeros(len(times),bool)
    for lo,hi in union_intervals(windows):in_window |= (times>=lo)&(times<=hi)
    output={}
    for name,est in estimates.items():
        errors=alignment_errors(est[valid],gt[valid])
        output[name]=dict(ATE_RMSE_m=None,ATE_P95_m=None,Window_RMSE_m=None,
                         evaluated_samples=int(valid.sum()),coverage=float(valid.mean()) if len(valid) else None,
                         window_samples=int((in_window&valid).sum()))
        if errors is not None:
            output[name].update(ATE_RMSE_m=rmse(errors),ATE_P95_m=float(np.percentile(errors,95)),
                                Window_RMSE_m=rmse(errors[in_window[valid]]))
    return output


def paired_difference(reject,recover,fallback=False):
    out={}
    for key,metric in [('delta_RR_m','ATE_RMSE_m'),('delta_RR_NLOS_m','Window_RMSE_m')]:
        a,b=reject.get(metric),recover.get(metric)
        out[key]=None if a is None or b is None else b-a
    d=out['delta_RR_m']
    out['interpretation']='NA' if d is None else ('FALLBACK_NOT_RECOVERY_BENEFIT' if fallback else
        ('RECOVER_WORSE' if d>0 else ('RECOVER_BETTER' if d<0 else 'TIED')))
    out['fallback']=fallback
    return out


def range_metrics(planned_ids, raw, hgt, offsets, final_recovered, fallback=False):
    """All dictionaries keyed by stable obs_id; no post-fit residual substitution."""
    if len(set(planned_ids))!=len(planned_ids): raise ValueError('DUPLICATE_PLANNED_ID')
    if not set(final_recovered)<=set(planned_ids): raise ValueError('RECOVERED_OUTSIDE_PLAN')
    if not set(offsets)<=set(final_recovered): raise ValueError('OFFSET_ON_UNRECOVERED_OBSERVATION')
    if any(not math.isfinite(x) or x<0 for x in offsets.values()):raise ValueError('INVALID_FINAL_OFFSET')
    before=[];after=[];rb=[];ra=[]
    for obs in planned_ids:
        if obs not in raw:raise ValueError('MISSING_RAW_OBSERVATION')
        if obs not in hgt or not math.isfinite(raw[obs]) or not math.isfinite(hgt[obs]):continue
        used=obs in final_recovered and not fallback
        if used and obs not in offsets:raise ValueError('MISSING_FINAL_OFFSET')
        e=raw[obs]-hgt[obs];corrected=e-(offsets[obs] if used else 0.)
        before.append(e);after.append(corrected)
        if used:rb.append(e);ra.append(corrected)
    return dict(range_before_RMSE_m=rmse(before),range_after_RMSE_m=rmse(after),range_count=len(before),
                recovered_before_RMSE_m=rmse(rb),recovered_after_RMSE_m=rmse(ra),recovered_range_count=len(rb))


def outcome_metrics(candidate_observations,candidate_segments,decision_segments,final_segments,fallback):
    if min(candidate_observations,candidate_segments,decision_segments,final_segments)<0:raise ValueError('NEGATIVE_COUNT')
    if not final_segments<=decision_segments<=candidate_segments:raise ValueError('INVALID_ACCEPTANCE_COUNTS')
    if fallback and final_segments:raise ValueError('FALLBACK_MUST_SUPPRESS_ALL')
    return dict(candidate_observations=candidate_observations,candidate_segments=candidate_segments,
                decision_accepted_segments=decision_segments,final_accepted_segments=final_segments,fallback=fallback,
                recovery_status='FALLBACK' if fallback else ('NO_CANDIDATES' if not candidate_segments else
                    ('ZERO_ACCEPTED' if not final_segments else 'RECOVERED')))


def save_csv(path,rows,fields=None):
    Path(path).write_bytes(r.csv_bytes(rows,fields or list(rows[0])))


def audit_recording(name,private,expected_source,estimator_status="NOT_RUN"):
    """Independent evaluator uses only author Vicon tag position and v2 anchors."""
    src=r.ROOT/'data/starloc/data'/name/'uwb.csv'
    if r.sha(src)!=expected_source:raise ValueError('EVALUATOR_SOURCE_CHANGED')
    columns=['time_s','range','from_id','to_id','tag_pos_x','tag_pos_y','tag_pos_z']
    data=pd.read_csv(src,usecols=columns)
    anchors=pd.read_csv(r.ROOT/'data/starloc/mocap/uwb_markers_v2.csv',index_col=0)
    rows=[]; gt=[]
    for i,row in data.iterrows():
        if int(row.from_id)!=1:continue
        t=float(row.time_s);pos=np.array([row['tag_pos_'+a] for a in 'xyz'])
        a=anchors.loc[int(row.to_id),['x','y','z']].to_numpy(float)
        h=float(np.linalg.norm(pos-a));z=float(row['range'])
        error=z-h if np.isfinite(pos).all() and math.isfinite(z) and z>0 else float('nan')
        rows.append(dict(source_row=i,obs_id=r.stable_id(name,i),time_s=t,tag_id=1,anchor_id=int(row.to_id),
                         range_m=z,h_GT_m=h,error_m=error))
        gt.append(dict(time_s=t,x=pos[0],y=pos[1],z=pos[2]))
    ev=events(rows); windows=union_intervals([(e['start_s'],e['end_s']) for e in ev])
    d=private/name;d.mkdir()
    save_csv(d/'range_reference.csv',rows)
    save_csv(d/'tag_gt.csv',gt)
    save_csv(d/'all_positive_error_events.csv',ev,['tag_id','anchor_id','start_s','end_s','count','mean_error_m'])
    r.write(d/'window_union.json',windows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for aid in sorted(set(v['anchor_id'] for v in rows)):
        rr=[v for v in rows if v['anchor_id']==aid]
        axes[0].plot([v['time_s'] for v in rr],[v['error_m'] for v in rr],lw=.5,label=str(aid))
    axes[0].axhline(.5,color='black',ls='--',lw=.7);axes[0].legend(ncol=8,fontsize=8)
    axes[0].set_ylabel('raw range − Vicon distance [m]')
    for e in ev:axes[1].plot([e['start_s'],e['end_s']],[e['anchor_id']]*2,lw=4)
    for lo,hi in windows:axes[1].axvspan(lo,hi,color='grey',alpha=.15)
    axes[1].set_ylabel('anchor / all qualifying events');axes[1].set_xlabel('recording time [s]')
    axes[0].set_title(name+' — evaluator-only total positive error; estimator '+estimator_status)
    fig.tight_layout();fig.savefig(d/'audit_diagnostic.png',dpi=130);plt.close(fig)
    return dict(recording=name,event_count=len(ev),window_union_count=len(windows),
                window_union_duration_s=sum(b-a for a,b in windows),reference_count=sum(math.isfinite(v['error_m']) for v in rows),
                plot=str(d/'audit_diagnostic.png'))


def evaluate(out):
    lock=r.verify(out);execution=r.read(out/'execution.json')
    if execution['lock_id']!=lock['lock_id']:raise ValueError('EXECUTION_LOCK_MISMATCH')
    if (out/'metrics.csv').exists():raise ValueError('EVALUATION_EXISTS_NO_OVERWRITE')
    private=r.WS/'evaluator_private/icra/recover_vs_reject'/out.name
    private.mkdir(parents=True,exist_ok=False)
    rows=[];pairs=[];audits=[]
    for prep in lock['rows']:
        name=prep['recording']
        audits.append(audit_recording(name,private,prep['source_sha256']['uwb.csv']))
        for method in r.METHODS:
            task=next(v for v in execution['tasks'] if v['recording']==name and v['task']==method)
            if task['status']!='NOT_RUN':raise ValueError('BACKEND_ADAPTER_NOT_ADMITTED_IN_THIS_RELEASE')
            rows.append(dict(recording=name,method=method,status=task['status'],failure=task['reason'],
                ATE_RMSE_m=None,Window_RMSE_m=None,ATE_P95_m=None,range_before_RMSE_m=None,
                range_after_RMSE_m=None,range_count=None,recovered_range_count=None,candidate_observations=None,
                candidate_segments=None,decision_accepted_segments=None,final_accepted_segments=None,
                coverage=None,fallback=None,exit_code=task['exit_code']))
        pairs.append(dict(recording=name,**paired_difference({},{}),comparison_status='NOT_RUN_GEOMETRY_OR_IMU',
                          common_samples=None,four_method_common_samples=None))
        pairs[-1]['fallback']=None
    save_csv(out/'metrics.csv',rows);save_csv(out/'paired_differences.csv',pairs)
    save_csv(out/'evaluator_audit.csv',audits)
    report=['# Recover vs Reject：准入阻塞交付', '',
        '三条冻结录制均为 **BLOCKED_GEOMETRY_OR_IMU**；科学进程 **0/15**，12方法单元全部NOT_RUN，3条配对差值NA。',
        '测量白名单准备与evaluator-only窗口枚举已执行；不把它们写成估计器、恢复或定位通过。', '',
        '## 准入原因', '',
        '作者README声明角速度在IMU系、加速度在rig系。已查的作者材料没有闭合CSV加速度的符号、测量原点及上游变换实现。',
        '厂商SDK说明不能替代作者CSV处理来源。未用GT、逐录制calib外参或测距残差猜方向；没有发布错误标注IMU系的cache。',
        's3/s4的radio tag1实际位于CAD location3；八anchor使用v2直接身份。静态beta未校正、未标定；审计误差是总误差。', '',
        '[作者README](https://github.com/utiasASRL/starloc/blob/d3ad541/README.md)、[作者论文](https://arxiv.org/html/2309.05518v1)、',
        '[厂商IMU说明](https://docs.stereolabs.com/docs/development/zed-sdk/modules/sensors/imu)。', '',
        '## 12个方法单元', '', '| recording | method | ATE RMSE | window RMSE | range before/after | status |',
        '|---|---|---|---|---|---|']
    report += ['| %s | %s | NA | NA | NA / NA | NOT_RUN: geometry/IMU |'%(v['recording'],v['method']) for v in rows]
    report += ['', '候选观测/段、decision/final接受数、覆盖率、fallback和退出码均NA，未运行不填零。完整字段见metrics.csv。', '',
        '## 配对差值', '', '| recording | Recover−Reject ATE | Recover−Reject window | RR / 四方法共同样本 |', '|---|---|---|---|']
    report += ['| %s | NA | NA | NA / NA |'%v['recording'] for v in pairs]
    report += ['', '正差值表示Recover更差；fallback不能计为恢复收益。没有科学结果支持胜负判断。', '',
        '## 独立审计诊断（不是detector support）', '', '| recording | 全部合格事件 | 去重窗口 | 并集时长s |', '|---|---:|---:|---:|']
    report += ['| %s | %d | %d | %.6f |'%(a['recording'],a['event_count'],a['window_union_count'],a['window_union_duration_s']) for a in audits]
    report += ['', '严格e>0.5m、≥2s、≥5包、gap≤1s；逐link保留全部事件，不只最长段。没有有效planned obs_id，主range比较保持NA。']
    report += ['\n![%s审计诊断](%s)'%(a['recording'],a['plot']) for a in audits]
    report += ['', '## 产物与复现', '', '[冻结协议](RECOVER_VS_REJECT_PROTOCOL.md)；[几何准入记录](recover_vs_reject_geometry.json)。',
        '输出目录：`%s`。evaluator私有目录：`%s`。'%(out,private),
        '执行命令使用 `/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py {prepare,execute,evaluate,verify} --run <上述目录>`，每阶段独立调用。',
        'prepare/execute/evaluate退出码2表示准入阻塞；verify退出码0表示hash核验通过。',
        '真实v2 cache发布、C++ prepare、PL producer/Stage2/final、SFUISE数据接入及科学运行均NOT_RUN。',
        '基础数学/报告辅助函数仅由工程fixture验证，不能称真实非空恢复或真实ROS接入通过。',
        '需要独立来源闭合CSV IMU坐标、specific-force符号与测量原点，并完成对应C++/ROS准入后才能继续科学矩阵。',
        '名单和参数不因阻塞改变。核心/默认/旧审计/历史结果未修改；T10=C2-C、T11=C、C1–C3不升级；未提交/push。', '']
    (out/'report.md').write_text('\n'.join(report))
    r.write(out/'evaluation.json',dict(status='BLOCKED_GEOMETRY_OR_IMU',private_directory=str(private),
                                    scientific_metrics='NOT_RUN',audit=audits,lock_id=lock['lock_id']))
    print(str(out/'report.md'))
    return 2
