#!/usr/bin/env python3
"""Presentation-only views of immutable SC8 summaries. Never runs estimators."""
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/sc8/presentation'
BLUE='#216A9B'; OFF='#8BB7CC'; ORANGE='#C36A2D'; RED='#B24F54'; GREY='#858B91'; TEAL='#148477'
COL={'O-FULL':BLUE,'O-NOFDE':OFF,'O-FDE-ENG':BLUE,'O-ENG-NOFDE':OFF,'S-ONLINE':ORANGE,'S-FINAL':ORANGE,'O-NOM':GREY}
MARK={'O-FULL':'o','O-NOFDE':'x','S-ONLINE':'s','S-FINAL':'D'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load():
    metrics=json.loads((ROOT/'docs/sc8/SC8_METRICS.json').read_text())
    data=json.loads((ROOT/'docs/sc8/SC8_PLOTDATA.json').read_text())
    audit=json.loads((ROOT/'docs/sc8/SC8_AUDIT.json').read_text())
    for name,key in [('SC8_METRICS.json','metrics_sha256'),('SC8_PLOTDATA.json','plotdata_sha256')]:
        assert sha(ROOT/'docs/sc8'/name)==audit[key],name+' changed since final SC8 audit'
    assert metrics['registered']==143 and metrics['logical_run_status_counts']=={'SUCCESS':130,'RESOURCE_LIMIT':12,'FAIL':1}
    assert metrics['status']=='SC8_EVALUATED_WITH_EXECUTION_GAPS'
    return metrics,data

def run(m,group,stream,method='O-FULL',seed=20261020,window=None):
    return next(r for r in m['runs'] if r['group']==group and r['stream']==stream and r['requested_method']==method and r.get('seed')==seed and (window is None or r['window']==window))

def views(r):return r['views']
def error(r,key='ape_m',view=None):return views(r)[view or next(iter(views(r)))][key]['rmse']

def style():
    plt.rcParams.update({'font.family':'Liberation Sans','font.size':11,'axes.titlesize':12,'axes.labelsize':11.5,
        'xtick.labelsize':10.5,'ytick.labelsize':10.5,'legend.fontsize':10.5,'svg.fonttype':'none',
        'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,
        'figure.facecolor':'white','axes.facecolor':'white','savefig.facecolor':'white','svg.hashsalt':'SC8-advisor-frozen-v1'})

def save_manifest(charts):
    """Also bind native diagrams/tables to the actual sources read for them."""
    native=[]
    for slide,paths,metric,caveat in [
        (1,['research/sc6_service.cpp','research/rolling_policy.hpp','docs/fde_next/SC7_RESULTS.md','docs/sc8/SC8_RESULTS.md'],'One FGO / one commit and rolling position certificate; registered 20/200 Hz and 30/30 policy','Historical documentation supplies architecture only; real raw unqualified.'),
        (2,['config/sc8_experiments.yaml','docs/sc8/SC8_METRICS.json'],'E00 evidence lanes, six recording durations, 143 outcomes / 159 attempts + ROS','SUCCESS is process exit; physical references and CAL boundaries remain open.'),
        (14,['docs/sc8/SC8_METRICS.json','docs/sc8/SC8_RESULTS.md'],'E00–E08 evidence-to-claim synthesis','No empirical risk certification, real qualification or omitted-study substitution.'),
        (15,['docs/requirements/sc8_advisor_slides/SC8_ADVISOR_SLIDES_REQUIREMENTS.md','docs/sc8/presentation/STORYBOARD.json'],'Four advisor questions / requested decisions','Proposed next actions, not executed future validation.')]:
        native.append(dict(slide=slide,rendering='Native editable PowerPoint shapes / text / connectors',source_path=paths,data_digest={p:sha(ROOT/p) for p in paths},section='Architecture' if slide==1 else 'E00' if slide==2 else 'SC8 synthesis' if slide==14 else 'Requirements slide15',displayed_metric=metric,caveat=caveat))
    payload=dict(schema='sc8-advisor-figure-sources-v1',frozen_solver='61907ab417450d317e443e4ff10f2cce062deabe',figures=charts,native_visual_sources=native)
    (OUT/'FIGURE_SOURCES.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')

def figures():
    m,d=load();style();dest=OUT/'figures';dest.mkdir(parents=True,exist_ok=True);manifest=[]
    def finish(fig,name,slide,original,description,caveat):
        # Plain numeric log ticks avoid tiny exponent glyphs on projected slides.
        for ax in fig.axes:
            if ax.get_xscale() in ['log','symlog']:ax.xaxis.set_major_formatter(FuncFormatter(lambda v,p:f'{v:g}'))
            if ax.get_yscale() in ['log','symlog']:ax.yaxis.set_major_formatter(FuncFormatter(lambda v,p:f'{v:g}'))
        fig.canvas.draw();renderer=fig.canvas.get_renderer();width,height=fig.canvas.get_width_height()
        for artist in [*fig.axes,*fig.legends]:
            b=artist.get_tightbbox(renderer)
            assert b is None or (b.x0>=-1 and b.y0>=-1 and b.x1<=width+1 and b.y1<=height+1),(name,'cropped plot text/axes',list(b.bounds),width,height)
        for ext in ['svg','png']:
            p=dest/(name+'.'+ext);fig.savefig(p,dpi=300,metadata={'Date':None} if ext=='svg' else {})
            if ext=='svg':p.write_text('\n'.join(s.rstrip() for s in p.read_text().splitlines())+'\n')
        plt.close(fig)
        manifest.append(dict(slide=slide,asset=name,source_path=['docs/sc8/SC8_METRICS.json','docs/sc8/SC8_PLOTDATA.json'],
            data_digest={s:sha(ROOT/s) for s in ['docs/sc8/SC8_METRICS.json','docs/sc8/SC8_PLOTDATA.json']},
            original_figure=str(original),original_figure_sha256=sha(ROOT/'docs/sc8/figures'/original),
            section='SC8_RESULTS '+description,displayed_metric=description,caveat=caveat,
            rendering='Replotted from frozen summary / actual count bins; SVG with editable text and 300dpi PNG fallback.',
            axes_and_legend_bounds_checked=True,
            outputs={ext:dict(path=str((dest/(name+'.'+ext)).relative_to(ROOT)),sha256=sha(dest/(name+'.'+ext))) for ext in ['svg','png']}))
    # E01: complete per-run scores, without substituting common-grid scores.
    fig,ax=plt.subplots(figsize=(7.55,3.55),layout='constrained')
    for j,method in enumerate(['O-FULL','O-NOFDE','S-ONLINE','S-FINAL']):
        vals=[]
        for seed in [20261020,20261021,20261022]:
            rec=run(m,'E01','SC8-H','SFUISE' if method.startswith('S-') else method,seed)
            vals.append(error(rec,view=method))
        ax.scatter(vals,np.arange(3)+(j-1.5)*.13,label=method,color=COL[method],marker=MARK[method],s=52,facecolors='none' if method=='S-FINAL' else None,linewidths=1.5)
    ax.set(yticks=range(3),yticklabels=['20261020','20261021','20261022'],xlabel='Tag APE RMSE (m), no fitted alignment',ylabel='Independent source / noise seed',xlim=(0,.355));ax.invert_yaxis()
    ax.legend(ncol=2,loc='upper center',bbox_to_anchor=(.5,1.23),frameon=False)
    finish(fig,'s03_healthy',3,'F01_paired_accuracy.svg','E01: three healthy seeds, per-run output-stamp APE','Different initialization information and output stamps; S-FINAL is same-run noncausal valid history, not batch optimum.')
    # E02: qualification-aware own-Vicon example; no unavailable FULL trace.
    fig,axs=plt.subplots(1,2,figsize=(7.5,3.55))
    fig.subplots_adjust(left=.09,right=.99,bottom=.18,top=.73,wspace=.29)
    truth=d['reference_tracks']['R3'];p=np.asarray(truth['p']);t=np.asarray(truth['t'])
    axs[0].plot(p[:,0],p[:,1],color='black',lw=1.7,label='GT (segmented CAL)');axs[1].plot(t,p[:,2],color='black',lw=1.7)
    for c in d['curves'].values():
        if c['stream']!='R3':continue
        p=np.asarray(c['estimate']);t=np.asarray(c['t']);mask=np.r_[False,np.diff(t)>.100000001];p=p.copy();p[mask]=np.nan
        axs[0].plot(p[:,0],p[:,1],color=COL[c['view']],ls='--' if c['view']=='S-FINAL' else '-',label=c['view'],lw=1)
        axs[1].plot(t,p[:,2],color=COL[c['view']],ls='--' if c['view']=='S-FINAL' else '-',lw=1)
    axs[0].set(xlabel='Tag x (m)',ylabel='Tag y (m)',title='Own Vicon R3: TEST XY');axs[0].set_aspect('equal',adjustable='datalim')
    axs[1].set(xlabel='Sensor time (s)',ylabel='Tag height z (m)',title='TEST starts at 16.679 s')
    fig.legend(*axs[0].get_legend_handles_labels(),ncol=2,loc='upper center',bbox_to_anchor=(.5,1.01),fontsize=10.5,frameon=False)
    finish(fig,'s04_vicon',4,'F02_real_trajectories.svg','E00/E02: R3 XY and height on registered TEST','GT-assisted TEST-involved segmented calibration; no FULL TEST output; no independent blind-test claim.')
    # E02 temporal coverage; missing R3 remains NA.
    fig,ax=plt.subplots(figsize=(7.6,3.45),layout='constrained')
    for j,method in enumerate(['O-FULL','O-NOFDE']):
        vals=[]
        for i in range(1,7):
            score=next(iter(run(m,'E02',f'R{i}',method)['views'].values()))
            c=score.get('output_coverage_on_GT_TEST_grid',score.get('reference_proxy_scores',{}).get('output_coverage_on_GT_TEST_grid'))
            vals.append(c['fraction']*100 if c else np.nan)
        y=np.arange(6)+(j-.5)*.32;ax.barh(y,vals,height=.29,color=BLUE if j==0 else OFF,label='O-FDE-ENG' if j==0 else 'O-ENG-NOFDE')
        for yi,v in zip(y,vals):
            ax.text((v+1) if np.isfinite(v) else 1,yi,f'{v:.2f}%' if np.isfinite(v) else 'NA: no complete TEST output',va='center',fontsize=11,color=GREY if not np.isfinite(v) else '#222222')
    ax.set(yticks=range(6),yticklabels=['R1 Walk2','R2 Walk3','R3 Own, no obstacle','R4 Own, obstacle','R5 STAR 3D','R6 STAR fast'],xlim=(0,113),xlabel='Covered / planned GT TEST stamps (%)');ax.invert_yaxis();ax.legend(loc='upper center',bbox_to_anchor=(.5,1.19),ncol=2,frameon=False)
    finish(fig,'s05_coverage',5,'F00_admission.svg','E02: output coverage on full GT TEST grid','Temporal coverage only for Walk; R3 FULL NA, no real raw PL; percentages are not accuracy or commit rates.')
    # E03 matched gains, all signs and all three seeds retained.
    fig,axs=plt.subplots(1,2,figsize=(8.1,3.45),layout='constrained')
    for ax,key,label in [(axs[0],'ape_m','OFF − ON tag APE RMSE (m)'),(axs[1],'are_rad','OFF − ON body ARE RMSE (rad)')]:
        for i,stream in enumerate(['SC8-U','SC8-A','SC8-G','SC8-J']):
            pp=[p for p in m['paired'] if p['group']=='E03' and p['stream']==stream]
            assert len(pp)==3
            vv=[p['gains'][key]['off_minus_on'] for p in pp]
            ax.scatter(vv,i+np.linspace(-.13,.13,3),c=BLUE,s=32);ax.plot(np.mean(vv),i,marker='|',markersize=20,color='#111111',mew=2)
        ax.axvline(0,color='#333333',lw=.9);ax.set(yticks=range(4),yticklabels=['UWB only','Accel','Gyro','Joint'],xlabel=label);ax.invert_yaxis();ax.margins(x=.12)
    finish(fig,'s06_gains',6,'F04_fault_paired_gains.svg','E03: matched per-seed gains, N=3 each; black tick is mean','Negative is deterioration. Position and attitude effects separate; real physical paired gains remain unavailable.')
    # E03 actual SC8-J first-seed timeline.
    fig,axs=plt.subplots(3,1,figsize=(8.9,3.6),sharex=True,layout='constrained',gridspec_kw={'height_ratios':[1,1,.7]})
    for c in d['curves'].values():
        if c['stream']!='SC8-J':continue
        axs[0].plot(c['t'],c['ape'],color=COL[c['view']],ls='--' if c['view']=='S-FINAL' else '-',lw=1,label=c['view'])
        axs[1].plot(c['t'],c['are'],color=COL[c['view']],ls='--' if c['view']=='S-FINAL' else '-',lw=1)
    axs[0].set_yscale('symlog',linthresh=.1);axs[0].set_ylim(bottom=0);axs[0].set_ylabel('Tag APE (m)');axs[1].set_ylabel('Body ARE (rad)')
    diag=d['native_diagnostics']['E03-SC8-J-O-FULL-20261020-30x30'];t=np.asarray(diag['t'])
    for i,(key,label) in enumerate([('imu_alarm','IMU FD'),('uwb_alarm','UWB isolation'),('bridge','Bridge'),('repair','Prior repair'),('rejoined','Rejoined state')]):
        yy=np.asarray(diag[key]);axs[2].scatter(t[yy==1],np.full(np.sum(yy==1),i),marker='|',s=22,color=BLUE)
    axs[2].set(yticks=range(5),yticklabels=['IMU FD','UWB isolation','Bridge','Prior repair','Rejoined state'],xlabel='Sensor time (s)',xlim=(0,30))
    for ax in axs:ax.axvspan(6.005,6.505,color=RED,alpha=.15);ax.axvspan(6.05,6.2,color=TEAL,alpha=.18)
    axs[0].legend(ncol=4,loc='upper center',bbox_to_anchor=(.5,1.42),frameon=False,fontsize=10.5)
    finish(fig,'s07_fault_timeline',7,'F03_joint_fault_receipts.svg','E03: SC8-J seed20261020 APE/ARE and native receipt states','T-SIM; source pulses fixed. Rejoined flag persists and is not proof of accuracy recovery; full SFUISE outlier scale retained.')
    # E04: actual 64x64 PE/PL count bins, no resampling into invented points.
    def stanford(axis,width,name,slide):
        fig,axs=plt.subplots(2,2,figsize=(width,3.55),layout='constrained')
        cm=LinearSegmentedColormap.from_list('counts',['#dcefea',TEAL,'#075447'])
        maximum=max(np.max(v['layers']['timely']) for a in d['stanford'].values() for v in a[axis].values())
        for i,method in enumerate(['O-FULL','O-NOFDE']):
            for j,view in enumerate(['full','zoom']):
                h=d['stanford'][method][axis][view];ax=axs[i,j];x=np.array(h['x_edges']);y=np.array(h['y_edges']);layers=h['layers']
                assert sum(np.sum(v) for v in layers.values())+h['outside']==h['total']
                ax.pcolormesh(x,y,np.ma.masked_equal(np.array(layers['timely']).T,0),norm=LogNorm(1,maximum),cmap=cm,rasterized=True)
                late=np.array(layers['late']).T;yy,xx=np.where(late>0);ax.scatter((x[xx]+x[xx+1])/2,(y[yy]+y[yy+1])/2,s=9,color=GREY,marker='s')
                lim=min(x[-1],y[-1]);ax.plot([0,lim],[0,lim],'--',color='#333333',lw=.8)
                for al in ([2,5,10] if axis=='H' else [3,5,10]):
                    if al<=y[-1]:ax.axhline(al,color=RED,ls=':',lw=.8)
                    if al<=x[-1]:ax.axvline(al,color=RED,ls=':',lw=.8)
                ax.set(xlim=(x[0],x[-1]),ylim=(y[0],y[-1]),title=f'{method} / {view}',xlabel=f'{axis}PE (m)',ylabel=f'{axis}PL (m)')
                ax.text(.04,.94,f'N={h["total"]:,}; outside={h["outside"]}',transform=ax.transAxes,va='top',fontsize=10.5)
        finish(fig,name,slide,f'F05_{axis}_Stanford.svg',f'E04: {axis} PE vs PL actual log-count bins, full + zoom','Timely bins teal (darker=more), late bins gray. Figure N=10200 per method; overall 77131 correlated receipts is a separate aggregate.')
    stanford('H',12.2,'s08_horizontal',8);stanford('V',8.55,'s09_vertical',9)
    fig,ax=plt.subplots(figsize=(3.25,1.9),layout='constrained')
    for i,al in enumerate(['2/3','5/5']):
        vv=[100*run(m,'E01','SC8-H',seed=s)['integrity']['joint_AL'][al]['timely_within_AL']/600 for s in [20261020,20261021,20261022]]
        ax.scatter(i+np.linspace(-.08,.08,3),vv,c=BLUE,s=26)
    ax.set(xticks=[0,1],xticklabels=['2 / 3 m','5 / 5 m'],ylabel='Timely joint (%)',ylim=(0,105),xlabel='Research HAL / VAL')
    finish(fig,'s09_availability',9,'F06_AL_and_SI.svg','E04: 3 healthy seeds, timely joint / all 600 planned epochs','Diagnostic AL only; no estimates/PL changed; no certified application requirement.')
    # E05: all seven unique configurations, complete common domain checked.
    fig,axs=plt.subplots(1,3,figsize=(12.2,2.45))
    fig.subplots_adjust(left=.075,right=.995,bottom=.24,top=.79,wspace=.45)
    wr=[r for r in m['runs'] if r['group']=='E05']
    for stream,col,mark,label in [('SC8-WH',BLUE,'o','Healthy'),('SC8-WJ',RED,'^','Repeated fault')]:
        for coupled,ls in [(True,'-'),(False,'--')]:
            rr=sorted([r for r in wr if r['stream']==stream and ((r['window'][0]==r['window'][1]) if coupled else r['window'][0]==300)],key=lambda r:r['window'][0] if coupled else r['window'][1])
            assert all(r['window_common_coverage']['complete_pose_interval'] for r in rr)
            x=[r['window'][0] if coupled else r['window'][1] for r in rr]
            lab=label+(' LF=LI' if coupled else ' LF=300')
            axs[0].plot(x,[error(r) for r in rr],marker=mark,color=col,ls=ls,label=lab)
            axs[1].plot(x,[r['timing']['core_ms']['p95'] for r in rr],marker=mark,color=col,ls=ls)
        rr=[r for r in wr if r['stream']==stream]
        axs[2].scatter([r['timing']['core_ms']['p95'] for r in rr],[r['window_common_summary']['hpl']['mean'] for r in rr],s=[r['resources']['rss']['max']/1024*.45 for r in rr],c=col,marker=mark,alpha=.65,label=label)
    axs[0].set(xlabel='LF=LI, or LI (LF=300) (epochs)',ylabel='Tag APE RMSE (m)',ylim=(0,.125))
    fig.legend(*axs[0].get_legend_handles_labels(),fontsize=10.5,ncol=4,loc='upper center',bbox_to_anchor=(.5,1.02),frameon=False,handlelength=1.3,columnspacing=.8)
    axs[1].axhline(40,color=RED,ls=':',lw=1);axs[1].set(xlabel='Window (20 Hz UWB epochs)',ylabel='Measured core P95 (ms)',yscale='log')
    axs[2].set(xlabel='Measured core P95 (ms)',ylabel='Common mean HPL (m)',xscale='log',ylim=(0,2.9));axs[2].text(.04,.06,'Bubble area: sampled RSS\n1000 epochs = 50 s\nFixed LF=300: LI=30/100/300',transform=axs[2].transAxes,fontsize=10.5)
    finish(fig,'s10_window',10,'F07_window_frontier.svg','E05: 14 actual window runs, common epochs1101–2100','LF, LI and replay coupled; same seed/stream, no universal trend; resource markers are sampled RSS, not peak.')
    # E06: complete-service CDF, not sum of stage percentiles.
    fig,axs=plt.subplots(1,3,figsize=(12.2,2.5),layout='constrained')
    for ax,phase in zip(axs[:2],['Healthy','Fault']):
        for method in ['O-FULL','O-NOFDE','S-ONLINE']:
            for kind,ls in [('core','-'),('sink','--')]:
                a=d['latency'][phase+'|'+method+'|'+kind];ax.plot(a['x'],a['cdf'],color=COL[method],ls=ls,lw=1.3,label=method if kind=='core' else None)
        ax.axvline(40,color=RED,ls=':',lw=.8);ax.axvline(50,color=GREY,ls=':',lw=.8)
        ax.set(xlabel='Whole-service latency (ms)',ylabel='Empirical CDF',title=phase+': solid core / dashed sink',xscale='symlog',xlim=(0,None),ylim=(0,1.04));ax.legend(loc='lower right',frameon=False)
    merged={'Source':['source_ms'],'IMU + graph':['imu_fd_ms','graph_prepare_ms'],'Candidates':['candidate_ms'],'Proof':['proof_ms','finalize_ms'],'Commit + receipt':['commit_ms','state_receipt_ms'],'Cleanup + other':['maintenance_ms','envelope_cleanup_ms','abort_guard_and_clock_overhead']}
    groups=[]
    for stream in ['SC8-H','SC8-J']:
        rr=[r for r in m['runs'] if r['stream']==stream and r['group'] in ['E01','E03'] and r['requested_method']=='O-FULL']
        groups.append({k:float(np.mean([r['exclusive_means_ms'].get(k,0) for r in rr])) for k in {k for keys in merged.values() for k in keys}})
        for r in rr:assert abs(sum(r['exclusive_means_ms'].values())-r['exclusive_mean_core_ms'])<1e-8
    b=np.zeros(2)
    for (label,keys),col in zip(merged.items(),[BLUE,TEAL,'#92A2B5','#B7CBD4','#6D8F9F','#CFD7DA']):
        vv=[sum(g[k] for k in keys) for g in groups];axs[2].bar(['Healthy','Joint'],vv,bottom=b,color=col,label=label);b+=vv
    axs[2].set(ylabel='Exclusive mean service (ms)',title='O-FULL: N=3 seeds / load',ylim=(0,54));axs[2].legend(ncol=2,fontsize=10.5,loc='upper right',frameon=False,columnspacing=.4,handlelength=1)
    finish(fig,'s11_latency',11,'F08_latency_and_breakdown.svg','E06: all measured E01/E03 main core/sink latencies and exclusive means','CDF counts are frame descriptions, not independent trials; SFUISE has different capabilities/cutoffs; E06 performance repeats listed separately.')
    # E07: matched full trajectory / PL / P, no fabricated rejected G2 scores.
    fig,ax=plt.subplots(figsize=(8.1,3.55),layout='constrained');table=[];percent=[];labels=[]
    rr=[r for r in m['runs'] if r['group']=='E07'];cc={c['id']:c for c in m['ablation_comparisons']}
    for r in rr:
        c=cc.get(r['id'],{});a=c.get('A_common',{});p=c.get('PL_common',{});tim=c.get('P_common',{})
        raw=[a.get('ablated',{}).get(k,{}).get('rmse') for k in ['ape_m','are_rad']]+[p.get(k,{}).get('ablated',{}).get('mean') for k in ['hpl','vpl']]+[tim.get('ablated',{}).get('p95')]
        base=[a.get('full',{}).get(k,{}).get('rmse') for k in ['ape_m','are_rad']]+[p.get(k,{}).get('full',{}).get('mean') for k in ['hpl','vpl']]+[tim.get('full',{}).get('p95')]
        table.append(raw);percent.append([100*(v-b)/b if v is not None and b else np.nan for v,b in zip(raw,base)])
        labels.append(r['stream'].replace('SC8-','')+' / '+{'O-ABL-SUPPORT':'support off','O-ABL-JOINT':'joint + support off','O-ABL-G2':'old numerical recovery'}[r['requested_method']])
    arr=np.array(percent);limit=np.nanmax(abs(arr));ax.imshow(arr,cmap='RdBu_r',vmin=-limit,vmax=limit,aspect='auto')
    for i in range(5):
        for j in range(5):ax.text(j,i,'NA' if table[i][j] is None else f'{table[i][j]:.4g}\n{arr[i,j]:+.1f}%',ha='center',va='center',fontsize=12,color='white' if np.isfinite(arr[i,j]) and abs(arr[i,j])>.55*limit else '#222222')
    ax.set(yticks=range(5),yticklabels=labels,xticks=range(5),xticklabels=['APE (m)','ARE (rad)','HPL (m)','VPL (m)','P95 (ms)']);ax.grid(False)
    finish(fig,'s12_ablation',12,'F09_ablations.svg','E07: raw ablation values and signed matched-FULL percent changes','G2 no qualified output. Joint+sample0/support combined reversion; M same-center witness distinct from full-trajectory A and measured P.')
    # E08: preserve every existing event/extremum point, absolute time and blank tail.
    fig,axs=plt.subplots(3,1,figsize=(8.75,3.55),sharex=True,layout='constrained');ar=axs[0].twinx();ar.set_ylabel('Body ARE (rad)',fontsize=11.5)
    for rid,a in d['native_diagnostics'].items():
        if not rid.startswith('E08'):continue
        rec=next(r for r in m['runs'] if r['id']==rid);method=rec['requested_method'];col=COL[method];ls=':' if rec['stream']=='SC8-L0' else '-';label=rec['stream'].replace('SC8-','')+' '+method;t=np.array(a['t'])
        axs[0].plot(t,a['ape'],c=col,ls=ls,lw=1,label=label);ar.plot(t,a['rotation_error'],c=col,ls='--',lw=.8,alpha=.5)
        axs[1].plot(t,a['core_rolling_p95'],c=col,ls=ls,lw=1);axs[2].plot(t,np.array(a['rss'])/1024,c=col,ls=ls,lw=1)
    for ax in axs:
        ax.set_xlim(0,500)
        for k in [121,1121,3121,5121,7121,9121]:ax.axvline(k*.05,c=RED,lw=.7,alpha=.5)
    axs[0].set_ylabel('Tag APE (m)');axs[1].set_ylabel('P95 (ms)');axs[2].set(ylabel='RSS (MiB)',xlabel='Absolute sensor time (s); dashed on right axis = body ARE')
    axs[0].legend(ncol=3,loc='upper center',bbox_to_anchor=(.5,1.48),frameon=False,fontsize=10.5)
    finish(fig,'s13_long',13,'F10_long_streams.svg','E08: all saved long-stream APE/ARE, rolling core P95 and sampled RSS','Actual extrema/events from frozen compact data; FULL ends7171; late episode not confirmed; sampled RSS not process peak.')
    save_manifest(manifest)
    print('Presentation figures:',len(manifest),'SVG + PNG pairs')

if __name__=='__main__':figures()
