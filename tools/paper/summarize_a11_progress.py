#!/usr/bin/env python3
"""Read-only progress aggregation for the frozen first-block captures."""
import argparse,csv,json,statistics
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence-root',type=Path,required=True)
a=p.parse_args();root=a.evidence_root.resolve()
data=[];windows=[]
fig,ax=plt.subplots(figsize=(8,4.6))
for scene in ('los','step','ramp'):
    d=root/'runs'/f'P1_{scene}_seed10101'
    with (d/'conditional_lm_calls.csv').open() as f: calls=list(csv.DictReader(f))
    with (d/'first_block_stationarity.csv').open() as f: grads=list(csv.DictReader(f))
    decreases=[float(c['error_before'])-float(c['error_after']) for c in calls]
    g=[float(s['max_scaled_gradient']) for s in grads]
    assert all(v>0 for v in decreases)
    assert all(c['accepted_state_update']=='1' for c in calls)
    assert all(c['first_try_reason']=='NOT_RUN_A11_NO_SHADOW_SOLVE' for c in calls)
    assert all(c['stationary']=='0' and c['valid']=='1' for c in grads)
    with (d/'first_block_call50_coordinate_fd.csv').open() as f: coord=list(csv.DictReader(f))[0]
    lm=json.loads((d/'discovery_failure_diagnostics.json').read_text())['last_conditional_lm']
    assert lm['check_result'] and not lm['last_qualification_stationarity']['stationary']
    item=dict(scene=scene,accepted_updates=len(calls),no_update_returns=0,
              rejected_trials=sum(int(c['rejected_lambda_trials_before_acceptance']) for c in calls),
              strict_objective_decreases=len(decreases),error50=float(calls[49]['error_after']),
              error200=float(calls[-1]['error_after']),gradient50=g[49],gradient200=g[-1],
              gradient200_over_gradient50=g[-1]/g[49],min_gradient=min(g),
              call50_max_gradient_key=coord['key'],call50_max_gradient_coordinate=int(coord['coordinate']),
              last20_gradient_median=statistics.median(g[-20:]),
              last20_min_decrease=min(decreases[-20:]),last20_max_decrease=max(decreases[-20:]),
              final_generic_check=lm['check_result'],original_stationarity_reached=lm['last_qualification_stationarity']['stationary'],
              classification='OSCILLATORY_SLOW_PROGRESS_NOT_NO_UPDATE_STALL_CHECKED_DERIVATIVES_CONSISTENT')
    data.append(item)
    for start,end in ((31,50),(81,100),(131,150),(181,200)):
        z=g[start-1:end]
        windows.append(dict(scene=scene,first_call=start,last_call=end,gradient_min=min(z),
                            gradient_median=statistics.median(z),gradient_max=max(z)))
    ax.semilogy(range(1,201),g,label=scene.upper(),lw=1.2)
ax.axhline(1e-6,color='black',ls='--',lw=.8,label='stationarity tolerance (roundoff < 3e-10)')
ax.axvline(50,color='gray',ls=':',lw=.9)
ax.set(xlabel='Authoritative iterate call',ylabel='Maximum scaled navigation gradient',
       title='A11 development only: same optimizer, frozen P1 / seed10101',xlim=(1,200))
ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower left');fig.tight_layout()
fig.savefig(root/'FIRST_BLOCK_PROGRESS.png',dpi=180);fig.savefig(root/'FIRST_BLOCK_PROGRESS.pdf')
for name,content in [('PROGRESS_SUMMARY.csv',data),('PROGRESS_WINDOWS.csv',windows)]:
    with (root/name).open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(content[0]));w.writeheader();w.writerows(content)
(root/'PROGRESS.json').write_text(json.dumps(data,indent=2,sort_keys=True)+'\n')
print(json.dumps(data,indent=2))
