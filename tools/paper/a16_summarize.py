#!/usr/bin/env python3
"""A16 postprocess already-computed results; no residual evaluation."""
import csv,json,hashlib
from pathlib import Path
from fractions import Fraction as F
from collections import defaultdict
from a16_mpfr import dec
from a16_reference import read_exact,rational_decomposition,writecsv
E=Path(__file__).resolve().parents[2]/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
def main():
 raw,_=read_exact(E/'endpoints/exact_binary64.csv');meta=list(csv.DictReader((E/'endpoints/factors.csv').open()));nr,nt=rational_decomposition(raw,meta);rr=list(csv.DictReader((E/'reference/dps100_interval/factor_objectives.csv').open()))
 out=[];groups=defaultdict(lambda:defaultdict(F));counts=defaultdict(int)
 for n,r in zip(nr,rr):
  assert n['factor']==int(r['factor'])
  lo,hi=F(r['D_lo']),F(r['D_hi']);g=groups[n['type']];counts[n['type']]+=1;g['D_ref_lo']+=lo;g['D_ref_hi']+=hi
  d={'factor':n['factor'],'type':n['type'],'D_ref_lo':lo,'D_ref_hi':hi,'unwhite_effect_lo':n['D_u']-hi,'unwhite_effect_hi':n['D_u']-lo}
  for k in ['D_u','D_w','D_f','whitening_effect','factor_square_sum_effect']:d[k]=n[k];g[k]+=n[k]
  g['unwhite_effect_lo']+=d['unwhite_effect_lo'];g['unwhite_effect_hi']+=d['unwhite_effect_hi'];out.append({k:dec(v,up=k.endswith('hi')) if isinstance(v,F) else v for k,v in d.items()})
 writecsv(E/'reference/factor_decomposition.csv',out)
 writecsv(E/'reference/type_decomposition.csv',[{'type':t,'count':counts[t],**{k:dec(v,up=k.endswith('hi')) for k,v in g.items()}} for t,g in groups.items()])
 s=json.loads((E/'reference/dps100_interval/summary.json').read_text());lo,hi=(F(*s['exact_D_bound_fractions'][k]) for k in ['lo','hi']);gn=F('1.54833136618045015513e-13');gap=nt['D_w']-gn
 result={'D_ref':{'lo':dec(lo),'hi':dec(hi,up=True)},'D_w_minus_stable_GN':dec(gap),'D_w_minus_D_ref':{'lo':dec(nt['D_w']-hi),'hi':dec(nt['D_w']-lo,up=True)},'residual_evaluation_explained_fraction':{'lo':dec((nt['D_w']-hi)/gap),'hi':dec((nt['D_w']-lo)/gap,up=True)},'unexplained_fixed_endpoint_minus_stable_GN':{'lo':dec(lo-gn),'hi':dec(hi-gn,up=True)},'E_ref_minus_native_graph':{p:{'lo':dec(F(s['E_'+n]['lo'])-F.from_float(raw[p,0,'native_graph_error'][0][0])),'hi':dec(F(s['E_'+n]['hi'])-F.from_float(raw[p,0,'native_graph_error'][0][0]),up=True)} for p,n in [('BASE','base'),('TRIAL','trial')]},'stable_GN_recomputed_this_round':False,'scope':'A15 stable GN archival value is a comparator, not a certified interval for J or derivatives'}
 (E/'reference/DECOMPOSITION_SUMMARY.json').write_text(json.dumps(result,indent=2)+'\n')
 checks=[]
 for path,expected in json.loads((E/'before_identity.json').read_text()).items():
  p=Path(path);p=p if p.is_absolute() else E.parents[3]/p
  actual=hashlib.sha256(p.read_bytes()).hexdigest();checks.append({'path':str(p),'expected':expected,'actual':actual,'pass':actual==expected})
 for name,expected in json.loads((E/'ENDPOINT_SEAL.json').read_text())['files'].items():
  actual=hashlib.sha256((E/'endpoints'/name).read_bytes()).hexdigest();checks.append({'path':'endpoints/'+name,'expected':expected,'actual':actual,'pass':actual==expected})
 (E/'UNCHANGED_CHECKS.json').write_text(json.dumps({'pass':all(c['pass'] for c in checks),'checks':checks},indent=2)+'\n')
 assert all(c['pass'] for c in checks);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
