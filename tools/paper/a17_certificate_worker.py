#!/usr/bin/env python3
"""A17 fixed-state certificate service ONLY; no solve/retract/optimizer state."""
import argparse,csv,json,math,sys,hashlib,struct,time
from pathlib import Path
from fractions import Fraction as F
from collections import defaultdict
import a16_mpfr as mp
from a16_mpfr import I,M,describe,dec
from a16_reference import read_exact,Evaluator,mv,dot,exactD,writecsv
mp.PREC=333
# A norm has nonnegative squared terms. Natural x*x loses dependency when
# x straddles zero; use its exact interval square hull for the already-supported
# zero-angle branch. This changes the enclosure, not the factor formula.
import a16_reference as reference_module
def certified_norm(v):
 total=I(0)
 for x in v:
  lo,hi=x.lo,x.hi
  lower=M(0) if lo<=0 and 0<=hi else min(lo.op('mul',lo,mp.D),hi.op('mul',hi,mp.D))
  upper=max(lo.op('mul',lo,mp.U),hi.op('mul',hi,mp.U))
  total+=I(lower,upper)
 return total.fun('sqrt')
reference_module.norm=certified_norm
POLICY='PAPER_CERTIFIED_PAIR_REDUCTION_V1'
WIDTH=F('1e-15')
class NumericFailure(Exception):pass

def choose(P,D,threshold=.001):
 if any(not mp.number(v.raw) for x in [P,D] for v in [x.lo,x.hi]):return {'status':'NUMERIC_REFERENCE_NONFINITE'}
 if any((x.hi.fraction()-x.lo.fraction())/2>WIDTH for x in [P,D]):return {'status':'NUMERIC_REDUCTION_UNRESOLVED','detail':'WIDTH'}
 if P.hi<=0 or D.hi<=0:return {'status':'REJECT','detail':'CERTIFIED_NON_DESCENT'}
 if P.lo<=0 or D.lo<=0:return {'status':'NUMERIC_REDUCTION_UNRESOLVED','detail':'SIGN'}
 ratio=I(D.lo)/I(P.hi)
 rlo=ratio.lo # RNDD lower bound of Dlo/Phi, including the division.
 rhi=D.hi.op('div',P.lo,mp.U)
 threshold=M(threshold)
 evidence={'ratio_lo':describe(I(rlo,rhi))['lo'],'ratio_hi':describe(I(rlo,rhi))['hi'],'conservative_fidelity_expression':'RNDD(D.lo/P.hi)'}
 if rhi<=threshold:return {**evidence,'status':'REJECT','detail':'CERTIFIED_LOW_FIDELITY'}
 if rlo<=threshold:return {**evidence,'status':'NUMERIC_REDUCTION_UNRESOLVED','detail':'FIDELITY'}
 q=mp.getd(rlo.raw,mp.D)
 if not math.isfinite(q):return {**evidence,'status':'NUMERIC_REFERENCE_NONFINITE','detail':'FIDELITY_CONVERSION'}
 if not (M(q)<=rlo and threshold<M(q)):return {**evidence,'status':'NUMERIC_REDUCTION_UNRESOLVED','detail':'FIDELITY_CONVERSION'}
 return {**evidence,'status':'ACCEPT','fidelity_hex':q.hex(),'fidelity_bits':struct.pack('>d',q).hex(),'conversion_le_ratio_lower':True}

class PairEvaluator(Evaluator):
 def check(self,p,i,n,v):
  # Observational intermediate differences are A16 evidence, not decision data.
  pass

def prediction(path,exact=False):
 v={};res={};seen=set();qv=defaultdict(F);qr={}
 for x in csv.DictReader(path.open()):
  k=(int(x['factor']),int(x['row']));idx=(*k,x['key'],int(x['coordinate']))
  if idx in seen:raise NumericFailure('NUMERIC_REFERENCE_UNSUPPORTED:duplicate J entry')
  seen.add(idx);a,d,r=(float.fromhex(x[n]) for n in ['A_hex','delta_hex','r_hex'])
  if not all(math.isfinite(t) for t in [a,d,r]):raise NumericFailure('NUMERIC_REFERENCE_NONFINITE:linear')
  if k in res and res[k]!=r:raise NumericFailure('NUMERIC_REFERENCE_UNSUPPORTED:RHS')
  res[k]=r;v[k]=v.get(k,I(0))+I(a)*I(d)
  if exact:qv[k]+=F.from_float(a)*F.from_float(d);qr[k]=F.from_float(r)
 out=defaultdict(lambda:I(0));qt=F(0)
 for k,w in v.items():out[k[0]]+=-I(res[k])*w-I(.5)*w*w
 if exact:qt=sum(-qr[k]*w-w*w/2 for k,w in qv.items())
 return sum(out.values(),I(0)),out,qt,len(seen)

def evaluate(directory,exact=False):
 start=time.monotonic();raw,entries=read_exact(directory/'exact_binary64.csv')
 if any(not math.isfinite(x) for a in raw.values() for row in a for x in row):raise NumericFailure('NUMERIC_REFERENCE_NONFINITE:input')
 fs=list(csv.DictReader((directory/'factors.csv').open()));P,pf,Q,nj=prediction(directory/'linear.csv',exact)
 ev=PairEvaluator(raw,I);D=I(0);rows=[]
 for f in fs:
  i=int(f['factor']);R=ev.mat('CONST',i,'whitening_R');u0=ev.residual(f,'BASE');u1=ev.residual(f,'TRIAL');w0=mv(R,u0);w1=mv(R,u1);df=exactD(w0,w1);D+=df
  rows.append({'factor':i,'type':f['type'],**{'P_'+k:v for k,v in describe(pf.get(i,I(0))).items()},**{'D_'+k:v for k,v in describe(df).items()}})
 threshold=raw['CONST',0,'min_model_fidelity'][0][0]
 decision=choose(P,D,threshold)
 result={'schema':'A17_PAIR_CERTIFICATE_V1','policy':POLICY,'bits':333,'P':describe(P),'D':describe(D),'decision':decision,'branch_count':len(ev.branches),'J_nonzero_entries':nj,'factor_count':len(fs),'exact_input_sha256':hashlib.sha256((directory/'exact_binary64.csv').read_bytes()).hexdigest(),'linear_sha256':hashlib.sha256((directory/'linear.csv').read_bytes()).hexdigest(),'wall_s':time.monotonic()-start}
 if exact:result['P_exact_rational']={'numerator':Q.numerator,'denominator':Q.denominator,'decimal':dec(Q),'enclosed':P.lo.fraction()<=Q<=P.hi.fraction()}
 writecsv(directory/'factor_certificates.csv',rows)
 if ev.branches:writecsv(directory/'branches.csv',ev.branches)
 else:(directory/'branches.csv').write_text('phase,factor,name,reference_branch,native_branch,interval_certified\n')
 (directory/'certificate.json').write_text(json.dumps(result,indent=2)+'\n');return result

def guarded(directory,exact=False):
 try:return evaluate(directory,exact)
 except Exception as e:
  msg=str(e)
  status='NUMERIC_REFERENCE_NONFINITE' if 'NONFINITE' in msg or 'nonfinite' in msg or 'NaN' in msg else 'NUMERIC_REFERENCE_UNSUPPORTED'
  r={'policy':POLICY,'bits':333,'decision':{'status':status,'detail':type(e).__name__+':'+msg}};(directory/'certificate.json').write_text(json.dumps(r,indent=2)+'\n');return r

def main():
 p=argparse.ArgumentParser();p.add_argument('--serve',action='store_true');p.add_argument('--directory',type=Path);p.add_argument('--exact-regression',action='store_true');a=p.parse_args()
 if a.serve:
  for line in sys.stdin:
   path=line.strip()
   if path=='QUIT':break
   r=guarded(Path(path));d=r['decision'];print(d['status']+' '+d.get('fidelity_hex','0x0p+0'),flush=True)
  return 0
 r=guarded(a.directory,a.exact_regression);print(json.dumps(r,indent=2));return 0 if r['decision']['status'] in ['ACCEPT','REJECT'] else 2
if __name__=='__main__':raise SystemExit(main())
