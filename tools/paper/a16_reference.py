#!/usr/bin/env python3
"""Only A16's sealed A15 call17 trial1 endpoint pair. Fixed 50/100 dps.
No state retraction, solve, differentiation, estimator, or precision search.
"""
import argparse,csv,json,hashlib,struct,time
from pathlib import Path
from fractions import Fraction as F
from collections import Counter,defaultdict
import a16_mpfr as mp
from a16_mpfr import M,I,dec,describe

def add(a,b):return [x+y for x,y in zip(a,b)]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def scale(s,a):return [s*x for x in a]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def mv(a,b):return [dot(row,b) for row in a]
def tr(a):return list(map(list,zip(*a)))
def mm(a,b):return [mv(tr(b),row) for row in a]
def madd(a,b):return [add(x,y) for x,y in zip(a,b)]
def ms(s,a):return [scale(s,row) for row in a]
def skew(v):
 x,y,z=v;zero=x*0;return [[zero,-z,y],[z,zero,-x],[-y,x,zero]]
def norm(v):return dot(v,v).fun('sqrt')
def rot(x):return [row[:3] for row in x[:3]]
def pos(x):return [row[3] for row in x[:3]]
def exactD(a,b):return sum((x-y)*(x+y)/2 for x,y in zip(a,b))

def read_exact(path):
 data={};count=0
 for row in csv.DictReader(path.open()):
  x=float.fromhex(row['hex']);bits=struct.unpack('Q',struct.pack('d',x))[0]
  if bits!=int(row['bits'],16):raise ValueError('hex/bits mismatch')
  k=(row['phase'],int(row['index']),row['name']);ij=(int(row['row']),int(row['column']))
  if ij in data.setdefault(k,{}):raise ValueError('duplicate exact entry')
  data[k][ij]=x;count+=1
 matrices={}
 for k,v in data.items():
  nr=max(i for i,j in v)+1;nc=max(j for i,j in v)+1
  assert len(v)==nr*nc
  matrices[k]=[[v[i,j] for j in range(nc)] for i in range(nr)]
 return matrices,count

class Evaluator:
 def __init__(self,raw,T):self.raw=raw;self.T=T;self.cache={};self.branches=[];self.intermediates=[]
 def mat(self,phase,i,name):
  key=(phase,i,name)
  if key not in self.cache:self.cache[key]=[[self.T(v) for v in row] for row in self.raw[key]]
  return self.cache[key]
 def vec(self,p,i,n):return [row[0] for row in self.mat(p,i,n)]
 def scalar(self,p,i,n):return self.mat(p,i,n)[0][0]
 def check(self,p,i,n,v):
  a=self.mat(p,i,n);flat=[x for r in v for x in r] if isinstance(v[0],list) else v;b=[x for r in a for x in r]
  assert len(flat)==len(b)
  # Native values are comparison evidence, never fed back into reference math.
  bound=max(max(abs(x) for x in (z-y).bounds()) for z,y in zip(flat,b))
  self.intermediates.append({'phase':p,'factor':i,'name':n,'native_difference_upper':dec(bound)})
 def branch(self,p,i,n,b):
  native=int(self.raw[p,i,n][0][0]);assert b==native,('branch mismatch',p,i,n,b,native)
  self.branches.append({'phase':p,'factor':i,'name':n,'reference_branch':b,'native_branch':native,'interval_certified':self.T==I})
 def log(self,R,p,i,prefix):
  trace=sum(R[j][j] for j in range(3));t3=trace-3
  if trace+1<self.scalar('CONST',0,'log_near_pi_threshold'):raise ArithmeticError('near-pi reference unsupported; stop uncertain')
  if t3<self.scalar('CONST',0,'log_near_zero_threshold'):
   theta=((trace-1)/2).fun('acos');mag=theta/(2*theta.fun('sin'));branch=2
  else:mag=self.T(.5)-t3/12+t3*t3/60;branch=1
  self.branch(p,i,prefix+'_branch',branch);self.check(p,i,prefix+'_matrix',R)
  return scale(mag,[R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]])
 def exp(self,w,p,i):
  theta2=dot(w,w);near=theta2<=self.scalar('CONST',0,'epsilon');self.branch(p,i,'exp_near_zero',int(near))
  W=skew(w);eye=[[self.T(int(j==k)) for k in range(3)] for j in range(3)]
  if near:return madd(eye,W)
  theta=theta2.fun('sqrt');K=ms(1/theta,W);KK=mm(K,K);s2=(theta/2).fun('sin')
  return madd(madd(eye,ms(theta.fun('sin'),K)),ms(2*s2*s2,KK))
 def residual(self,f,p):
  i=int(f['factor']);typ=f['type'];keys=[s.split(':')[0] for s in f['keys'].split(';')]
  def state(k):return self.mat(p,int(k[1:]),k[0].upper())
  def vector(k):return [r[0] for r in state(k)]
  def const(n):return self.vec('CONST',i,n)
  if typ=='RANGE':
   X=state(keys[0]);d=sub(add(mv(rot(X),const('lever')),pos(X)),const('anchor'))
   return [norm(d)+self.scalar('CONST',i,'beta')-self.scalar('CONST',i,'measurement')]
  if typ in ('V_PRIOR','B_PRIOR'):return sub(vector(keys[0]),const('prior'))
  if typ=='POSE_PRIOR':
   X=state(keys[0]);P=self.mat('CONST',i,'prior');R=mm(tr(rot(X)),rot(P));T=mv(tr(rot(X)),sub(pos(P),pos(X)))
   self.check(p,i,'native_relative_T',T);w=self.log(R,p,i,'pose_log');theta=norm(w);small=theta<self.scalar('CONST',0,'pose_log_threshold');self.branch(p,i,'pose_log_small',int(small))
   if small:u=T
   else:
    W=skew(scale(1/theta,w));WT=mv(W,T);tan=(theta/2).fun('tan');u=add(sub(T,scale(theta/2,WT)),scale(1-theta/(2*tan),mv(W,WT)))
   return scale(-1,w+u)
  if typ=='IMU':
   Xi=state(keys[0]);vi=vector(keys[1]);Xj=state(keys[2]);vj=vector(keys[3]);bi=vector(keys[4]);bj=vector(keys[5]);Ri,Rj=rot(Xi),rot(Xj)
   bdiff=sub(bi,const('bias_hat'));bc=add(add(const('pim'),mv(self.mat('CONST',i,'H_bias_acc'),bdiff[:3])),mv(self.mat('CONST',i,'H_bias_gyro'),bdiff[3:]));self.check(p,i,'native_bias_corrected',bc)
   dt=self.scalar('CONST',i,'dt');g=const('gravity');dt22=self.T(.5)*dt*dt
   xi=bc[:3]+add(add(bc[3:6],scale(dt,mv(tr(Ri),vi))),scale(dt22,mv(tr(Ri),g)))+add(bc[6:9],scale(dt,mv(tr(Ri),g)));self.check(p,i,'native_xi',xi)
   Rp=mm(Ri,self.exp(xi[:3],p,i));pp=add(pos(Xi),mv(Ri,xi[3:6]));vp=add(vi,mv(Ri,xi[6:9]));self.check(p,i,'native_pred_R',Rp);self.check(p,i,'native_pred_p',pp);self.check(p,i,'native_pred_v',vp)
   w=self.log(mm(tr(Rj),Rp),p,i,'imu_log')
   return w+mv(tr(Rj),sub(pp,pos(Xj)))+mv(tr(Rj),sub(vp,vj))+sub(bi,bj)
  raise ValueError('unsupported '+typ)

def fdig(q):return dec(q)
def rational_decomposition(raw,factors):
 rows=[];tot=defaultdict(F)
 for f in factors:
  i=int(f['factor']);R=[[F.from_float(x) for x in r] for r in raw['CONST',i,'whitening_R']]
  def v(p,n):return [F.from_float(r[0]) for r in raw[p,i,n]]
  du=exactD(mv(R,v('BASE','native_unwhite')),mv(R,v('TRIAL','native_unwhite')))
  dw=exactD(v('BASE','native_white'),v('TRIAL','native_white'))
  df=v('BASE','native_factor_error')[0]-v('TRIAL','native_factor_error')[0]
  d={'factor':i,'type':f['type'],'D_u':du,'D_w':dw,'D_f':df,'whitening_effect':dw-du,'factor_square_sum_effect':df-dw}
  rows.append(d)
  for k in ['D_u','D_w','D_f','whitening_effect','factor_square_sum_effect']:tot[k]+=d[k]
 eg0=F.from_float(raw['BASE',0,'native_graph_error'][0][0]);eg1=F.from_float(raw['TRIAL',0,'native_graph_error'][0][0]);tot['D_g']=eg0-eg1;tot['graph_accumulation_subtraction_effect']=tot['D_g']-tot['D_f']
 return rows,dict(tot)

def writecsv(path,rows):
 with path.open('w') as s:
  w=csv.DictWriter(s,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def run(raw,factors,root,dps,bits,T,rational):
 mp.PREC=bits;ev=Evaluator(raw,T);out=root/f'dps{dps}_{"interval" if T==I else "nearest"}';out.mkdir();factorrows=[];resrows=[];D=T(0);E0=T(0);E1=T(0);maxu=F(0);maxw=F(0)
 for f in factors:
  i=int(f['factor']);u0=ev.residual(f,'BASE');u1=ev.residual(f,'TRIAL');R=ev.mat('CONST',i,'whitening_R');w0=mv(R,u0);w1=mv(R,u1)
  assert len(u0)==int(f['dimension'])==len(u1)
  df=exactD(w0,w1);e0=dot(w0,w0)/2;e1=dot(w1,w1)/2;D+=df;E0+=e0;E1+=e1
  factorrows.append({'factor':i,'type':f['type'],**{'D_'+k:v for k,v in describe(df).items()},**{'E_base_'+k:v for k,v in describe(e0).items()},**{'E_trial_'+k:v for k,v in describe(e1).items()}})
  for p,u,w in [('BASE',u0,w0),('TRIAL',u1,w1)]:
   un=ev.vec(p,i,'native_unwhite');wn=ev.vec(p,i,'native_white')
   for j,(x,y,a,b) in enumerate(zip(u,w,un,wn)):
    ud=x-a;wd=y-b;maxu=max(maxu,*map(abs,ud.bounds()));maxw=max(maxw,*map(abs,wd.bounds()))
    resrows.append({'phase':p,'factor':i,'row':j,**{'unwhite_'+k:v for k,v in describe(x).items()},**{'white_'+k:v for k,v in describe(y).items()},**{'ref_minus_native_unwhite_'+k:v for k,v in describe(ud).items()},**{'ref_minus_native_white_'+k:v for k,v in describe(wd).items()}})
 writecsv(out/'factor_objectives.csv',factorrows);writecsv(out/'residuals.csv',resrows);writecsv(out/'branches.csv',ev.branches);writecsv(out/'intermediates.csv',ev.intermediates)
 lo,hi=D.bounds();gn=F('1.54833136618045015513e-13')
 summary={'dps':dps,'bits':bits,'arithmetic':'outward_interval' if T==I else 'nearest','D':describe(D),'E_base':describe(E0),'E_trial':describe(E1),'exact_D_bound_fractions':{'lo':[lo.numerator,lo.denominator],'hi':[hi.numerator,hi.denominator]},'D_minus_A15_stable_GN':{'lo':dec(lo-gn),'hi':dec(hi-gn,up=True)},'max_unwhite_native_difference_upper':dec(maxu,up=True),'max_white_native_difference_upper':dec(maxw,up=True),'formula_sanity_pass':maxu<=F('1e-8') and maxw<=F('1e-6'),'branch_checks':len(ev.branches),'factor_count':len(factors),'residual_count':len(resrows),'halfwidth_pass':(hi-lo)/2<=F('1e-15'),'strict_descent':lo>0,'rational_native_decomposition':{k:dec(v) for k,v in rational.items()},'unwhite_evaluation_effect':{'lo':dec(rational['D_u']-hi),'hi':dec(rational['D_u']-lo,up=True)}}
 (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)
 if not summary['formula_sanity_pass']:raise ArithmeticError('formula sanity failed; no conclusion')
 return summary

def main():
 p=argparse.ArgumentParser();p.add_argument('--endpoints',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
 raw,count=read_exact(a.endpoints/'exact_binary64.csv');factors=list(csv.DictReader((a.endpoints/'factors.csv').open()));assert len(factors)==371
 (a.output/'input_identity.json').write_text(json.dumps({'exact_binary64_sha256':hashlib.sha256((a.endpoints/'exact_binary64.csv').read_bytes()).hexdigest(),'factor_sha256':hashlib.sha256((a.endpoints/'factors.csv').read_bytes()).hexdigest(),'binary64_entries':count,'types':dict(Counter(f['type'] for f in factors)),'mpfr':mp.version().decode(),'no_other_endpoint_or_trial':True},indent=2)+'\n')
 rrows,rtot=rational_decomposition(raw,factors);writecsv(a.output/'native_rational_decomposition.csv',[{k:(dec(v) if isinstance(v,F) else v) for k,v in r.items()} for r in rrows])
 results=[]
 for dps,bits in [(50,167),(100,333)]:
  for T in [M,I]:results.append(run(raw,factors,a.output,dps,bits,T,rtot))
 def bound(r,which):n,d=r['exact_D_bound_fractions'][which];return F(n,d)
 diff=abs(bound(results[0],'lo')-bound(results[2],'lo'))
 final={'two_precision_D_difference':dec(diff,up=True),'two_precision_pass':diff<=F('1e-18'),'interval_pass':all(r['halfwidth_pass'] for r in results if r['arithmetic']=='outward_interval'),'strict_descent_both_intervals':all(r['strict_descent'] for r in results if r['arithmetic']=='outward_interval'),'formula_sanity_pass':all(r['formula_sanity_pass'] for r in results),'optimizer_iterates':0,'estimator_processes':0,'scope':'fixed A15 call17 trial1 binary64 endpoints only'}
 # Every nearest result must lie inside independently outward-rounded bounds.
 for j in [0,2]:assert bound(results[j+1],'lo')<=bound(results[j],'lo')<=bound(results[j+1],'hi')
 (a.output/'VERDICT.json').write_text(json.dumps(final,indent=2)+'\n');print(json.dumps(final),flush=True)
 return 0 if final['two_precision_pass'] and final['interval_pass'] else 2
if __name__=='__main__':raise SystemExit(main())
