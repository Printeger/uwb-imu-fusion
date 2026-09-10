#!/usr/bin/env python3
import csv,json,sys,hashlib
from fractions import Fraction as F
from pathlib import Path
def binary(s):
 n,e=s.split('@');return F(int(n,2))*F(2)**int(e)
E=Path(sys.argv[1]);batch=E/sys.argv[2];rows=[]
for p in (E/'43_pair_manifest.txt').read_text().splitlines():
 p=Path(p);old=json.loads((p/'certificate.json').read_text());new=json.loads((batch/p.name/'certificate.json').read_text());checks={}
 for name in ['P','D']:
  lo,hi=F(new[name+'_lo']),F(new[name+'_hi']);a,b=F(old[name]['lo']),F(old[name]['hi']);checks[name+'_compatible']=max(lo,a)<=min(hi,b) and abs((lo+hi-a-b)/2)<=F('1e-18') and (hi-lo)/2<=F('1e-15')
 checks['decision']=old['decision']['status']==new['status'];checks['factors']=new['factor_count']==371;checks['branches']=new['branch_count']==164
 if new['status']=='ACCEPT':
  lo,hi=binary(new['fidelity_lo_binary']),binary(new['fidelity_hi_binary']);q=F.from_float(new['fidelity_binary64']);checks['fidelity']=q<=lo<=binary(new['D_lo_binary'])/binary(new['P_hi_binary']) and binary(new['D_hi_binary'])/binary(new['P_lo_binary'])<=hi and q>F.from_float(.001)
  checks['conversion_same']=new['fidelity_binary64']==float.fromhex(old['decision']['fidelity_hex'])
 for name,field in [('exact_binary64.csv','exact_input_sha256'),('linear.csv','linear_sha256')]:checks[name+'_hash']=hashlib.sha256((p/name).read_bytes()).hexdigest()==old[field]
 # Exact rational prediction independent from MPFR, all 43 pairs.
 v={};r={}
 for z in csv.DictReader((p/'linear.csv').open()):
  key=(z['factor'],z['row']);v[key]=v.get(key,F(0))+F.from_float(float.fromhex(z['A_hex']))*F.from_float(float.fromhex(z['delta_hex']));r[key]=F.from_float(float.fromhex(z['r_hex']))
 pred=sum(-r[k]*x-x*x/2 for k,x in v.items());checks['exact_rational_P']=F(new['P_lo'])<=pred<=F(new['P_hi'])
 rows.append({'pair':p.name,'checks':checks,'passed':all(checks.values())})
result={'pairs':rows,'passed':len(rows)==43 and all(x['passed'] for x in rows)};(E/(sys.argv[2]+'_checks.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'failed':[x for x in rows if not x['passed']]}));sys.exit(0 if result['passed'] else 1)
