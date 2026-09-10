#!/usr/bin/env python3
import sys,json,shutil,struct
from pathlib import Path
from fractions import Fraction as F
import a17_certificate_worker as old
from a16_mpfr import I
from a16_reference import read_exact,mv
E=Path(sys.argv[1]);source=E/'cpp_tests_4/nonzero_native';dest=E/'nonzero_old_reference';shutil.copytree(source,dest)
with (dest/'exact_binary64.csv').open('a') as f:f.write('CONST,0,min_model_fidelity,0,0,'+(.001).hex()+','+struct.pack('>d',.001).hex()+'\n')
r=old.evaluate(dest,True);cpp=json.loads((source/'certificate.json').read_text());checks={}
for n in ['P','D']:
 lo,hi=F(cpp[n+'_lo']),F(cpp[n+'_hi']);a,b=F(r[n]['lo']),F(r[n]['hi']);checks[n+'_compatible']=max(lo,a)<=min(hi,b) and abs((lo+hi-a-b)/2)<=F('1e-18') and (hi-lo)/2<=F('1e-15')
checks['decision']=cpp['status']==r['decision']['status'];checks['exact_P']=r['P_exact_rational']['enclosed'];checks['nonzero_actual_conditional_beta']=read_exact(source/'exact_binary64.csv')[0]['CONST',3,'beta'][0][0]>.03
result={'passed':all(checks.values()),'checks':checks,'old_reference':r};(E/'NONZERO_GATE.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'checks':checks}));sys.exit(0 if result['passed'] else 1)
