#!/usr/bin/env python3
"""Predeclared engineering gate and exactly one fresh first-block pilot."""
import json,hashlib,datetime,subprocess,sys,csv,shutil
from pathlib import Path
from a16_reference import read_exact
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a17_certified_prototype_20260910T002211Z';A16=ROOT/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert not (E/'PILOT_ONCE_TICKET.json').exists()
 checks={}
 for rec in ['compile_identity','link','certificate_tests_final','legacy_focused_tests','legacy_v2_counter_tests','static_regression_run','gate_checks_run','fixture_fixed_run']:
  checks[rec]=json.loads((E/rec/'command.json').read_text())['exit_code']==0
 unit=json.loads((E/'certificate_tests_final/stdout.log').read_text());checks['23_meaningful_interval_cases']=len(unit['cases'])==23 and unit['pass']
 static=json.loads((E/'STATIC_ENGINEERING_CHECKS.json').read_text());checks['static_checks']=all(v for k,v in static.items() if isinstance(v,bool))
 checks['fixture_native_counts']='PASS' in (E/'fixture_fixed/FIXTURE_GATE.txt').read_text()
 a,_=read_exact(E/'static_regression/exact_binary64.csv');b,_=read_exact(A16/'endpoints/exact_binary64.csv');checks['all_binary64_bits_including_signed_zero']=all(k in a and all(x.hex()==y.hex() for ra,rb in zip(a[k],v) for x,y in zip(ra,rb)) for k,v in b.items())
 before=json.loads((E/'before_identity.json').read_text());unchanged={}
 for name,h in before.items():
  if name.startswith('doc/ie_sprint/'):continue # A17 authorized contract addendum.
  p=Path(name);p=p if p.is_absolute() else ROOT/p;unchanged[name]=sha(p)==h
 checks['production_and_A16_source_libraries_unchanged']=all(unchanged.values())
 checks['protocol_unchanged']=sha(E/'PREREGISTERED_PROTOCOL.md')==sha(ROOT/'doc/ie_sprint/T10_A17_AMENDMENT_PROTOCOL.md')
 result={'checks':checks,'unchanged':unchanged,'pass':all(checks.values()),'recorded_before_pilot_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
 (E/'ENGINEERING_GATE.json').write_text(json.dumps(result,indent=2)+'\n');assert result['pass'],result
 # Archive final implementation before the single scientific call; no retries.
 srcs=['a17_first_block.cpp','a17_graph_io.h','a17_certificate_worker.py','a17_certificate_tests.py','a17_gate_checks.py','a17_authorize_and_run.py','a16_reference.py','a16_mpfr.py']
 identities={}
 for name in srcs:
  p=ROOT/'tools/paper'/name;q=E/'pre_pilot_source'/name;q.parent.mkdir(exist_ok=True);shutil.copy2(p,q);identities[str(p)]=sha(p)
 exe=E/'a17_first_block';identities[str(exe)]=sha(exe)
 cfg=ROOT/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml';identities[str(cfg)]=sha(cfg)
 cmd=[sys.executable,str(ROOT/'tools/paper/a14_run_logged.py'),'--record',str(E/'pilot_run'),'--seconds','120','--',str(exe),'--policy','PAPER_CERTIFIED_PAIR_REDUCTION_V1','development-first-block',str(cfg),'NO_CHECKPOINT',str(E/'pilot'),str(ROOT/'tools/paper/a17_certificate_worker.py')]
 ticket={'role':'development','process_budget':1,'max_calls':50,'timeout_s':120,'raw_initialization':True,'warm_start':False,'policy':'PAPER_CERTIFIED_PAIR_REDUCTION_V1','source_binary_config':identities,'argv':cmd,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
 with (E/'PILOT_ONCE_TICKET.json').open('x') as f:json.dump(ticket,f,indent=2);f.write('\n')
 print('ALL_ENGINEERING_GATES_PASS; consuming unique pilot ticket',flush=True)
 code=subprocess.call(cmd,cwd=ROOT)
 (E/'PILOT_COMPLETION.json').write_text(json.dumps({'exit_code':code,'retry':'NOT_RUN','completed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()},indent=2)+'\n');return code
if __name__=='__main__':raise SystemExit(main())
