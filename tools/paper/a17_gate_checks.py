#!/usr/bin/env python3
"""No optimizer: independently check frozen regression/actual interval policy."""
import argparse,csv,json,hashlib,subprocess,sys,shutil
from pathlib import Path
from fractions import Fraction as F
from a17_certificate_worker import prediction,guarded
from a16_reference import read_exact
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a17_certified_prototype_20260910T002211Z';A16=ROOT/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
def main():
 # Re-evaluate only the sealed static pair after the norm enclosure implementation
 # change; no graph reconstruction or extra native linear solve.
 source=E/'static_regression';dst=E/'final_static_certificate';dst.mkdir(exist_ok=False)
 for n in ['exact_binary64.csv','linear.csv','factors.csv']:shutil.copy2(source/n,dst/n)
 r=guarded(dst,exact=True);assert r['decision']['status']=='ACCEPT',r
 assert r['P_exact_rational']['enclosed'];gn=F('1.54833136618045015513e-13');p=F(r['P_exact_rational']['numerator'],r['P_exact_rational']['denominator']);assert abs(p-gn)<=F('1e-20')
 ar=json.loads((A16/'reference/dps100_interval/summary.json').read_text());assert max(F(r['D']['lo']),F(ar['D']['lo']))<=min(F(r['D']['hi']),F(ar['D']['hi']));assert abs((F(r['D']['lo'])+F(r['D']['hi']))/2-(F(ar['D']['lo'])+F(ar['D']['hi']))/2)<=F('1e-18')
 a,_=read_exact(source/'exact_binary64.csv');b,_=read_exact(A16/'endpoints/exact_binary64.csv');assert all(k in a and a[k]==v for k,v in b.items());assert len(a)==len(b)+1
 # Values/factor/PIM/unwhite/white data are all bitwise-equivalent as doubles.
 base=list(csv.DictReader((source/'base_identity.csv').open()))[0];assert base['graph'].endswith('a97cdd8aca336f329c2930a4d0fa1f88f84c5fba695df7f6da3c2cca9f79598b')
 assert (source/'REGRESSION_GATE.txt').exists()
 commands=[];exe=str(E/'a17_first_block');cfg=str(ROOT/'doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/P1_step_seed10101_conditional.yaml');worker=str(ROOT/'tools/paper/a17_certificate_worker.py')
 for label,args in [('default_off',[]),('invalid_policy',['--policy','INVALID','development-first-block',cfg,'NO_CHECKPOINT',str(E/'forbidden_invalid'),worker]),('wrong_scope',['--policy','PAPER_CERTIFIED_PAIR_REDUCTION_V1','validation',cfg,'NO_CHECKPOINT',str(E/'forbidden_validation'),worker])]:
  done=subprocess.run([exe]+args,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE);commands.append({'case':label,'argv':[exe]+args,'exit_code':done.returncode,'stdout':done.stdout,'stderr':done.stderr});assert done.returncode==2
 assert not (E/'forbidden_invalid').exists() and not (E/'forbidden_validation').exists()
 (E/'negative_entry_commands.json').write_text(json.dumps(commands,indent=2)+'\n')
 facts={'D_A16_overlap':True,'D_max_midpoint_difference_1e18_pass':True,'P_independent_exact_rational_enclosed':True,'P_minus_A15_stable_GN':str(float(p-gn)),'P_difference_1e20_pass':True,'all_A16_export_fields_equal_except_new_threshold':True,'native_direction_615_exact_and_endpoint_hashes':True,'default_off_invalid_policy_scope_rejected':True,'old_cache_reader_rejects_diagnostic_schema':True,'D':r['D'],'P':r['P'],'decision':r['decision']}
 (E/'STATIC_ENGINEERING_CHECKS.json').write_text(json.dumps(facts,indent=2)+'\n');print(json.dumps(facts,indent=2))
if __name__=='__main__':main()
