#!/usr/bin/env python3
"""Final archive/provenance checks; only existing fixed-pair results."""
from pathlib import Path
import csv,json,hashlib,re,shutil
from fractions import Fraction as F
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 source=ROOT/'doc/ie_sprint/evidence/t10_a15_terminal_numeric_20260909T152633Z/static/direction_summary.csv'
 r=csv.DictReader(source.open());rows=[x for x in r if 'call17_first_rejected' in x.values()];assert len(rows)==1
 values=[v for v in rows[0].values() if v=='1.54833136618045015513e-13'];assert len(values)==1
 p=E/'inputs/A15_selected/direction_summary.csv'
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=r.fieldnames);w.writeheader();w.writerows(rows)
 (E/'A15_GN_COMPARATOR_PROVENANCE.json').write_text(json.dumps({'original':str(source),'original_sha256':sha(source),'selected_sha256':sha(p),'stable_GN_exact_archival_string':values[0],'other_directions_recomputed':False},indent=2)+'\n')
 checks={}
 for name in ['dps50_interval','dps100_interval']:
  s=json.loads((E/'reference'/name/'summary.json').read_text());factors=list(csv.DictReader((E/'reference'/name/'factor_objectives.csv').open()));lo=sum(F(r['D_lo']) for r in factors);hi=sum(F(r['D_hi']) for r in factors)
  # Both enclose the same real total; their intersection must be nonempty.
  glo=F(*s['exact_D_bound_fractions']['lo']);ghi=F(*s['exact_D_bound_fractions']['hi']);checks[name+'_factor_total_interval_overlap']=max(lo,glo)<=min(hi,ghi)
  checks[name+'_positive_total']=glo>0
 seal=json.loads((E/'ENDPOINT_SEAL.json').read_text());checks['sealed_files_unchanged']=all(sha(E/'endpoints'/p)==h for p,h in seal['files'].items())
 missing=[]
 docs=[ROOT/'doc/ie_sprint/T10_A16_PROTOCOL.md',ROOT/'doc/ie_sprint/T10_A16_SOLVER_AMENDMENT_DRAFT.md',E/'VERIFICATION.md',E/'FORMULAS_AND_ERROR_BOUND.md']
 for p in docs:
  for target in re.findall(r'\]\(([^)]+)\)',p.read_text()):
   if target.startswith(('http://','https://','#')):continue
   q=(p.parent/target.split('#')[0]).resolve()
   if not q.exists():missing.append({'doc':str(p),'target':target})
 checks['new_document_local_links_valid']=not missing
 cp=E/'source/repo/tools/paper/a16_closeout_checks.py';shutil.copy2(__file__,cp)
 cmds=[]
 for p in sorted(E.glob('*/command.json')):
  j=json.loads(p.read_text());cmds.append({'record':str(p.relative_to(E)),'exit_code':j.get('exit_code'),'argv':j['argv']})
 (E/'COMMAND_INDEX.json').write_text(json.dumps(cmds,indent=2)+'\n')
 result={'checks':checks,'missing_links':missing,'pass':all(checks.values()),'new_precision_or_residual_evaluation':'NOT_RUN'}
 (E/'CLOSEOUT_CHECKS.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['pass']
if __name__=='__main__':main()
