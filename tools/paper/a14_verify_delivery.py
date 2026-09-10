#!/usr/bin/env python3
"""Read-only closeout verification; never executes any estimator."""
import csv,hashlib,json,re,shutil,subprocess,sys
from pathlib import Path
r=Path(sys.argv[1]);checks=[]
def h(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ck(n,v,**detail):checks.append(dict(name=n,passed=bool(v),**detail))
for name in ['paper_structure.tex','v2_roadmap.md']:
 expected={'paper_structure.tex':'8ac373919823d755d9c1b4ceb807527432d16b69d368042e19aa56d93dc67144','v2_roadmap.md':'b9bb63b65ebdb8a505bf99b26181318c6c64da1a72817b08cfa61315e2333bdb'}[name];ck('frozen_'+name,h(Path('doc/v2')/name)==expected)
for name in ['nlos_solver_utils.cpp','initializer.cpp']:
 old=Path('doc/ie_sprint/evidence/t10_a13_imu_covariance_20260909T135328Z/snapshots/src')/name;ck('unchanged_'+name,h(old)==h(Path('src')/name))
for x in json.loads((r/'FINAL_SOURCE_IDENTITY.json').read_text()):ck('source_after_run_'+x['path'],h(x['path'])==x['sha256'])
for x in json.loads((r/'FINAL_BINARY_IDENTITY.json').read_text()):ck('binary_after_run_'+Path(x['path']).name,h(x['path'])==x['sha256'])
v=json.loads((r/'PILOT_SUMMARY.json').read_text());ck('unevaluated_counts_are_null',all(v[k] is None for k in ['candidate_observation_count','segment_count','group_count','eligible_count','score_unavailable_group_count']))
ck('unique_failed_run_preserved',v['scientific_run_count']==1 and v['exit_code']==1 and len(json.loads((r/'FAILURES.json').read_text()))==1)
ck('T10_IN_PROGRESS','| T10 validation/gate | `IN_PROGRESS` |' in Path('doc/ie_sprint/STATUS.md').read_text())
# Review only new/current links; previous historical link validity is not a new task.
for p in [r/'VERIFICATION.md',Path('doc/ie_sprint/T10_A14_AMENDMENT_PROTOCOL.md')]:
 for link in re.findall(r'\]\(([^)]+)\)',p.read_text()):
  if '://' in link:continue
  target=(p.parent/link.split('#')[0]).resolve();ck(str(p)+' -> '+link,target.exists())
for target in ['uwb_imu_fgo','uwb_imu_fgo_paper_runner','uwb_imu_fgo_node']:
 b=Path('/home/mint/ws_fusion_uwb/build/uwb_imu_fgo/CMakeFiles')/(target+'.dir')
 for name in ['flags.make','link.txt']:
  source=b/name;dest=r/'after/build'/target/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
shutil.copy2('/home/mint/ws_fusion_uwb/build/uwb_imu_fgo/CMakeCache.txt',r/'after/build/CMakeCache.txt')
versions={str(cmd):subprocess.check_output(cmd,text=True) for cmd in [['uname','-a'],['g++','--version'],['cmake','--version'],['git','rev-parse','HEAD']]};(r/'after/build/VERSIONS.json').write_text(json.dumps(versions,indent=2)+'\n')
(r/'FINAL_DELIVERY_CHECKS.json').write_text(json.dumps({'passed':all(x['passed'] for x in checks),'checks':checks},indent=2)+'\n')
print(json.dumps({'passed':all(x['passed'] for x in checks),'count':len(checks),'failed':[x for x in checks if not x['passed']]},indent=2));raise SystemExit(0 if all(x['passed'] for x in checks) else 1)
