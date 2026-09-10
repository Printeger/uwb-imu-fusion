#!/usr/bin/env python3
"""Check A11 scope/identities after all authorized runs, without invoking estimator."""
import argparse,hashlib,json,re,subprocess,sys
from pathlib import Path
from run_a10_development_pilot import binary_identity,identity,RUNNER

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence-root',type=Path,required=True)
a=p.parse_args();root=a.evidence_root.resolve();repo=Path(__file__).resolve().parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
frozen=json.loads((root/'FROZEN_IDENTITIES.json').read_text())
assert all(sha(repo/path)==value for path,value in frozen.items())
at_run=json.loads((root/'SOURCE_AT_RUN.json').read_text())
checked=[]
for path,h in at_run.items():
    if path.endswith(('.cpp','.h')) or path=='CMakeLists.txt':
        assert sha(repo/path)==h
        checked.append(path)
parsed=0
for path in root.rglob('*.json'):
    json.loads(path.read_text(),parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s)))
    parsed+=1
commands=[json.loads(p.read_text()) for p in root.glob('commands/run_*/command.json')]
assert len(commands)==3 and all(c['exit_code']==1 and c['timeout_s']==120 for c in commands)
assert all(c['argv'].count(str(RUNNER))==1 for c in commands)
assert all(c['argv'][-3:]==['--diagnostic-conditional-lm-outer','1','--diagnostic-first-block-budget'] for c in commands)
assert json.loads((root/'BINARY_AFTER.json').read_text())['files']==binary_identity()['files']
for scene in ('los','step','ramp'):
    run=root/'runs'/f'P1_{scene}_seed10101'
    status=json.loads((run/'run_status.json').read_text())
    assert not status['stage2_refit_run'] and not status['gate_or_fallback_run']
    assert status['reason']=='A11_FIRST_BLOCK_DIAGNOSTIC_STOP_BEFORE_CHAIN: CONDITIONAL_LM_STATIONARITY_NOT_REACHED'
    for filename in ('admm_trace.csv','discovery_iterations.csv'):
        assert len((run/filename).read_text().splitlines())==1
    trace=(root/'commands'/('run_P1_'+scene+'_seed10101')/'file_access.trace').read_text()
    assert 'inputs/evaluation' not in trace and 'generation_manifest.json' not in trace
    assert 'imu.csv' in trace and 'uwb_observations.csv' in trace
expected_tests={'03_test_discovery':39,'04_test_refit':23,'05_test_inference':18,
                '06_test_paper_methods':5,'07_test_paper_stage2_cache':7}
for name,n in expected_tests.items():
    record=root/'commands'/name
    assert json.loads((record/'command.json').read_text())['exit_code']==0
    assert re.search(r'\[  PASSED  \] '+str(n)+r' tests\.',(record/'stdout.log').read_text())
assert json.loads((root/'commands/01_build/command.json').read_text())['exit_code']==2
assert json.loads((root/'commands/02_build_raw_string_fix/command.json').read_text())['exit_code']==0
check=subprocess.run(['git','diff','--check'],cwd=repo,capture_output=True,text=True)
assert check.returncode==0,check.stdout+check.stderr
marker='REVIEW_ACCEPTED_A10_SYNTHETIC_GENERATOR_AND_BOUNDED_PILOT_SCOPE'
for name in ('doc/ie_sprint/STATUS.md','doc/ie_sprint/T10_READINESS.md','paper/CLAIM_EVIDENCE.md'):
    s=(repo/name).read_text();assert marker in s and '15/18' in s and 'IN_PROGRESS' in s
summary=dict(status='PASS',frozen_files_unchanged=len(frozen),runtime_source_files_unchanged=len(checked),
             strict_json_count=parsed,estimator_processes=3,focused_tests_passed=sum(expected_tests.values()),
             first_build_failure_retained=True,git_diff_check_exit=check.returncode,
             held_out_validation_test_scheduler='NOT_RUN',python=sys.version)
(root/'FINAL_CHECKS.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
paths=[RUNNER.parent/('test_'+n) for n in ('nlos_discovery','nlos_refit','nlos_inference','paper_methods','paper_stage2_cache')]
(root/'TEST_FILE_IDENTITIES_AFTER.json').write_text(json.dumps(dict(semantics='POST_TEST_FILE_IDENTITY_NOT_PROC_MAPS',
        files=[identity(path) for path in paths]),indent=2,sort_keys=True)+'\n')
print(json.dumps(summary,indent=2))
