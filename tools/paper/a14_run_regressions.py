#!/usr/bin/env python3
"""Fixed engineering test list. No datasets, scientific runs or gate selection."""
import json,subprocess,sys
from pathlib import Path
r=Path(sys.argv[1]);binroot=Path('/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo')
tests=['test_config','test_imu_preint','test_graph_builder','test_paper_input','test_paper_methods','test_paper_stage2_cache','test_nlos_discovery','test_nlos_refit','test_nlos_inference','test_nlos_recoverability']
results=[]
for name in tests:
 cmd=[sys.executable,'tools/paper/a14_run_logged.py','--record',str(r/'commands'/name),'--seconds','120','--',str(binroot/name),'--gtest_output=xml:'+str((r/(name+'.xml')).resolve())]
 result=subprocess.run(cmd);results.append({'name':name,'exit_code':result.returncode})
# Pure command/identity fixture contract; this uses its mock runner, not an estimator.
cmd=[sys.executable,'tools/paper/a14_run_logged.py','--record',str(r/'commands/test_t09_runner_contract'),'--seconds','120','--',sys.executable,'-B','test/test_t09_runner_contract.py']
result=subprocess.run(cmd);results.append({'name':'test_t09_runner_contract','exit_code':result.returncode,'kind':'mock_runner_engineering_fixture'})
(r/'REGRESSION_EXITS.json').write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(0 if all(x['exit_code']==0 for x in results) else 1)
