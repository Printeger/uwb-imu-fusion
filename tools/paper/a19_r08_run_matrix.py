#!/usr/bin/env python3
"""Serial, no-retry, truth-free R08 matrix runner with a durable budget ledger."""
import argparse, datetime, hashlib, json, os, subprocess, time
from pathlib import Path
POLICY='PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1'
def now(): return datetime.datetime.now(datetime.timezone.utc).isoformat()
def write(path,value):
 path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
 tmp=path.with_suffix(path.suffix+'.tmp'); tmp.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n'); tmp.replace(path)
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--evidence-root',type=Path,required=True); ap.add_argument('--runner',type=Path,required=True)
 ap.add_argument('--initial-consumed-s',type=float,default=0.0)
 ap.add_argument('--matrix-name',default='RUN_MATRIX.json')
 ap.add_argument('--ledger-name',default='BUDGET_LEDGER.json')
 ap.add_argument('--ticket-prefix',default='T10-A19-R08')
 a=ap.parse_args()
 root=a.evidence_root.resolve(); runner=a.runner.resolve(); matrix=json.loads((root/a.matrix_name).read_text())
 ledger={'schema':'T10_A19_R08_DURABLE_BUDGET_LEDGER_V1','scheduled_limit_s':10800.0,'total_allocation_s':14400.0,
  'reserve_not_automatically_consumed_s':3600.0,'per_cell_reservation_s':700.0,'parallelism':1,
  'initial_engineering_generation_prepare_s':a.initial_consumed_s,'actual_consumed_s':a.initial_consumed_s,'entries':[]}
 ledger_path=root/a.ledger_name; write(ledger_path,ledger)
 for ordinal,cell in enumerate(matrix['order'],1):
  run_id=f"{cell['base']}_{cell['scenario']}"; run_dir=root/'attempts'/run_id; output=run_dir/'output'
  entry={'ordinal':ordinal,'run_id':run_id,'base':cell['base'],'scenario':cell['scenario'],'reserved_s':700.0,
         'status':'RESERVED','reserved_utc':now(),'attempt_count':1}
  if ledger['actual_consumed_s']+700.0>ledger['scheduled_limit_s']:
   entry.update(status='NOT_RUN_RESOURCE',reason='REMAINING_SCHEDULED_BUDGET_LT_700S'); ledger['entries'].append(entry); write(ledger_path,ledger); continue
  run_dir.mkdir(parents=True,exist_ok=False)
  ticket={'schema':'T10_A19_R08_VALIDATION_TICKET_V1','ticket_id':f'{a.ticket_prefix}-{ordinal:02d}',
          'reservation_key':json.loads(Path(cell['context']).read_text())['reservation_key'],'scenario':cell['scenario'],
          'initialization':'ORIGINAL_RAW_NO_CHECKPOINT','attempt_count':1,'consumed':False,'issued_utc':now()}
  write(run_dir/'TICKET.json',ticket)
  command=['/usr/bin/time','-v','-o',str(run_dir/'resources.txt'),'/usr/bin/timeout','--signal=KILL','--kill-after=5s','700s',
           '/usr/bin/strace','-f','-e','trace=openat,open,execve','-o',str(run_dir/'file_access.trace'),str(runner),
           '--policy',POLICY,'validation-stage1-stage2-score',cell['config'],'ORIGINAL_RAW_NO_CHECKPOINT',cell['context'],str(output)]
  write(run_dir/'command.json',{'argv':command,'cwd':os.getcwd(),'started_utc':now(),'runner_sha256':sha(runner)})
  ticket['consumed']=True; ticket['consumed_utc']=now(); write(run_dir/'TICKET_CONSUMPTION.json',ticket)
  start=time.monotonic()
  with (run_dir/'stdout.log').open('wb') as stdout,(run_dir/'stderr.log').open('wb') as stderr:
   code=subprocess.run(command,stdout=stdout,stderr=stderr).returncode
  elapsed=time.monotonic()-start; ledger['actual_consumed_s']+=elapsed
  trace=(run_dir/'file_access.trace').read_text(errors='replace')
  forbidden=[line for line in trace.splitlines() if any(token in line.lower() for token in ('/evaluation/','range_truth','motion.csv','ground_truth'))]
  entry.update(status='COMPLETE' if code==0 else 'FAILED',exit_code=code,elapsed_s=elapsed,finished_utc=now(),
               forbidden_truth_open_count=len(forbidden),forbidden_truth_open_lines=forbidden[:20])
  ledger['entries'].append(entry); write(ledger_path,ledger)
  write(run_dir/'RUN_RESULT.json',entry)
  if forbidden:
   ledger['stop_reason']='TRUTH_EXPOSURE_SHARED_CORRECTNESS'; write(ledger_path,ledger); break
 print(json.dumps({'entries':len(ledger['entries']),'complete':sum(e['status']=='COMPLETE' for e in ledger['entries']),
                   'failed':sum(e['status']=='FAILED' for e in ledger['entries']),'actual_consumed_s':ledger['actual_consumed_s']}))
if __name__=='__main__': main()
