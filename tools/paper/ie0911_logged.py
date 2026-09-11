#!/usr/bin/env python3
"""Run one command into a new evidence prefix, preserving every exit status."""
import json,sys,subprocess,time
from pathlib import Path
prefix=Path(sys.argv[1]); command=sys.argv[2:]
prefix.parent.mkdir(parents=True,exist_ok=True)
with prefix.with_suffix('.command.json').open('x') as f: json.dump(command,f)
t=time.monotonic()
with prefix.with_suffix('.log').open('x') as f:
    try:
        r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
        code=r.returncode
    except OSError as error:
        f.write(str(error)+'\n');code=126
with prefix.with_suffix('.result.json').open('x') as f:
    json.dump({'command':command,'exit_code':code,'wall_seconds':time.monotonic()-t},f,indent=2)
print(prefix,code)
sys.exit(code)
