#!/usr/bin/env python3
import json,os,subprocess,time
from pathlib import Path
ws=Path(__file__).resolve().parents[1]
out=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests')))
ready=Path.home()/'Documents/Lab2_Desktop_Capture/ready.json'
deadline=time.monotonic()+3600
print('Waiting for the user-approved desktop stream.',flush=True)
while time.monotonic()<deadline:
    try:
        data=json.loads(ready.read_text())
        if data.get('ready') and time.time()-ready.stat().st_mtime<30:break
    except (OSError,ValueError):pass
    time.sleep(.5)
else:raise SystemExit('Desktop capture approval still pending; results preserved.')
print('Desktop capture approved; resuming unfinished trials.',flush=True)
subprocess.run(['python3',str(ws/'experiments/run_matrix.py'),'--resume'],cwd=ws,check=True)
print('Retry matrix finished. The finalizer will export and verify the results.',flush=True)
