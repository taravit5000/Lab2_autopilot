#!/usr/bin/env python3
"""Wait for the matrix, then generate and check outputs."""
import csv,fcntl,subprocess,time,os,yaml
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
OUTPUT=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests')))
config=yaml.safe_load((WS/'experiments/experiment_config.yaml').read_text())
expected=len(config['speeds'])*len(config['angles'])*len(config['obstacle_distances'])
while True:
    path=OUTPUT/'results/results.csv';lock=OUTPUT/'results/runner.lock'
    if path.exists() and lock.exists():
        rows=list(csv.DictReader(path.open()))
        if len(rows)==expected and all(r['Experiment_Status']=='SUCCESS' for r in rows):
            with lock.open('r') as f:
                try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:pass
                else:break
    time.sleep(2)
for script in ['export_tables.py','check_results.py','package_images.py']:
    subprocess.run(['python3',str(WS/'experiments'/script)],cwd=WS,check=True)
print('Raw Excel tables and screenshot audit complete.',flush=True)
