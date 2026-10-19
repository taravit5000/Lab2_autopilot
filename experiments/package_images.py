#!/usr/bin/env python3
import os,zipfile
from pathlib import Path
out=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests')))
images=[]
for folder in ['desktop','simulation','rqt_graph','comparisons']:images.extend(sorted((out/'results'/folder).glob('*.png')))
with zipfile.ZipFile(out/'All_images.zip','w',zipfile.ZIP_STORED) as z:
    for p in images:z.write(p,p.relative_to(out))
    index=out/'Screenshots.html'
    if index.exists():z.write(index,'Screenshots.html')
print(f'Packed {len(images)} images: {out}/All_images.zip')
import csv,json,hashlib
rows=list(csv.DictReader((out/'results/results.csv').open()));complete=[row for row in rows if row['Experiment_Status']=='SUCCESS']
info_path=out/'Run_information.json'
if info_path.exists():
    info=json.loads(info_path.read_text())
    workspace=Path(__file__).resolve().parents[1]
    for rel in info.get('source_sha256',{}):info['source_sha256'][rel]=hashlib.sha256((workspace/rel).read_bytes()).hexdigest()
    info.update(completed_trials=len(complete),full_desktop_trial_screenshots=3*len(complete),total_collision_events=sum(int(row['Collision_Count']) for row in complete),comparison_sheets=len(list((out/'results/comparisons').glob('*.png'))));info_path.write_text(json.dumps(info,indent=2))
files=[]
for p in out.rglob('*'):
    if not p.is_file() or p.suffix=='.zip' or p.name.startswith('.~lock.'):continue
    if any('_previous_' in part or part=='prior_outputs' for part in p.relative_to(out).parts):continue
    files.append(p)
with zipfile.ZipFile(out/'All_results.zip','w',zipfile.ZIP_DEFLATED,compresslevel=1) as z:
    for p in files:z.write(p,p.relative_to(out))
print(f'Packed {len(files)} result files: {out}/All_results.zip')
