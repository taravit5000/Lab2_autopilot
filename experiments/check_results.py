#!/usr/bin/env python3
import csv,json,os,hashlib
from pathlib import Path
from PIL import Image, ImageChops
import yaml
r=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests')))/'results'
config=yaml.safe_load((Path(__file__).parent/'experiment_config.yaml').read_text());rows=list(csv.DictReader((r/'results.csv').open()))
expected=len(config['speeds'])*len(config['angles'])*len(config['obstacle_distances'])
assert len(rows)==expected
assert {(float(x['Speed_mps']),int(x['Detection_Angle_deg']),float(x['Obstacle_Distance_m'])) for x in rows}=={(s,a,d) for s in config['speeds'] for d in config['obstacle_distances'] for a in config['angles']}
hashes=set()
for i,row in enumerate(rows,1):
    assert row['Experiment_Status']=='SUCCESS',(i,row['Notes'])
    evidence=json.loads((r/'logs'/f'test_{i:02}'/'evidence.json').read_text());events=json.loads((r/'logs'/f'test_{i:02}'/'collisions.json').read_text())
    assert evidence['end_clock_ns']-evidence['start_clock_ns']==30_000_000_000
    assert len(events)==int(row['Collision_Count'])
    assert evidence['parameters']['obstacle_distance']==float(row['Obstacle_Distance_m'])
    assert evidence['parameters']['scan_angle']==int(row['Detection_Angle_deg'])
    assert evidence['parameters']['forward_speed']==float(row['Speed_mps'])
    assert evidence['contact_messages']>0 and evidence['tf_messages']>0
    assert evidence['odometry'] and evidence['commands']
    for name in evidence['desktop_screenshots']:
        hashes.add(hashlib.sha256((r/name).read_bytes()).hexdigest())
        with Image.open(r/name) as im:im.load();assert im.size==(1920,1080)
    original=r/row['Map_Screenshot']
    comparison=r/'comparisons'/(original.stem.replace('Map_','Comparison_').replace('_End','')+'.png')
    with Image.open(original) as frame,Image.open(comparison) as panel:
        assert panel.size==(frame.width,frame.height+170)
        assert ImageChops.difference(frame.convert('RGB'),panel.crop((0,85,frame.width,85+frame.height)).convert('RGB')).getbbox() is None,'Screenshot pixels changed in comparison sheet'
assert len(hashes)==expected*3,'Expected three distinct full-desktop captures per test'
assert json.loads((r/'logs/contact_validation/validation.json').read_text())['passed']
(r/'verification.txt').write_text(f'PASS: {expected} measured trials; exact 30-second clock intervals; collision events reconciled with counts; live odometry, TF, and contact topics; {expected*3} full-monitor screenshots decoded at 1920x1080; comparison sheets retain original screenshot pixels.\n')
print((r/'verification.txt').read_text())
