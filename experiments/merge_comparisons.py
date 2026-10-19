#!/usr/bin/env python3
"""Pair full desktop evidence with each trial's measured summary."""
import csv,os
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
out=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests')))
r=out/'results';dest=r/'comparisons';dest.mkdir(exist_ok=True)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26)
small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',21)
count=0
for row in csv.DictReader((r/'results.csv').open()):
    if row['Experiment_Status']!='SUCCESS':continue
    source=r/row['Map_Screenshot']
    with Image.open(source) as frame:
        frame=frame.convert('RGB');width,height=frame.size
        panel=Image.new('RGB',(width,height+170),'white');panel.paste(frame,(0,85));draw=ImageDraw.Draw(panel)
        draw.text((20,8),f"Test {row['Test_ID']} | Speed {float(row['Speed_mps']):.2f} m/s | Angle ±{row['Detection_Angle_deg']}° | Obstacle distance {float(row['Obstacle_Distance_m']):.2f} m",font=font,fill='black')
        draw.text((20,47),'Comparison sheet: original full desktop capture with measured summary',font=small,fill='black')
        draw.text((20,height+98),f"Collision events: {row['Collision_Count']}     Map coverage: {float(row['Map_Coverage_Percent']):.2f}%     Calculated map quality: {row['Map_Quality_Rating']}     Duration: {float(row['Duration_sim_seconds']):.3f} s",font=font,fill='black')
        draw.text((20,height+135),'Gazebo simulation; counts from contact sensors. Original screenshot preserved separately.',font=small,fill='black')
        panel.save(dest/(source.stem.replace('Map_','Comparison_').replace('_End','')+'.png'))
    count+=1
print(f'Saved {count} comparison sheets')
