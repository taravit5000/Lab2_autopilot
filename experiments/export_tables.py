#!/usr/bin/env python3
"""Export measured trials as Excel-readable tables."""
import csv,json,os,zipfile,yaml,subprocess,sys
from pathlib import Path
from xml.sax.saxutils import escape
out=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests')))
r=out/'results';rows=list(csv.DictReader((r/'results.csv').open()));dest=out/'Excel_data';dest.mkdir(exist_ok=True)
columns=['Test_ID','Speed_mps','Detection_Angle_deg','Obstacle_Distance_m','Duration_sim_seconds','Collision_Count','Map_Coverage_Percent','Map_Quality_Rating','Measured_Forward_Mean_mps','Measured_Forward_Max_mps','Experiment_Status']
def table(name,header,data):
    with (dest/(name+'.csv')).open('w',newline='') as f:
        w=csv.writer(f);w.writerow(header);w.writerows(data)
    return [header]+data
summary=table('Trial_results',columns,[[row[k] for k in columns] for row in rows]);contacts=[];motion=[];commands=[];quality=[]
for row in rows:
    folder=r/'logs'/f"test_{int(row['Test_ID']):02}"
    if row['Experiment_Status']!='SUCCESS':continue
    for e in json.loads((folder/'collisions.json').read_text()):
        contacts.append([row['Test_ID'],e['trial_time'],len(e['pairs']),json.dumps(e['pairs'])])
    d=json.loads((folder/'evidence.json').read_text());start=d['start_clock_ns']
    quality.append([row['Test_ID'],row['Speed_mps'],row['Detection_Angle_deg'],row['Obstacle_Distance_m'],d['coverage_percent'],d['rating'],d['known_cells_in_roi'],d['occupied_cells_in_roi'],d['wall_surface_completeness_percent'],d['occupied_wall_agreement_percent'],d['wall_neighbor_continuity_percent'],d['wall_tolerance_m']])
    for stamp,v,turn in d['commands']:commands.append([row['Test_ID'],round((stamp-start)/1e9,6),v,turn])
    for stamp,x,y,v in d['odometry']:motion.append([row['Test_ID'],round((stamp-start)/1e9,6),x,y,v])
collision_table=table('Collision_events',['Test_ID','Trial_time_s','Contact_pairs','Objects'],contacts)
table('Measured_motion',['Test_ID','Trial_time_s','X_m','Y_m','Forward_speed_mps'],motion)
table('Commanded_motion',['Test_ID','Trial_time_s','Forward_command_mps','Turn_command_radps'],commands)
quality_table=table('Map_quality',['Test_ID','Speed_mps','Detection_Half_Angle_deg','Obstacle_Distance_m','Map_Coverage_Percent','Calculated_Quality','Known_Cells','Occupied_Cells','Wall_Surface_Completeness_Percent','Occupied_Wall_Agreement_Percent','Wall_Neighbor_Continuity_Percent','Wall_Tolerance_m'],quality)
# A small workbook with raw values and no charts.
sheets=[('Trials',summary),('Contacts',collision_table),('Map_quality',quality_table)]
with zipfile.ZipFile(dest/'Angle_distance_results.xlsx','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('[Content_Types].xml','<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'+''.join(f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>' for i in range(1,len(sheets)+1))+'</Types>')
    z.writestr('_rels/.rels','<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>')
    z.writestr('xl/workbook.xml','<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>'+''.join(f'<sheet name="{name}" sheetId="{i}" r:id="rId{i}"/>' for i,(name,_) in enumerate(sheets,1))+'</sheets></workbook>')
    z.writestr('xl/_rels/workbook.xml.rels','<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'+''.join(f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i}.xml"/>' for i in range(1,len(sheets)+1))+'</Relationships>')
    for i,(_,data) in enumerate(sheets,1):
        text='<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>'
        for n,row in enumerate(data,1):
            text+=f'<row r="{n}">'
            for value in row:
                try: number=float(value);cell=f'<c><v>{number}</v></c>'
                except (ValueError,TypeError):cell='<c t="inlineStr"><is><t>'+escape(str(value))+'</t></is></c>'
                text+=cell
            text+='</row>'
        z.writestr(f'xl/worksheets/sheet{i}.xml',text+'</sheetData></worksheet>')
config=yaml.safe_load((Path(__file__).parent/'experiment_config.yaml').read_text())
expected=len(config['speeds'])*len(config['angles'])*len(config['obstacle_distances'])
(out/'Experiment_config.yaml').write_text(yaml.safe_dump(config))
(out/'Results_notes.txt').write_text(f"TurtleBot3 Gazebo simulation\n{expected} trials: fixed speed 0.10 m/s; detection half-angles 10, 20, 30, 40 degrees; obstacle distances 0.1, 0.2, 0.3, 0.4, 0.5 m.\nEach trial runs for 30 simulation seconds after 3 seconds warm-up.\nCollision counts come from Gazebo contact sensors, excluding ground and self contact. Contacts separated by over 0.2 seconds count as another event. Counts were not recorded by hand. No rosbag is needed.\nMeasured motion and trial tables are in Excel_data. No charts have been generated.\nDesktop screenshots show the actual native RViz2 and Gazebo windows.\nMap coverage is computed within the inner enclosure; quality ratings use the fixed geometric rubric in map_metrics.py.\n")
print(f'Exported {sum(row["Experiment_Status"]=="SUCCESS" for row in rows)} completed trials and {len(contacts)} collision events to {dest}')

subprocess.run([sys.executable,str(Path(__file__).parent/'merge_comparisons.py')],check=True)
from html import escape as html_escape
parts=['<!doctype html><meta charset="utf-8"><title>Lab 2 screenshots</title><style>body{font:16px sans-serif;max-width:1200px;margin:30px auto}img{width:100%;height:auto}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}</style><h1>Lab 2 simulation screenshots</h1><p>Actual full-desktop captures of native Gazebo and RViz2. Collision counts are measured by Gazebo contact sensors.</p><table><tr><th>Test</th><th>Commanded speed (m/s)</th><th>Half-angle (degrees)</th><th>Obstacle distance (m)</th><th>Collisions</th><th>Captures</th></tr>']
for row in rows:
    if row['Experiment_Status']!='SUCCESS':continue
    tag=f"Map_V{float(row['Speed_mps']):g}_D{float(row['Obstacle_Distance_m']):g}_A{row['Detection_Angle_deg']}"
    links=' | '.join(f'<a href="results/desktop/{tag}_{phase}.png">{phase}</a>' for phase in ['Start','Mid','End'])
    comparison=tag.replace('Map_','Comparison_')
    links+=f' | <a href="results/comparisons/{comparison}.png">Comparison</a>'
    parts.append('<tr>'+''.join('<td>'+html_escape(str(row[k]))+'</td>' for k in ['Test_ID','Speed_mps','Detection_Angle_deg','Obstacle_Distance_m','Collision_Count'])+'<td>'+links+'</td></tr>')
parts.append('</table>')
for row in rows:
    if row['Experiment_Status']=='SUCCESS':
        parts.append(f"<h2>Test {row['Test_ID']} — {row['Speed_mps']} m/s, ±{row['Detection_Angle_deg']}°, obstacle distance {row['Obstacle_Distance_m']} m</h2><a href=\"results/{row['Map_Screenshot']}\"><img loading=\"lazy\" src=\"results/comparisons/{Path(row['Map_Screenshot']).stem.replace('Map_','Comparison_').replace('_End','')}.png\"></a>")
(out/'Screenshots.html').write_text(''.join(parts))
complete=[row for row in rows if row['Experiment_Status']=='SUCCESS']
report=['TurtleBot3 obstacle avoidance in Gazebo','',f'Completed trials: {len(complete)} / {expected}','World: standard TurtleBot3 world with green towers, black enclosure walls, and nine cylindrical obstacles.','Controller: the existing autopilot, with its left-first obstacle response retained.','Conditions: fresh world for each trial; initial pose (-2.0, -0.5); 30 simulation seconds; fixed forward speed 0.10 m/s; obstacle thresholds 0.1–0.5 m; turn speed 0.3 rad/s.','Three simulation seconds of warm-up precede each timed interval (Gazebo clock 3 to 33 seconds).','Detection angles below are half-angles, measured on each side of forward.','', 'Test  Command m/s  Half-angle  Obstacle m  Contacts  Coverage %  Quality']
for row in complete:report.append(f"{int(row['Test_ID']):02}    {float(row['Speed_mps']):.2f}         {int(row['Detection_Angle_deg']):2}          {float(row['Obstacle_Distance_m']):.1f}        {int(row['Collision_Count']):2}       {float(row['Map_Coverage_Percent']):6.2f}      {row['Map_Quality_Rating']}")
report+=['','Counts are measured by Gazebo contact sensors, excluding the ground and self contact. A new event starts after a gap longer than 0.2 seconds. These are not hand-counted observations.','A separate deliberate-contact validation confirmed that the counter detects contact. Those validation events are excluded from the test matrix.','Raw event times and object names are in Excel_data/Collision_events.csv.','Commanded and measured motion samples are in separate CSV files. Forward mean/max in the trial table use odometry samples within 0.25 seconds after positive forward commands.','The modeled LiDAR minimum range is 0.12 m. The 0.10 m obstacle threshold is below that sensing limit; interpret that condition accordingly.','Map coverage is the known portion of the inner enclosure. Quality ratings use the fixed geometric rubric; they are calculated rather than manually scored.','Screenshots.html links the full desktop captures at the start, middle, and end. The images show actual native RViz2 and Gazebo windows.','No rosbag or generated result charts are used.','These are simulation results at a commanded forward speed of 0.10 m/s; they are not physical-robot measurements.','Robot specification: https://emanual.robotis.com/docs/en/platform/turtlebot3/features/']
turning_tests={str(command[0]) for command in commands if abs(float(command[3]))>1e-6}
report+=['',f"Observed contact events across completed trials: {len(contacts)}. Trials with recorded turn commands: {len(turning_tests)} of {len(complete)}."]
if complete and not turning_tests:report.append('The recorded commands stayed straight throughout these timed windows. Results apply to this starting pose, world, speed, and 30-second duration.')
retry_file=r/'failed_attempts.csv'
if retry_file.exists():
    retries=list(csv.DictReader(retry_file.open()))
    if retries:report.append(f'{len(retries)} earlier attempts were excluded because the desktop screenshot stream stopped; see results/failed_attempts.csv. Completed measurements come from successful retries.')
(out/'Lab_record.txt').write_text('\n'.join(report)+'\n')
