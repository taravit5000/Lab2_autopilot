#!/usr/bin/env python3
"""Run the angle and obstacle-distance test matrix."""
import argparse, csv, json, math, os, signal, subprocess, time, traceback, fcntl, shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from map_metrics import evaluate
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage

WS=Path(__file__).resolve().parents[1]; PKG=WS/'src/autopilot'; OUTPUT=Path(os.environ.get('LAB2_OUTPUT_DIR',str(Path.home()/'Documents/Lab2_Angle_Distance_Tests'))); RESULTS=OUTPUT/'results'
FIELDS=['Test_ID','Speed_mps','Detection_Angle_deg','Obstacle_Distance_m','Duration_sim_seconds','Duration_wall_seconds','Collision_Count','Map_Coverage_Percent','Map_Quality_Rating','Map_Screenshot','RQT_Graph_Screenshot','Experiment_Status','Notes','Measured_Forward_Mean_mps','Measured_Forward_Max_mps']
def atomic_json(path,data):
    temp=path.with_suffix('.tmp'); temp.write_text(json.dumps(data,indent=2)); temp.replace(path)
def write_csv(rows,path):
    temp=path.with_suffix('.tmp')
    with temp.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=FIELDS); writer.writeheader(); writer.writerows(rows)
    temp.replace(path)

def update_counter(path,node,test_id,speed,angle,distance):
    elapsed=0 if node.start_ns is None else (node.now_ns-node.start_ns)/1e9
    count=sum(node.start_ns is not None and node.start_ns/1e9<=e['sim_time']<=node.end_ns/1e9 for e in node.events)
    atomic_json(path,dict(test=test_id,speed=speed,angle=angle,distance=distance,elapsed=round(elapsed,1),collisions=count))

def desktopshot(node,path):
    service=Path.home()/'Documents/Lab2_Desktop_Capture'
    if not (service/'ready.json').exists():raise RuntimeError('Desktop capture service unavailable')
    previous=(service/'ready.json').stat().st_mtime_ns
    path.unlink(missing_ok=True)
    (service/'request.txt').write_text(str(path.resolve()))
    try:node.wait(lambda:path.exists() and (service/'ready.json').stat().st_mtime_ns>previous and json.loads((service/'ready.json').read_text()).get('last_capture')==str(path.resolve()),15)
    except TimeoutError as exc:raise TimeoutError('Full-desktop capture stream did not deliver a frame') from exc
    from PIL import Image
    with Image.open(path) as im:
        im.load()
        assert im.size==(1920,1080),'Expected full monitor screenshot'


class Monitor(Node):
    def __init__(self):
        super().__init__('experiment_monitor'); self.now_ns=0; self.grid=None; self.scan=None
        self.odom=[]; self.commands=[]; self.events=[]; self.start_ns=None; self.end_ns=None
        self.last_contact=-1e9; self.contact_messages=0; self.tf_count=0
        self.subs=[self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data),
          self.create_subscription(OccupancyGrid,'/map',self.map_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)),
          self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data),
          self.create_subscription(Odometry,'/odom',self.odom_cb,100),
          self.create_subscription(Twist,'/cmd_vel',self.cmd_cb,100),
          self.create_subscription(TFMessage,'/tf',self.tf_cb,100)]
        for link in ['base_link','base_scan','wheel_left_link','wheel_right_link','caster_back_link']:
            self.subs.append(self.create_subscription(ContactsState,'/contacts/'+link,self.contact_cb,100))
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
    def clock(self,m): self.now_ns=m.clock.sec*10**9+m.clock.nanosec
    def map_cb(self,m): self.grid=m
    def tf_cb(self,m): self.tf_count+=len(m.transforms)
    def odom_cb(self,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if self.start_ns is not None and self.start_ns <= stamp <= self.end_ns:
            self.odom.append([stamp,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x])
    def cmd_cb(self,m):
        if self.start_ns is not None and self.start_ns<=self.now_ns<=self.end_ns:
            self.commands.append([self.now_ns,m.linear.x,m.angular.z])
    def contact_cb(self,m):
        self.contact_messages+=1
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        pairs=[]
        for s in m.states:
            a,b=s.collision1_name,s.collision2_name
            if 'ground_plane' in a or 'ground_plane' in b: continue
            if ('burger::' in a)==('burger::' in b): continue
            pairs.append([a,b])
        if pairs:
            if stamp-self.last_contact>.2:
                self.events.append(dict(sim_time=stamp,pairs=pairs))
            self.last_contact=stamp
    def spin_for(self,seconds):
        stop=time.monotonic()+seconds
        while time.monotonic()<stop: rclpy.spin_once(self,timeout_sec=.02)
    def wait(self,predicate,timeout=60):
        stop=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>stop: raise TimeoutError('ROS condition timed out')
            rclpy.spin_once(self,timeout_sec=.02)
    def call(self,typ,name,request,timeout=30):
        client=self.create_client(typ,name)
        if not client.wait_for_service(timeout_sec=timeout): raise TimeoutError(name+' unavailable')
        fut=client.call_async(request); self.wait(fut.done,timeout); result=fut.result(); self.destroy_client(client)
        return result
    def enabled(self,value):
        req=SetParameters.Request(); req.parameters=[Parameter('enabled',value=value).to_parameter_msg()]
        result=self.call(SetParameters,'/autopilot/set_parameters',req)
        assert all(r.successful for r in result.results),str(result)
    def step(self,n):
        target=self.now_ns+n*1000000
        subprocess.run(['gz','world','-w','lab2','-m',str(n)],check=True,timeout=15,stdout=subprocess.DEVNULL)
        self.wait(lambda:self.now_ns>=target,timeout=max(30,n*.004))
        if self.now_ns!=target: raise RuntimeError(f'Unexpected clock {self.now_ns} != {target}')
        self.spin_for(.15)

class Processes:
    def __init__(self,folder): self.folder=folder; self.procs=[]
    def start(self,name,args,env=None):
        env=dict(os.environ if env is None else env)
        for key in ['GIO_LAUNCHED_DESKTOP_FILE','GIO_LAUNCHED_DESKTOP_FILE_PID','SNAP_APP_DESKTOP_FILE','CHROME_DESKTOP','DESKTOP_STARTUP_ID']:env.pop(key,None)
        f=(self.folder/(name+'.log')).open('w'); proc=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        self.procs.append((proc,f)); return proc
    def close(self):
        for proc,f in reversed(self.procs):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try: proc.wait(timeout=3)
                    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
            f.close()

def create_world(folder,contact_check=False):
    tree=ET.parse(PKG/'worlds/turtlebot3_world.world'); world=tree.getroot().find('world')
    model=ET.parse(PKG/'config/burger_contacts.sdf').getroot().find('model'); model.set('name','burger')
    model.find('pose').text='-2.0 -0.5 .01 0 0 0'; world.append(model)
    path=folder/'world.sdf'; tree.write(path,encoding='unicode'); return path

def diagnostics(node,path):
    data={'nodes':node.get_node_names_and_namespaces(),'topics':{}}
    for name,types in node.get_topic_names_and_types():
        data['topics'][name]={'types':types,'publishers':[dict(node=x.node_name,namespace=x.node_namespace) for x in node.get_publishers_info_by_topic(name)],'subscribers':[dict(node=x.node_name,namespace=x.node_namespace) for x in node.get_subscriptions_info_by_topic(name)]}
    atomic_json(path,data)

def save_map(grid,tag,speed,angle):
    a=np.array(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
    origin=[grid.info.origin.position.x,grid.info.origin.position.y]
    np.savez_compressed(RESULTS/'map_data'/f'{tag}.npz',data=a,resolution=grid.info.resolution,origin=origin,stamp=[grid.header.stamp.sec,grid.header.stamp.nanosec])
    meta=dict(image=f'{tag}.pgm',resolution=float(grid.info.resolution),origin=origin+[0.0],negate=0,occupied_thresh=.65,free_thresh=.196)
    (RESULTS/'map_data'/f'{tag}.yaml').write_text(yaml.safe_dump(meta))
    from PIL import Image
    pixels=np.where(a<0,205,np.where(a>=65,0,254)).astype(np.uint8)
    Image.fromarray(np.flipud(pixels)).save(RESULTS/'map_data'/f'{tag}.pgm')
    evidence=evaluate(a,float(grid.info.resolution),origin)
    coverage=evidence['coverage_percent']; rating=evidence['rating']
    return coverage,rating,evidence


def trial(test_id,speed,angle,distance,contact_check=False,pilot=False,review=False):
    tag=f'{"Review" if review else "Map"}_V{speed:g}_D{distance:g}_A{angle}'; folder=RESULTS/'logs'/('review' if review else 'contact_validation' if contact_check else 'pilot' if pilot else f'test_{test_id:02}')
    if folder.exists() and any(folder.iterdir()):
        folder.rename(folder.with_name(folder.name+'_previous_'+str(time.time_ns())))
    folder.mkdir(parents=True,exist_ok=True)
    if not contact_check and not review:
        # Remove stale outputs before retrying.
        prior=[RESULTS/'maps'/f'{tag}.png',RESULTS/'maps/rendered'/f'{tag}.png',
               RESULTS/'rqt_graph'/f'RQT_Graph_V{speed:g}_D{distance:g}_A{angle}.png',
               RESULTS/'rviz'/f'RViz_Detail_V{speed:g}_D{distance:g}_A{angle}.png']
        prior += [RESULTS/'map_data'/f'{tag}.{ext}' for ext in ['npz','pgm','yaml']]
        prior += [RESULTS/'simulation'/f'Gazebo_V{speed:g}_D{distance:g}_A{angle}_{phase}.png' for phase in ['Start','Mid','End']]
        for path in prior:
            if path.exists():
                saved=folder/'prior_outputs'/path.relative_to(RESULTS);saved.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(path,saved);path.unlink()
    processes=Processes(folder); node=None
    row={k:'' for k in FIELDS}; row.update(Test_ID=f'{test_id:02}',Speed_mps=speed,Detection_Angle_deg=angle,Obstacle_Distance_m=distance,Experiment_Status='FAILED')
    began=time.monotonic()
    try:
        world=create_world(folder)
        processes.start('gazebo',['gzserver','--verbose','-u','--seed','12345','-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',str(world)])
        node=Monitor()
        node.wait(lambda:node.count_publishers('/clock')>0,45)
        # Start the clock at 1 s.
        subprocess.run(['gz','world','-w','lab2','-m','1000'],check=True,timeout=15)
        node.wait(lambda:node.now_ns>=1000000000,45); node.spin_for(.5)
        if node.now_ns!=1000000000: raise RuntimeError('initial clock epoch mismatch')
        urdf=Path('/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf').read_text()
        description=folder/'description.yaml'; description.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf,'use_sim_time':True}}}))
        processes.start('state_publisher',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(description)])
        if contact_check:
            node.step(1000)
            req=SpawnEntity.Request(); req.name='contact_probe'; req.xml='<sdf version="1.6"><model name="contact_probe"><static>true</static><link name="wall"><collision name="collision"><geometry><box><size>.1 .3 .3</size></box></geometry></collision></link></model></sdf>'
            req.initial_pose.position.x=-1.92; req.initial_pose.position.y=-.5; req.initial_pose.position.z=.15
            result=node.call(SpawnEntity,'/spawn_entity',req); assert result.success,result.status_message
            node.step(500)
            assert node.events,'Deliberate wall contact not detected'
            atomic_json(folder/'validation.json',dict(passed=True,contact_messages=node.contact_messages,events=node.events,ground_contacts_excluded=True))
            print('True contact validation PASSED',flush=True); return None
        processes.start('slam',['ros2','launch','slam_toolbox','online_async_launch.py','use_sim_time:=true',f'slam_params_file:={PKG / "config/slam.yaml"}'])
        processes.start('autopilot',[str(WS/'install/autopilot/lib/autopilot/autopilot'),'--ros-args','-p','use_sim_time:=true','-p','enabled:=false','-p',f'forward_speed:={speed}','-p',f'scan_angle:={angle}','-p',f'obstacle_distance:={distance}','-p','turn_speed:=0.3'])
        rviz_process=processes.start('rviz',['rviz2','-d',str(PKG/'config/lab2.rviz'),'--ros-args','-p','use_sim_time:=true'])
        counter_file=folder/'live_count.json'
        processes.start('counter',['python3',str(WS/'experiments/live_counter.py'),str(counter_file)])
        gazebo_request=folder/'gazebo_capture.request'
        sim_tag=f'{"Review_Gazebo" if review else "Gazebo"}_V{speed:g}_D{distance:g}_A{angle}'
        gazebo_initial=RESULTS/'simulation'/f'{sim_tag}_Start.png'
        gazebo_env=dict(os.environ,LAB2_GAZEBO_REQUEST=str(gazebo_request),LAB2_GAZEBO_CAPTURE=str(gazebo_initial),LAB2_VIEW_INDEX=str(test_id-1))
        gazebo_process=processes.start('gzclient',['gzclient','--verbose','--gui-client-plugin',str(WS/'build/gazebo_capture/liblab2_gazebo_capture.so')],env=gazebo_env)
        node.wait(lambda:node.count_subscribers('/scan')>=2,40)
        node.wait(lambda:gazebo_initial.exists(),30)
        node.step(2000); node.wait(lambda:node.grid is not None and node.scan is not None,30)
        assert len(node.scan.ranges)>0 and node.contact_messages>0
        assert node.buffer.can_transform('odom','base_scan',rclpy.time.Time()),'odom→base_scan TF missing'
        assert node.buffer.can_transform('map','base_footprint',rclpy.time.Time()),'map→base TF missing'
        parameter_request=GetParameters.Request();parameter_request.names=['forward_speed','scan_angle','obstacle_distance','turn_speed']
        values=node.call(GetParameters,'/autopilot/get_parameters',parameter_request).values
        actual=dict(forward_speed=values[0].double_value,scan_angle=values[1].integer_value,obstacle_distance=values[2].double_value,turn_speed=values[3].double_value)
        assert actual==dict(forward_speed=speed,scan_angle=angle,obstacle_distance=distance,turn_speed=.3),str(actual)
        # Capture the initial view after warm-up.
        gazebo_initial.unlink()
        gazebo_request.write_text(str(gazebo_initial))
        node.wait(lambda:gazebo_initial.exists(),15)
        node.spin_for(.3)
        subprocess.run(['python3',str(WS/'experiments/arrange_windows.py')],check=True)
        node.spin_for(6)
        update_counter(counter_file,node,test_id,speed,angle,distance)
        node.spin_for(.3)
        desktopshot(node,RESULTS/'desktop'/f'{tag}_Start.png')
        initial_grid=node.grid
        node.enabled(True); node.spin_for(.3)
        node.start_ns=node.now_ns; node.end_ns=node.start_ns+30*10**9
        node.events=[]; node.last_contact=-1e9
        wall_start=time.monotonic()
        # Advance exactly 30 s.
        step_process=subprocess.Popen(['gz','world','-w','lab2','-m','30000'],stdout=subprocess.DEVNULL)
        graph_path=RESULTS/'rqt_graph'/f'{"Review_" if review else ""}RQT_Graph_V{speed:g}_D{distance:g}_A{angle}.png'
        graph_proc=processes.start('rqt_graph',['python3',str(WS/'experiments/capture_rqt.py'),str(graph_path)])
        captured=False; midway=False; last_count_update=0
        deadline=time.monotonic()+150
        while node.now_ns<node.end_ns:
            if time.monotonic()>deadline: raise TimeoutError('30 simulation seconds exceeded wall timeout')
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last_count_update>.25:
                update_counter(counter_file,node,test_id,speed,angle,distance)
                last_count_update=time.monotonic()
            if not captured and node.now_ns-node.start_ns>=10*10**9:
                diagnostics(node,folder/'ros_graph.json'); captured=True
            if not midway and node.now_ns-node.start_ns>=15*10**9:
                gazebo_request.write_text(str(RESULTS/'simulation'/f'{sim_tag}_Mid.png'))
                desktopshot(node,RESULTS/'desktop'/f'{tag}_Mid.png')
                midway=True
        wall_duration=time.monotonic()-wall_start
        step_process.wait(timeout=5)
        assert node.now_ns==node.end_ns, f'clock overshot: {node.now_ns-node.end_ns} ns'
        node.enabled(False); node.pub.publish(Twist()); node.spin_for(1)
        # Wait for the final map while paused.
        node.spin_for(2)
        assert node.grid is not None and len(node.grid.data)>0
        assert node.odom and math.hypot(node.odom[-1][1]-node.odom[0][1],node.odom[-1][2]-node.odom[0][2])>.02,'No measured movement'
        assert node.grid.header.stamp.sec*10**9+node.grid.header.stamp.nanosec > initial_grid.header.stamp.sec*10**9+initial_grid.header.stamp.nanosec,'No SLAM map update'
        graph_proc.wait(timeout=25)
        assert graph_proc.returncode==0 and graph_path.exists(),f'rqt capture unavailable (exit {graph_proc.returncode}); see rqt_graph.log'
        from PIL import Image
        im=Image.open(graph_path); assert im.width>=800 and np.array(im).std()>5,'Empty graph image'
        coverage,rating,evidence=save_map(node.grid,tag,speed,angle)
        map_image=RESULTS/'desktop'/f'{tag}_End.png'
        gazebo_final=RESULTS/'simulation'/f'{sim_tag}_End.png'
        gazebo_request.write_text(str(gazebo_final))
        node.wait(lambda:gazebo_final.exists(),15)
        update_counter(counter_file,node,test_id,speed,angle,distance)
        node.spin_for(.5)
        desktopshot(node,map_image)
        evidence['map_capture_method']='Full monitor capture through approved GNOME ScreenCast portal; native RViz2 and Gazebo'
        evidence['desktop_screenshots']=[str((RESULTS/'desktop'/f'{tag}_{phase}.png').relative_to(RESULTS)) for phase in ['Start','Mid','End']]
        # Check the odometry time span.
        stamps=[v[0] for v in node.odom]
        evidence.update(start_clock_ns=node.start_ns,end_clock_ns=node.now_ns,duration_sim_seconds=(node.now_ns-node.start_ns)/1e9,first_odom_ns=min(stamps),last_odom_ns=max(stamps),odom_sample_span_seconds=(max(stamps)-min(stamps))/1e9,wall_duration=wall_duration,tf_messages=node.tf_count,contact_messages=node.contact_messages,map_stamp=[node.grid.header.stamp.sec,node.grid.header.stamp.nanosec])
        forward=[o[3] for o in node.odom if any(0<=o[0]-c[0]<250000000 and c[1]>0 for c in node.commands)]
        evidence['parameters']=actual; evidence['odometry']=node.odom; evidence['commands']=node.commands
        for event in node.events: event.update(test_id=test_id,speed=speed,angle=angle,distance=distance,trial_time=event['sim_time']-node.start_ns/1e9)
        events=[e for e in node.events if 0<=e['trial_time']<=30]
        atomic_json(folder/'collisions.json',events); atomic_json(folder/'evidence.json',evidence)
        notes='Full desktop capture of native RViz2 and Gazebo. Collisions measured by contact sensors; not hand counted.'
        if speed>.22: notes+=' Command exceeds physical Burger nominal 0.22 m/s; Gazebo model has no linear clamp.'
        row.update(Duration_sim_seconds=30.0,Duration_wall_seconds=round(wall_duration,4),Collision_Count=len(events),Map_Coverage_Percent=round(coverage,4),Map_Quality_Rating=rating,Map_Screenshot=f'desktop/{tag}_End.png',RQT_Graph_Screenshot=f'rqt_graph/{graph_path.name}',Experiment_Status='SUCCESS',Notes=notes,Measured_Forward_Mean_mps=round(float(np.mean(forward)),5) if forward else '',Measured_Forward_Max_mps=round(max(forward),5) if forward else '')
        atomic_json(folder/'result.json',row)
        print(f'{test_id:02}: SUCCESS v={speed} angle={angle} distance={distance} coverage={coverage:.2f}% collisions={len(events)} duration=30.000 s',flush=True)
        if review:
            print('Review paused at 30 s. Close RViz or Gazebo to end the session.',flush=True)
            while rviz_process.poll() is None and gazebo_process.poll() is None:node.spin_for(.5)
        if pilot:
            for output_folder in ['desktop','map_data','rqt_graph','simulation']:
                shutil.copytree(RESULTS/output_folder,RESULTS/'pilot_artifacts'/output_folder,dirs_exist_ok=True)
            shutil.copy2(graph_path,RESULTS/'rqt_graph/RQT_Graph_Overview.png')
            atomic_json(RESULTS/'pilot_validation.json',dict(passed=True,result=row,contact_validation=json.loads((RESULTS/'logs/contact_validation/validation.json').read_text())))
        return row
    except Exception as e:
        row['Notes']=f'{type(e).__name__}: {e}'
        (folder/'error.txt').write_text(traceback.format_exc())
        if node and rclpy.ok(): diagnostics(node,folder/'failure_diagnostics.json')
        atomic_json(folder/'result.json',row); print(f'{test_id:02}: FAILED {e}',flush=True)
        if contact_check: raise
        return row
    finally:
        if node:
            try: node.pub.publish(Twist()); node.spin_for(.1)
            except Exception: pass
        processes.close()
        if node: node.destroy_node()
        print(f'Trial process lifecycle: {time.monotonic()-began:.1f} wall seconds',flush=True)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--review',action='store_true'); parser.add_argument('--pilot',action='store_true'); parser.add_argument('--contact-check',action='store_true'); parser.add_argument('--resume',action='store_true'); parser.add_argument('--tests',type=int,nargs='+'); args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('GAZEBO_MASTER_URI')!='http://127.0.0.1:11386':
        raise SystemExit('Source experiments/env.sh: isolated localhost domain 86 and Gazebo port 11386 required')
    for folder in ['desktop','maps','simulation','rviz','map_data','rqt_graph','logs']: (RESULTS/folder).mkdir(parents=True,exist_ok=True)
    lock=(RESULTS/'runner.lock').open('w')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('Another experiment runner is active')
    signal.signal(signal.SIGTERM,lambda signum,frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init()
    try:
        if args.review:trial(0,.1,30,.3,review=True);return
        if args.contact_check: trial(0,.1,10,.1,contact_check=True); return
        if args.pilot:
            if not (RESULTS/'logs/contact_validation/validation.json').exists(): trial(0,.1,10,.1,contact_check=True)
            row=trial(1,.1,10,.1,pilot=True); write_csv([row],RESULTS/'pilot_results.csv'); return
        gate=RESULTS/'pilot_validation.json'
        if not gate.exists() or not json.loads(gate.read_text())['passed']: raise SystemExit('Complete and validate --pilot before matrix')
        configs=yaml.safe_load((WS/'experiments/experiment_config.yaml').read_text())
        matrix=[(s,a,d) for s in configs['speeds'] for d in configs['obstacle_distances'] for a in configs['angles']]
        rows=[dict.fromkeys(FIELDS,'') for _ in matrix]
        for i,(speed,angle,distance) in enumerate(matrix): rows[i].update(Test_ID=f'{i+1:02}',Speed_mps=speed,Detection_Angle_deg=angle,Obstacle_Distance_m=distance,Experiment_Status='NOT_RUN')
        dest=RESULTS/'results.csv'
        if args.resume and dest.exists():
            for row in csv.DictReader(dest.open()): rows[int(row['Test_ID'])-1]=row
        write_csv(rows,dest)
        for i,row in enumerate(rows):
            if args.tests and i+1 not in args.tests: continue
            if args.resume and row['Experiment_Status']=='SUCCESS': continue
            rows[i]=trial(i+1,float(row['Speed_mps']),int(row['Detection_Angle_deg']),float(row['Obstacle_Distance_m'])); write_csv(rows,dest)
    finally:
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
