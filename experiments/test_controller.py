#!/usr/bin/env python3
"""Check controller responses to synthetic scans."""
import math, os, subprocess, signal, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from pathlib import Path
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
os.environ['ROS_DOMAIN_ID']='87'
rclpy.init(); node=Node('controller_checks'); received=[]
pub=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
sub=node.create_subscription(Twist,'/cmd_vel',lambda m:received.append((m.linear.x,m.angular.z)),10)
exe=Path(__file__).resolve().parents[1]/'install/autopilot/lib/autopilot/autopilot'
proc=subprocess.Popen([str(exe),'--ros-args','-p','forward_speed:=0.2','-p','scan_angle:=30'],stdout=subprocess.DEVNULL)
def spin(t):
    until=time.monotonic()+t
    while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.01)
def check(name,ranges,amin,inc,expected):
    scan=LaserScan(); scan.range_min=.1; scan.range_max=5.; scan.angle_min=amin; scan.angle_increment=inc; scan.ranges=ranges
    received.clear()
    for _ in range(4): pub.publish(scan); spin(.06)
    assert received and all(abs(a-b)<1e-6 for a,b in zip(received[-1],expected)),(name,received)
    print('PASS',name)
try:
    spin(1.5)
    check('non-degree metadata left obstacle',[math.inf,math.inf,.3,math.inf,math.inf],-.6,.3,(0.,-.3))
    check('negative angle increment right obstacle',[math.inf,math.inf,math.inf,.3,math.inf],.6,-.3,(0.,.3))
    check('obstacle outside selected sector',[.2,math.inf,math.inf,math.inf,math.inf],-.8,.4,(.2,0.))
    check('NaNs ignored with valid clear rays',[math.nan,math.inf,math.inf],-.2,.2,(.2,0.))
    check('all invalid stops',[math.nan,-math.inf,0.],-.2,.2,(0.,0.))
    check('empty scan stops',[],0.,.1,(0.,0.))
    check('zero increment stops',[1.],0.,0.,(0.,0.))
    check('2pi wrap right obstacle',[math.inf,.3,math.inf],2*math.pi-.4,.2,(0.,.3))
    check('left-first tie',[.3,.3,.3],-.2,.2,(0.,-.3))
    client=node.create_client(SetParameters,'/autopilot/set_parameters'); assert client.wait_for_service(timeout_sec=3)
    req=SetParameters.Request(); req.parameters=[Parameter('scan_angle',value=200).to_parameter_msg()]; fut=client.call_async(req)
    while not fut.done(): spin(.05)
    assert not fut.result().results[0].successful; print('PASS invalid angle rejected')
    check('clear forward before watchdog',[math.inf]*3,-.2,.2,(.2,0.)); spin(1.3)
    assert received[-1]==(0.,0.); print('PASS stale scan watchdog')
finally:
    proc.send_signal(signal.SIGINT); proc.wait(timeout=5); node.destroy_node(); rclpy.shutdown()
