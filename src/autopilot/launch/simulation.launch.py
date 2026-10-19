"""Launch the TurtleBot3 simulation."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '86':
        raise RuntimeError('Source lab2_ws/experiments/env.sh before simulation')
    p=Path(get_package_share_directory('autopilot'))
    robot=Path(get_package_share_directory('turtlebot3_gazebo'))
    return LaunchDescription([
        ExecuteProcess(cmd=['gzserver','--verbose','--seed','12345','-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',str(p/'worlds/turtlebot3_world.world')],output='screen'),
        Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','burger','-file',str(p/'config/burger_contacts.sdf'),'-x','-2.0','-y','-0.5','-z','.01']),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':(robot/'urdf/turtlebot3_burger.urdf').read_text()}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox'))/'launch/online_async_launch.py')),launch_arguments={'use_sim_time':'true','slam_params_file':str(p/'config/slam.yaml')}.items()),
        Node(package='autopilot',executable='autopilot',parameters=[{'use_sim_time':True,'enabled':False,'scan_angle':20,'forward_speed':.1,'turn_speed':.3,'obstacle_distance':.5}]),
    ])
