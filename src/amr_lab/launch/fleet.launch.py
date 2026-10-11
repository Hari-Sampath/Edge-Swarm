"""Launch the Robot Operating System 2 (ROS 2) mobile robot fleet."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share=get_package_share_directory('amr_lab'); params=os.path.join(share,'config','params.yaml')
    args=[DeclareLaunchArgument('seed',default_value='1'),DeclareLaunchArgument('packet_loss',default_value='0.0'),DeclareLaunchArgument('headless',default_value='true'),DeclareLaunchArgument('task_count',default_value='8'),DeclareLaunchArgument('max_ticks',default_value='3000')]
    overrides={'seed':ParameterValue(LaunchConfiguration('seed'),value_type=int),'packet_loss':ParameterValue(LaunchConfiguration('packet_loss'),value_type=float),'task_count':ParameterValue(LaunchConfiguration('task_count'),value_type=int),'max_ticks':ParameterValue(LaunchConfiguration('max_ticks'),value_type=int),'headless':ParameterValue(LaunchConfiguration('headless'),value_type=bool)}
    sim=Node(package='amr_lab',executable='sim_node',name='simulator',output='screen',parameters=[params,overrides])
    return LaunchDescription(args+[sim])
