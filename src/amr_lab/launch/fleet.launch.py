"""Launch simulator, task server, metrics, and one process per robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def make_nodes(context):
    share=get_package_share_directory('amr_lab'); params=os.path.join(share,'config','params.yaml')
    count=int(LaunchConfiguration('robot_count').perform(context))
    common=[params,{'seed':ParameterValue(LaunchConfiguration('seed'),value_type=int),'packet_loss':ParameterValue(LaunchConfiguration('packet_loss'),value_type=float),'task_count':ParameterValue(LaunchConfiguration('task_count'),value_type=int),'task_arrival_rate':ParameterValue(LaunchConfiguration('task_arrival_rate'),value_type=float),'max_ticks':ParameterValue(LaunchConfiguration('max_ticks'),value_type=int),'headless':ParameterValue(LaunchConfiguration('headless'),value_type=bool),'robot_count':count}]
    nodes=[Node(package='amr_lab',executable='sim_node',name='simulator',output='screen',parameters=common),Node(package='amr_lab',executable='task_server_node',name='task_server',output='screen',parameters=common),Node(package='amr_lab',executable='metrics_node',name='metrics',output='screen',parameters=common)]
    nodes.extend(Node(package='amr_lab',executable='robot_node',name=f'robot_{i}',output='screen',parameters=common+[{'robot_id':f'robot_{i}'}]) for i in range(count))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=nodes[2],on_exit=[EmitEvent(event=Shutdown(reason='metrics complete'))])))
    return nodes

def generate_launch_description():
    args=[DeclareLaunchArgument('seed',default_value='1'),DeclareLaunchArgument('packet_loss',default_value='0.0'),DeclareLaunchArgument('headless',default_value='true'),DeclareLaunchArgument('task_count',default_value='8'),DeclareLaunchArgument('task_arrival_rate',default_value='0.0'),DeclareLaunchArgument('max_ticks',default_value='3000'),DeclareLaunchArgument('robot_count',default_value='4')]
    return LaunchDescription(args+[OpaqueFunction(function=make_nodes)])
