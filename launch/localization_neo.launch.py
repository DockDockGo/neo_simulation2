import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from pathlib import Path
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    parameters =  LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time')    

    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'yaml_filename': map_file}

    configured_params = RewrittenYaml(
        source_file=parameters,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    return LaunchDescription([
        Node(
            package='neo_localization2', 
            executable='neo_localization_node', 
            output='screen',
            name='neo_localization2_node', 
            parameters= [configured_params],
            ),
    ])