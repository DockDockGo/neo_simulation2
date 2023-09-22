# Neobotix GmbH

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions
import time
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
from launch.actions import TimerAction

import os
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import time
import numpy as np

MY_NO_ROBOTS = os.environ['Number_of_Robots']
MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    global MY_NO_ROBOTS 

    # TODO remove this condifion and maybr run gazebo in headkess mode

    # if(int(MY_NO_ROBOTS) > 5):
    #     print("Warn: Having more than 5 robots is not a good idea - too much overhead")
    #     print("Therefore Let's spawn 5")
    #     MY_NO_ROBOTS = '5'
    #     time.sleep(5)   

    ld = LaunchDescription()
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items()
        )
    ld.add_action(gazebo)

    #  using numpy compute the x and y position of the robots given number of robots as input
    max_rows = 10
    max_cols = 10

    delta_x = 1.0 #meter
    delta_y = 1.0 #meter

    x0 = -0.75
    y0 = 0.25

    num_cols = min(max_cols, int(MY_NO_ROBOTS))
    num_rows = int(MY_NO_ROBOTS) // num_cols    

    x_pos = np.arange(x0, x0 + delta_x * num_cols, delta_x)
    y_pos = np.arange(y0, y0 + delta_y * num_rows, delta_y)
    
    last_x_pos = None
    last_y_pos = None

    if int(MY_NO_ROBOTS) % num_cols != 0:
        last_x_pos = x_pos[-1] + delta_x
        last_y_pos = np.arange(y0, y0 + delta_y * (int(MY_NO_ROBOTS) % num_cols), delta_y)
# -------------------------DEFAULT--------------------------------------------

    # i = 0
    # for y in y_pos:
    #     for x in x_pos:
    #         ld.add_action(IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
    #             ),
    #             launch_arguments={
    #                 'use_multi_robots': 'True',
    #                 # 'x': str(x),
    #                 # 'y': str(y),
    #                 'x': str(y),
    #                 'y': str(x),
    #                 'namespace_robot': "robot" + str(i),
    #             }.items(),
    #         ))
    #         i += 1
    
    # if last_x_pos is not None:
    #     for y in last_y_pos:
    #         ld.add_action(IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
    #             ),
    #             launch_arguments={
    #                 'use_multi_robots': 'True',
    #                 # 'x': str(last_x_pos),
    #                 # 'y': str(y),
    #                 'x': str(y),
    #                 'y': str(last_x_pos),
    #                 'namespace_robot': "robot" + str(i),
    #             }.items(),
    #         ))
    #         i += 1
# ------------------------------TIMER----------------------------------------
    spawn_actions = []

    i = 0
    for y in y_pos:
        for x in x_pos:
            spawn_actions.append(
                IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
                ),
                launch_arguments={
                    'use_multi_robots': 'True',
                    # 'x': str(x),
                    # 'y': str(y),
                    'x': str(y),
                    'y': str(x),
                    'namespace_robot': "robot" + str(i),
                }.items(),
            ))
            i += 1


    delay_time = 5.0
    delayed_spawn_action = [TimerAction(period=delay_time, actions=[spawn_actions[-1]])]

    for k in range(len(spawn_actions)-2,-1,-1):
        delayed_spawn_action.append(
            TimerAction(period=delay_time, actions=[spawn_actions[k],delayed_spawn_action[-1]])
        )
        
    ld.add_action(delayed_spawn_action[-1])

# --------------------------------------------------------------------------------

    # for action in spawn_actions:
    #     ld.add_action(action)

    # for l in range(1, len(spawn_actions)):
    #     ld.add_action(
    #         RegisterEventHandler(
    #             event_handler=OnExecutionComplete(
    #                 target_action=spawn_actions[l - 1],
    #                 on_completion=[spawn_actions[l]],
    #             )
    #         )
    #     )
    #     # ld.add_action(spawn_actions[l])


    # for i in range(0, int(MY_NO_ROBOTS)):
    #     ld.add_action(IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
    #         ),
    #         launch_arguments={
    #             'use_multi_robots': 'True',
    #             'x':
    #             'y': str((2.0 - int(i))),
    #             'namespace_robot': "robot" + str(i),
    #         }.items(),
    #     ))

    return ld