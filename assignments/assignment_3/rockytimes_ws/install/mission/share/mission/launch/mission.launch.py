#!/usr/bin/env python3
"""
Combined Launch File

Sequence:
  1. Launch cylinder_landing.launch.py with verbose output.
  2. Wait for user input ("Press Enter to execute spiral trajectory...").
  3. Launch the spiral trajectory node.
  4. Launch rqt.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def wait_for_enter(context, *args, **kwargs):
    input("Press Enter to execute spiral trajectory and launch rqt...")
    return []

def generate_launch_description():
    ld = LaunchDescription()
    
    # 1. Include the cylinder landing launch file with verbose flag.
    cylinder_landing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.environ['HOME'], 'workspaces/rockytimes_ws/src/mission/launch/cylinder_landing.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )
    ld.add_action(cylinder_landing_launch)
    
    # 2. Wait for user input.
    ld.add_action(OpaqueFunction(function=wait_for_enter))
    
    # 3. Launch the spiral trajectory node.
    spiral_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mission', 'spiral_trajectory.py'],
        output='screen'
    )
    ld.add_action(spiral_node)
    
    # 4. Launch rqt.
    rqt_proc = ExecuteProcess(
        cmd=['rqt'],
        output='screen'
    )
    ld.add_action(rqt_proc)
    
    return ld
