#!/usr/bin/env python3
# Copyright 2025 Noh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Complete simulation launch file for TurtleBot4 with UWB MultiAnchor.

This launch file starts Gazebo, spawns the robot with MultiAnchor sensor,
and sets up all necessary bridges and transforms.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate complete simulation launch description."""
    
    pkg_turtlebot4_gz_bringup = FindPackageShare('turtlebot4_gz_bringup')
    pkg_uwb_multianchor = FindPackageShare('uwb_multianchor')
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='depot',
        description='World file name'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot4 model'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Spawn X position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Spawn Y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.1',
        description='Spawn Z position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Spawn yaw angle'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot4',
        description='Robot name'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # Gazebo simulation (without robot spawn)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_gz.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )
    
    # Spawn TurtleBot4 with MultiAnchor sensor
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_uwb_multianchor,
                'launch',
                'turtlebot4_multianchor_spawn.launch.py'
            ])
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'robot_name': LaunchConfiguration('robot_name'),
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )
    
    # TurtleBot4 nodes
    turtlebot4_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot4_gz_bringup,
                'launch',
                'turtlebot4_nodes.launch.py'
            ])
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(yaw_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(namespace_arg)
    
    # Add launch includes
    ld.add_action(gazebo_sim)
    ld.add_action(spawn_robot)
    ld.add_action(turtlebot4_nodes)
    
    return ld

