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
Launch file for spawning TurtleBot4 with UWB MultiAnchor sensor.

This launch file extends the standard TurtleBot4 spawn with UWB MultiAnchor
positioning system support.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_robot(context, *args, **kwargs):
    """Spawn robot with MultiAnchor sensor."""
    
    pkg_uwb_multianchor = FindPackageShare('uwb_multianchor')
    
    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    model = LaunchConfiguration('model')
    
    # Determine which URDF to use based on model
    urdf_file = PythonExpression([
        "'turtlebot4_standard_multianchor.urdf.xacro' if '",
        model,
        "' == 'standard' else 'turtlebot4_lite_multianchor.urdf.xacro'"
    ])
    
    # Robot description
    robot_description_command = [
        'xacro', ' ',
        PathJoinSubstitution([
            pkg_uwb_multianchor,
            'urdf',
            urdf_file
        ]),
        ' gazebo:=gz'
    ]
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_command,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Spawn entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-namespace', namespace,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '-topic', 'robot_description',
        ],
        output='screen',
    )
    
    # MultiAnchor sensor static transform
    multianchor_tf = Node(
        name='multianchor_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0.0',
            'multianchorsensor_link',
            [robot_name, '/multianchorsensor_link/multianchorsensor']
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    # ROS-Gazebo bridge for MultiAnchor scan topic
    multianchor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='multianchor_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            ['/model/', robot_name, '/multianchorsensor_link/multianchorsensor/scan',
             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        ],
        remappings=[
            (['/model/', robot_name, '/multianchorsensor_link/multianchorsensor/scan'], 
             '/scan')
        ]
    )
    
    return [
        robot_state_publisher,
        spawn_robot,
        multianchor_tf,
        multianchor_bridge,
    ]


def generate_launch_description():
    """Generate launch description for TurtleBot4 with MultiAnchor."""
    
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
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
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot4 model (standard or lite)'
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(namespace_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(yaw_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(model_arg)
    
    # Add spawn function
    ld.add_action(OpaqueFunction(function=spawn_robot))
    
    return ld

