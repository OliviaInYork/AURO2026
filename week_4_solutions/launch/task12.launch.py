#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool, with modifications by Pedro Ribeiro
#
# Based on turtlebot3_world.launch.py, from turtlebot3_gazebo, which uses Gazebo (gz sim),
# through ros_gz_sim.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Needed to run ROS2 nodes from a package
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments: declared, with their defaults and descriptions, so that --show-args
    # describes them, and read by the configurations below.

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the simulation (Gazebo) clock if true'
    )

    x_pose_argument = DeclareLaunchArgument(
        'x_pose',
        default_value='-2.0',
        description='x coordinate where the robot is spawned'
    )

    y_pose_argument = DeclareLaunchArgument(
        'y_pose',
        default_value='-0.5',
        description='y coordinate where the robot is spawned'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

    # Gazebo server, running the world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Gazebo client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawns the robot, and bridges its topics between Gazebo and ROS
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Lets Gazebo find the models of the TurtleBot3 and of the world
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    # For Task 12, we declare the parameters of the launch file, which are then passed to
    # the node below.

    T_constant = LaunchConfiguration('T')
    F_constant = LaunchConfiguration('F')

    T_constant_argument = DeclareLaunchArgument(
        'T',
        default_value='5',
        description="Value of constant 'T'"
    )

    F_constant_argument = DeclareLaunchArgument(
        'F',
        default_value='1',
        description="Value of constant 'F'"
    )

    # The node of task5.py, whose constants T and F are given by ROS parameters
    turtlebot3_fsmc_node = Node(
        name='TurtleBot3FSMC',
        package='week_4_solutions',
        executable='task5',
        output='both',
        parameters=[{
            'T': T_constant,
            'F': F_constant
        }]
    )

    ld = LaunchDescription()

    # Add launch options
    ld.add_action(use_sim_time_argument)
    ld.add_action(x_pose_argument)
    ld.add_action(y_pose_argument)
    ld.add_action(T_constant_argument)
    ld.add_action(F_constant_argument)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)

    # Nodes
    ld.add_action(turtlebot3_fsmc_node)

    return ld
