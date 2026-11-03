#!/usr/bin/env python3
#
# Task 10: the simulation of Task 9, together with RViz2.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Needed to run ROS2 nodes from a package
from launch_ros.actions import Node


def generate_launch_description():

    # The simulation of Task 9
    task9_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('week_4_solutions'), 'launch',
                         'task9.launch.py')
        )
    )

    # RViz2, with the configuration for the TurtleBot3 provided by turtlebot3_gazebo,
    # using the simulation's clock
    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'rviz', 'tb3_gazebo.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='log'
    )

    ld = LaunchDescription()

    ld.add_action(task9_cmd)

    # For Task 10, we add RViz2
    ld.add_action(rviz_node)

    return ld
