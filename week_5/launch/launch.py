# Based on turtlebot3_world.launch.py, from turtlebot3_gazebo, which uses Gazebo (gz sim),
# through ros_gz_sim.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'week_5'
    turtlebot3_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
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
        default_value='1.0',
        description='x coordinate where the robot is spawned'
    )

    y_pose_argument = DeclareLaunchArgument(
        'y_pose',
        default_value='1.0',
        description='y coordinate where the robot is spawned'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'week_5_world.world'
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
            os.path.join(turtlebot3_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawns the robot, and bridges its topics between Gazebo and ROS
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Lets Gazebo find the models of the world and of the TurtleBot3
    set_env_vars_world_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory(package_name),
            'models'))

    set_env_vars_turtlebot3_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    ld = LaunchDescription()

    # Add launch options
    ld.add_action(use_sim_time_argument)
    ld.add_action(x_pose_argument)
    ld.add_action(y_pose_argument)

    # Set the resource paths before Gazebo starts, so it can resolve model:// URIs
    ld.add_action(set_env_vars_world_resources)
    ld.add_action(set_env_vars_turtlebot3_resources)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
