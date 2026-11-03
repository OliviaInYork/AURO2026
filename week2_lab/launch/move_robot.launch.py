import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Panda MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(
            file_path="config/panda.urdf.xacro"
        )
        .trajectory_execution(
            file_path="config/gripper_moveit_controllers.yaml"
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("moveit2_tutorials"),
                "config",
                "motion_planning_python_api_tutorial.yaml",
            )
        )
        .to_moveit_configs()
    )

    # Our MoveItPy program
    move_robot_node = Node(
        package="week2_lab",
        executable="move_robot",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory(
            "moveit_resources_panda_moveit_config"
        ),
        "config/ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            (
                "/controller_manager/robot_description",
                "/robot_description",
            )
        ],
        output="log",
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager", "--param-file", ros2_controllers_path],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager", "--param-file", ros2_controllers_path],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            ros2_controllers_path,
        ],
    )

    # RViz - viewer only
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit2_tutorials"),
        "config",
        "motion_planning_python_api_tutorial.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    delay_move_robot_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=panda_arm_controller_spawner,
            on_exit=[move_robot_node],
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_hand_controller_spawner,
            panda_arm_controller_spawner,
            static_tf_node,
            rviz_node,
            delay_move_robot_after_spawner
        ]
    )