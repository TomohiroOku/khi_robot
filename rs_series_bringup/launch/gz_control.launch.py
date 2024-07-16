import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# * On exit, some errors happens inevitably
# * 1. segmentation fault of Gazebo sim originate from gz_ros2_control
# * 2. error of controller_manager by Ctrl+C
# *
# * Two warnings:
# * [gz_ros_control]: On init...
# * [controller_manager]: ResourceManager has already loaded an urdf file. Ignoring attempt to reload a robot description file.


def generate_launch_description():

    pkg_description = get_package_share_directory("rs_series_bringup")

    robot_description = xacro.process_file(
        os.path.join(pkg_description, "urdf", "rs007l.gz.urdf.xacro")
    ).toxml()
    rviz_config = os.path.join(
        get_package_share_directory("rs_series_description"), "config", "rs_series.rviz"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": True}],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[{"topic": "/robot_description"}],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        output="screen",
    )

    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        output="screen",
    )

    delay_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[load_joint_state_broadcaster])
    )

    delay_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller, rviz],
        )
    )

    delay_rqt_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_trajectory_controller,
            on_exit=[rqt_controller_manager, rqt_joint_trajectory_controller],
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo,
            spawn,
            delay_broadcaster_after_spawn,
            delay_controller_after_broadcaster,
            delay_rqt_after_controller,
            DeclareLaunchArgument("use_sim_time", default_value="true"),
        ]
    )
