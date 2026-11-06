"""Spawn the AR4 arm (vendor's ar_description, unmodified) into Gazebo Harmonic
(gz-sim) and hook it up to ros2_control via gz_ros2_control.

  ros2 launch ar_gz_bringup ar_gz.launch.py

Then, in another terminal, drive it with MoveIt (vendor's ar_moveit_config,
unmodified):

  ros2 launch ar_moveit_config ar_moveit.launch.py use_sim_time:=true
"""
import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def sim_robot_description() -> str:
    """URDF for sim: same as the vendor's in every way, except the effort/velocity limits that are 0 are fixed

    The vendor's ar_description/urdf/ar_macro.xacro (read only, that file is not edited) sets
    effort="0" on almost every joint (and joint_2 has velocity="0"). These values have no effect on the real arm,
    because the real side is driven over serial and never reads them — but in sim, the physics uses them directly.
    effort=0 means the joint motor can apply 0 Nm, so the joint is free and gravity pulls the arm
    down until it hits the hard stop within a few tens of seconds. Only joints that carry gravity load show it (joint_3,
    joint_5); joint_1/4/6 rotate about a vertical axis so they can stay at 0 — matching how MoveIt reported
    CheckStartStateBounds out of bounds only for joint_3/joint_5 every time
    """
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("ar_gz_bringup"), "urdf", "ar_gz.urdf.xacro"
        ),
        mappings={
            "name": "ar",
            # use the vendor's controllers.yaml as is, unmodified
            "simulation_controllers": os.path.join(
                get_package_share_directory("ar_hardware_interface"),
                "config",
                "controllers.yaml",
            ),
        },
    ).toxml()
    urdf = re.sub(r'(<limit[^>]*?)effort="[^"]*"', r'\1effort="100"', urdf)
    urdf = re.sub(r'(<limit[^>]*?)velocity="[^"]*"', r'\1velocity="3.0"', urdf)
    return urdf


def generate_launch_description():
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    robot_description = {"robot_description": sim_robot_description()}

    # gui:=false -> run gz as server only (-s), no 3D window
    # Use this when running together with the real arm: Ogre2 rendering through WSLg loads the CPU so much that usbip (the
    # Teensy's USB forwarded from Windows into WSL) times out and serial drops — watch the sim in RViz instead
    gz_args = PythonExpression(
        [
            "'-r -v3 empty.sdf' if '",
            LaunchConfiguration("gui"),
            "' == 'true' else '-r -s -v3 empty.sdf'",
        ]
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "ar", "-allow_renaming", "true"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager-timeout", "60"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            choices=["true", "false"],
            description="Open the Gazebo 3D window or not (false = server only, saves CPU)",
        ),
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
    ])
