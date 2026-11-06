"""Same as the vendor's ar_moveit_config/launch/ar_moveit.launch.py in every way (SRDF/kinematics/
joint_limits are still loaded straight from the vendor files; no vendor file is edited) — 2 differences, both are
Iron→Jazzy compatibility issues, not bugs in the vendor files:

  1. request_adapters of the OMPL planning pipeline must be a list of strings on MoveIt2/Jazzy
     (2.12.4), not one long space-separated string as the vendor wrote it (written for Iron)
  2. parameters that contain lists (request_adapters, controller_names, joints) must come from YAML files
     written directly (config/ompl_planning_pipeline.yaml, config/moveit_controllers.yaml),
     not a Python dict built in code and handed to launch_ros — because launch_ros turns lists
     into python tuples before dumping them to a temp file, giving a "!!python/tuple" tag that the ROS2 (C)
     YAML parser can't read, and then parsing of "the whole file" silently fails — even plain string parameters in
     the same file (e.g. planning_plugin) are lost too, although their values are correct

  ros2 launch ar_gz_bringup ar_moveit_jazzy.launch.py use_sim_time:=true
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt use simulation time.",
        ),
        # On the PC (WSLg) leave it empty — see the reason at rviz_node below
        # On the Pi (real screen/Raspberry Pi Connect) that problem doesn't exist; give a config path and it opens with MotionPlanning
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description=".rviz file to load at startup ('' = open an empty RViz, for WSLg)",
        ),
    ]
    rviz_config = LaunchConfiguration("rviz_config")
    has_rviz_config = PythonExpression(["'", rviz_config, "' != ''"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ar_description"), "urdf", "ar.urdf.xacro"]),
            " ",
            "name:=ar",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ar_moveit_config"), "srdf", "ar.srdf.xacro"]),
            " ",
            "name:=ar",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ar_moveit_config"), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml("ar_moveit_config", os.path.join("config", "joint_limits.yaml"))
    }

    # planning_plugin/request_adapters come from a YAML file written directly (config/ompl_planning_pipeline.yaml),
    # not a Python dict — so launch_ros can't turn lists into !!python/tuple and break parsing of the whole file
    ompl_planning_pipeline_config = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "ompl_planning_pipeline.yaml"]
    )

    # same reason as ompl_planning_pipeline_config — controller_names/joints are lists
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        # 0.01 (the original value) is too tight for this sim — just the small delay between Plan finishing
        # and pressing Execute makes a joint drift more than 0.01 rad, so Execute is rejected midway,
        # and then the controller (which doesn't enforce command limits because the vendor's ros2_control file
        # fails to parse the limits) leaves the arm stuck at the joint's hard stop for good
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Open RViz "empty", deliberately without -d loading a config at startup, because -d makes the RViz window
    # appear late (it parses the config first), colliding with the moment WSLg registers the window for input,
    # so mouse clicks in the window stop working for good (not even minimize) for the life of that window —
    # nothing to do with the config contents (tested: even an empty config hangs the same way with -d)
    # Fix confirmed to work 100%: open an empty RViz so the window appears/responds first,
    # then load the config afterwards via the menu File > Open Config, choosing the file
    # ar_gz_bringup/config/ar_moveit.rviz (gives the full MotionPlanning panel like the vendor's)
    rviz_parameters = [
        robot_description,
        robot_description_semantic,
        ompl_planning_pipeline_config,
        robot_description_kinematics,
        robot_description_planning,
        {"use_sim_time": use_sim_time},
    ]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        parameters=rviz_parameters,
        condition=UnlessCondition(has_rviz_config),
    )
    rviz_node_with_config = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
        condition=IfCondition(has_rviz_config),
    )

    # Note: robot_state_publisher is not run here — same as the vendor's original file, because
    # ar_gz_bringup/ar_gz.launch.py (or ar_hardware_interface on the real-arm side) already runs its own;
    # running it twice would cause a node name clash
    return LaunchDescription(declared_arguments + [rviz_node, rviz_node_with_config, move_group_node])
