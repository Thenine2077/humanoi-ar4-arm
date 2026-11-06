"""MoveIt + RViz for both AR4 arms (one window showing both arms + the pole) — start after dual_hardware.launch.py

  ros2 launch ar_dual dual_moveit.launch.py      # (arm moveit calls this, with the RViz config file)

In RViz: pick left_arm / right_arm / both_arms in Planning Group -> drag the end-effector ball, or use the Joints tab
-> Plan (checks arm-arm and arm-pole/head/camera collisions) -> Execute
Same layout as ar_gz_bringup/launch/ar_moveit_jazzy.launch.py (single arm) — list parameters come straight from
YAML files to stop launch_ros converting them to !!python/tuple (see the explanation in that file)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def share(*parts):
    return os.path.join(get_package_share_directory("ar_dual"), *parts)


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    # none (or empty) = open a blank RViz — on the PC (WSLg) -d must not be used, otherwise mouse clicks in the window don't work
    # (see ar_gz_bringup/launch/ar_moveit_jazzy.launch.py); load the config afterwards via File > Open Config
    has_rviz_config = PythonExpression(["'", rviz_config, "' not in ('', 'none')"])

    robot_description = {"robot_description": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", share("urdf", "humanoi_arms.urdf.xacro"),
        " left_hw:=mock right_hw:=mock"]), value_type=str)}   # MoveIt only needs the geometry, ignores the ros2_control part
    with open(share("srdf", "humanoi_arms.srdf"), encoding="utf-8") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}
    kinematics = share("config", "kinematics.yaml")
    with open(share("config", "joint_limits.yaml"), encoding="utf-8") as f:
        planning = {"robot_description_planning": yaml.safe_load(f)}
    ompl = share("config", "ompl_planning_pipeline.yaml")
    controllers = share("config", "moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }
    scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics, planning, ompl,
                    trajectory_execution, controllers, scene_monitor])

    rviz_parameters = [robot_description, robot_description_semantic, ompl, kinematics, planning]
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
                parameters=rviz_parameters, condition=UnlessCondition(has_rviz_config))
    rviz_with_config = Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
                            arguments=["-d", rviz_config], parameters=rviz_parameters,
                            condition=IfCondition(has_rviz_config))
    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=share("config", "dual_moveit.rviz"),
                              description=".rviz file (none = blank RViz, for WSLg where -d must not be used)"),
        rviz, rviz_with_config, move_group,
    ])
