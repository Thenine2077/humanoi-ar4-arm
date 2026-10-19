"""MoveIt + RViz สำหรับแขน AR4 สองตัว (หน้าต่างเดียว เห็นทั้งสองแขน + เสา) — เปิดหลัง dual_hardware.launch.py

  ros2 launch ar_dual dual_moveit.launch.py      # (arm moveit เรียกตัวนี้ พร้อมไฟล์ RViz config)

ใน RViz: ช่อง Planning Group เลือก left_arm / right_arm / both_arms -> ลากลูกบอลปลายแขน หรือแท็บ Joints
-> Plan (เช็กการชนแขนกับแขน และแขนกับเสา/หัว/กล้องให้) -> Execute
โครงเหมือน ar_gz_bringup/launch/ar_moveit_jazzy.launch.py (แขนเดียว) — พารามิเตอร์ที่เป็น list มาจากไฟล์
YAML ตรงๆ เพื่อกัน launch_ros แปลงเป็น !!python/tuple (ดูคำอธิบายในไฟล์นั้น)
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
    # none (หรือว่าง) = เปิด RViz เปล่า — บน PC (WSLg) ห้ามใช้ -d ไม่งั้นเมาส์คลิกในหน้าต่างไม่ได้
    # (ดู ar_gz_bringup/launch/ar_moveit_jazzy.launch.py) แล้วค่อยโหลด config ผ่าน File > Open Config
    has_rviz_config = PythonExpression(["'", rviz_config, "' not in ('', 'none')"])

    robot_description = {"robot_description": ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", share("urdf", "humanoi_arms.urdf.xacro"),
        " left_hw:=mock right_hw:=mock"]), value_type=str)}   # MoveIt ใช้แค่รูปทรง ไม่สนส่วน ros2_control
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
                              description="ไฟล์ .rviz (none = RViz เปล่า สำหรับ WSLg ที่ห้ามใช้ -d)"),
        rviz, rviz_with_config, move_group,
    ])
