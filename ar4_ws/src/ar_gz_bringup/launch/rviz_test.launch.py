"""ไฟล์ทดสอบ: เปิด move_group (จริง มีข้อมูล robot ครบ) พร้อมกับ rviz2 เปล่าๆ (ไม่มี -d, ไม่มี
MotionPlanning display) ผ่าน ros2 launch ในไฟล์เดียวกัน เพื่อแยกว่าปัญหาเมาส์กดไม่ได้เกิดจาก
"เปิดหลายโหนดพร้อมกันผ่าน ros2 launch" หรือเกิดจาก "MotionPlanning/interactive marker
ที่โหลดมาจาก config ไฟล์ของผู้ขายตั้งแต่ต้น"

  ros2 launch ar_gz_bringup rviz_test.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

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

    ompl_planning_pipeline_config = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "ompl_planning_pipeline.yaml"]
    )
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
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

    # เพิ่ม -d โหลด config ของผู้ขายเหมือน ar_moveit_jazzy.launch.py ทุกอย่าง
    # (ตัวแปรสุดท้ายที่ยังไม่ได้แยกทดสอบ)
    rviz_config_file = PathJoinSubstitution([FindPackageShare("ar_gz_bringup"), "config", "ar_moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(declared_arguments + [rviz_node, move_group_node])
