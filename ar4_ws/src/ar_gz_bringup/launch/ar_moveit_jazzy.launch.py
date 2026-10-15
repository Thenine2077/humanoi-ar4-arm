"""เหมือน ar_moveit_config/launch/ar_moveit.launch.py ของผู้ขายทุกอย่าง (SRDF/kinematics/
joint_limits ยังโหลดจากไฟล์ของผู้ขายตรงๆ ไม่แก้ไฟล์ผู้ขายเลย) — ต่างกัน 2 จุด ทั้งคู่เป็น
ปัญหาความเข้ากันได้ Iron→Jazzy ไม่ใช่บั๊กของไฟล์ผู้ขาย:

  1. request_adapters ของ OMPL planning pipeline ต้องเป็น list ของสตริงบน MoveIt2/Jazzy
     (2.12.4) ไม่ใช่สตริงยาวคั่นด้วยช่องว่างแบบที่ผู้ขายเขียนไว้ (เขียนสำหรับ Iron)
  2. พารามิเตอร์ที่มี list (request_adapters, controller_names, joints) ต้องมาจากไฟล์ YAML
     ที่เขียนตรงๆ (config/ompl_planning_pipeline.yaml, config/moveit_controllers.yaml)
     ไม่ใช่ Python dict ที่ประกอบในโค้ดแล้วส่งให้ launch_ros — เพราะ launch_ros แปลง list
     เป็น python tuple ก่อน dump เป็นไฟล์ชั่วคราว ได้ tag "!!python/tuple" ที่ parser YAML
     ของ ROS2 (C) อ่านไม่ออก แล้ว parse "ทั้งไฟล์" พังเงียบๆ — แม้พารามิเตอร์สตริงธรรมดาที่อยู่
     ไฟล์เดียวกัน (เช่น planning_plugin) ก็หายไปด้วย ทั้งที่ค่าถูกต้อง

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
        # บน PC (WSLg) ปล่อยว่างไว้ — ดูเหตุผลที่ rviz_node ข้างล่าง
        # บน Pi (จอจริง/Raspberry Pi Connect) ไม่มีปัญหานั้น ใส่ path config ได้เลยจะเปิดมาพร้อม MotionPlanning
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="ไฟล์ .rviz ที่จะโหลดตอนเปิด ('' = เปิด RViz เปล่า สำหรับ WSLg)",
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

    # planning_plugin/request_adapters มาจากไฟล์ YAML ที่เขียนตรงๆ (config/ompl_planning_pipeline.yaml)
    # ไม่ใช่ Python dict — กัน launch_ros แปลง list เป็น !!python/tuple แล้ว parse ทั้งไฟล์พัง
    ompl_planning_pipeline_config = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "ompl_planning_pipeline.yaml"]
    )

    # เหตุผลเดียวกับ ompl_planning_pipeline_config — controller_names/joints เป็น list
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        # 0.01 (ค่าเดิม) แคบเกินไปสำหรับ sim นี้ — แค่ความหน่วงเล็กน้อยระหว่าง Plan เสร็จ
        # กับกด Execute ก็ทำให้ joint เบี่ยงเกิน 0.01 rad จน Execute ถูก reject กลางทาง
        # แล้ว controller (ที่ไม่ enforce command limit เพราะไฟล์ ros2_control ของผู้ขาย
        # parse limit ไม่ผ่าน) ทิ้งแขนค้างไว้ที่ hard-stop ของข้อต่อถาวร
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

    # เปิด RViz "เปล่า" ตั้งใจไม่ใส่ -d โหลด config ตอนเปิด เพราะ -d ทำให้หน้าต่าง RViz
    # โชว์ช้า (ต้อง parse config ก่อน) ไปชนจังหวะที่ WSLg ลงทะเบียนรับ input ของหน้าต่าง
    # พอดี ทำให้เมาส์คลิกอะไรในหน้าต่างไม่ได้เลยถาวร (แม้แต่ minimize) ตลอดอายุหน้าต่างนั้น —
    # ไม่เกี่ยวกับเนื้อหา config เลย (ทดสอบแล้วว่า config เปล่าๆก็ค้างเหมือนกันถ้าใช้ -d)
    # วิธีแก้ที่ยืนยันแล้วว่าได้ผล 100%: เปิด RViz เปล่าให้หน้าต่างโชว์/ตอบสนองก่อน
    # แล้วค่อยโหลด config เอาทีหลังผ่านเมนู File > Open Config เลือกไฟล์
    # ar_gz_bringup/config/ar_moveit.rviz (จะได้ MotionPlanning panel ครบเหมือนของผู้ขาย)
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

    # หมายเหตุ: ไม่รัน robot_state_publisher ที่นี่ — ตรงกับไฟล์ต้นฉบับของผู้ขาย เพราะ
    # ar_gz_bringup/ar_gz.launch.py (หรือ ar_hardware_interface ฝั่งแขนจริง) รันของตัวเองอยู่แล้ว
    # รันซ้ำจะชื่อโหนดชนกัน
    return LaunchDescription(declared_arguments + [rviz_node, rviz_node_with_config, move_group_node])
