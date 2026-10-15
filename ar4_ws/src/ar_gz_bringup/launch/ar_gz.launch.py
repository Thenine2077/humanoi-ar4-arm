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
    """URDF สำหรับ sim: เหมือนของผู้ขายทุกอย่าง ยกเว้นแก้ effort/velocity limit ที่เป็น 0

    ar_description/urdf/ar_macro.xacro ของผู้ขาย (อ่านอย่างเดียว ไม่แก้ไฟล์นั้น) ตั้ง
    effort="0" ไว้แทบทุกข้อ (และ joint_2 ตั้ง velocity="0") ค่าพวกนี้ไม่มีผลกับแขนจริง
    เพราะฝั่งจริงสั่งงานผ่าน serial ไม่ได้อ่านค่านี้ — แต่ในซิม ฟิสิกส์เอาไปใช้ตรงๆ
    effort=0 แปลว่ามอเตอร์ข้อต่อออกแรงได้ 0 Nm ข้อต่อจึงเป็นอิสระ แรงโน้มถ่วงดึงแขน
    ตกลงจนชน hard-stop ภายในไม่กี่สิบวินาที เห็นผลเฉพาะข้อที่รับแรงโน้มถ่วง (joint_3,
    joint_5) ส่วน joint_1/4/6 หมุนรอบแกนตั้งจึงอยู่ที่ 0 ได้ — ตรงกับที่ MoveIt รายงาน
    CheckStartStateBounds หลุด bounds เฉพาะ joint_3/joint_5 ทุกครั้ง
    """
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("ar_gz_bringup"), "urdf", "ar_gz.urdf.xacro"
        ),
        mappings={
            "name": "ar",
            # ใช้ controllers.yaml ของผู้ขายตรงๆ ไม่แก้ไข
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

    # gui:=false -> รัน gz แบบ server อย่างเดียว (-s) ไม่เปิดหน้าต่าง 3D
    # ใช้ตอนรันคู่กับแขนจริง: การเรนเดอร์ Ogre2 ผ่าน WSLg กิน CPU หนักจน usbip (USB ของ
    # Teensy ที่ส่งจาก Windows เข้า WSL) timeout แล้ว serial หลุด — ดูซิมผ่าน RViz แทนได้
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
            description="เปิดหน้าต่าง 3D ของ Gazebo หรือไม่ (false = server อย่างเดียว ประหยัด CPU)",
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
