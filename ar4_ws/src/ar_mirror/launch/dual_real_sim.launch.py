"""เปิดแขนจำลอง (Gazebo) + แขนจริง (namespace 'real') พร้อมกัน แล้วมิเรอร์ให้ขยับตามกัน

  ros2 launch ar_mirror dual_real_sim.launch.py
  ros2 launch ar_mirror dual_real_sim.launch.py serial_port:=/dev/ttyACM0 calibrate:=True

สั่งงานผ่าน MoveIt (ar_moveit_config, เปิดแยกอีกหน้าต่างตามปกติ) เข้าแขนจำลองเหมือนเดิม
(ลาก marker ใน RViz แล้ว Plan & Execute) แขนจริงจะขยับตามภายใน ~1/rate วินาที ผ่าน joint_mirror

หมายเหตุ: ฝั่งแขนจริงเขียนขึ้นใหม่ที่นี่ (ไม่ได้ include ar_hardware_interface/ar_hardware.launch.py
ของผู้ขายตรงๆ) เพราะไฟล์นั้นใส่ "-c /controller_manager" แบบ absolute path ซึ่งจะหา
controller_manager ใน namespace "real" ไม่เจอ — แต่ยังใช้ไฟล์ข้อมูลของผู้ขาย (urdf xacro,
controllers.yaml, controller_update_rate.yaml) ทั้งหมดตรงๆ ไม่มีการแก้ไขไฟล์ผู้ขายเลย

⚠️ MoveIt วางแผนเทียบ collision กับฉากในซิมเท่านั้น ไม่รู้จักสิ่งกีดขวางรอบแขนจริง
   เคลียร์พื้นที่รอบแขนจริงให้ปลอดภัยก่อนรันเสมอ — แขนจะขยับทันทีที่มีคนสั่งฝั่งซิม
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")

    # ---------- แขนจำลอง (Gazebo, namespace default) ----------
    # gui default = false: หน้าต่าง 3D ของ Gazebo เรนเดอร์ผ่าน WSLg กิน CPU หนักมากจน
    # usbip (USB ของ Teensy ที่ forward จาก Windows) timeout แล้ว serial หลุดภายในไม่กี่วินาที
    # ดูแขนจำลองผ่าน RViz ของ MoveIt แทนได้ (เห็นทั้งซิมและแผนการเคลื่อนที่อยู่แล้ว)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ar_gz_bringup"), "launch", "ar_gz.launch.py"
            )
        ),
        launch_arguments={"gui": LaunchConfiguration("gz_gui")}.items(),
    )

    # ---------- แขนจริง (namespace "real") ----------
    # ไฟล์ข้อมูลทั้งหมดของผู้ขาย ไม่แก้ไข — reuse ตรงๆ
    real_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ar_hardware_interface"), "urdf", "ar.urdf.xacro"]
            ),
            " ",
            "name:=ar",
            " ",
            "serial_port:=", serial_port,
            " ",
            "calibrate:=", calibrate,
        ]
    )
    real_robot_description = {"robot_description": real_robot_description_content}

    # ใช้สำเนาของเราเอง (ar_mirror/config/real_controllers.yaml) แทนของผู้ขาย เพราะคีย์ใน
    # ไฟล์ผู้ขายเป็น "controller_manager:" ซึ่งใช้ได้เฉพาะ root namespace — ใต้ /real ต้องเป็น
    # "/real/controller_manager:" ไม่งั้น controller ไม่ได้ type แล้ว spawner ตาย
    real_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare("ar_mirror"), "config", "real_controllers.yaml"]
    )

    real_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="real",
        parameters=[
            real_robot_description,
            ParameterFile(real_controllers_cfg, allow_substs=True),
        ],
        output="screen",
    )

    real_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="real",
        output="both",
        parameters=[real_robot_description],
        # tf2 publish ไป "/tf" แบบ absolute เสมอ (namespace ไม่ช่วย) — ถ้าไม่ remap ฝั่งแขนจริง
        # จะยิงเฟรมชื่อ link_1..link_6 ชนกับของซิมที่ชื่อเดียวกัน แถมคนละฐานเวลา (แขนจริงใช้
        # wall time, ซิมใช้ sim time) RViz เลยขึ้น TF_OLD_DATA รัวๆ แล้วเลือกใช้อันที่เวลาใหม่กว่า
        # (ของแขนจริง) แทนซิม — แยกไป /real/tf ให้ไม่ชนกัน
        remappings=[("/tf", "/real/tf"), ("/tf_static", "/real/tf_static")],
    )

    real_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace="real",
        arguments=[
            "joint_state_broadcaster",
            "-c", "controller_manager",  # relative -> resolves under /real
            "--controller-manager-timeout", "60",
        ],
    )

    real_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="real",
        arguments=[
            "joint_trajectory_controller",
            "-c", "controller_manager",  # relative -> resolves under /real
            "--controller-manager-timeout", "60",
        ],
    )

    # ---------- มิเรอร์: sim /joint_states -> real joint_trajectory_controller ----------
    mirror = Node(
        package="ar_mirror",
        executable="joint_mirror",
        output="screen",
    )

    # ต่อ serial กับแขนจริง "หลัง" Gazebo โหลดเสร็จแล้ว — ถ้าต่อพร้อมกันตอน Gazebo บูต
    # (ช่วงที่กิน CPU สูงสุด) usbip จะ timeout แล้วหลุดทันที ("Error in transmit" ->
    # "read: End of file" -> hardware 'ar' ถูก deactivate -> controller activate ไม่ได้)
    real_side_delayed = TimerAction(
        period=LaunchConfiguration("real_start_delay"),
        actions=[
            real_controller_manager,
            real_robot_state_publisher,
            real_joint_state_broadcaster,
            real_joint_trajectory_controller,
            mirror,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("calibrate", default_value="True"),
        DeclareLaunchArgument(
            "gz_gui",
            default_value="false",
            choices=["true", "false"],
            description="เปิดหน้าต่าง 3D ของ Gazebo ไหม (false = ประหยัด CPU กัน serial หลุด)",
        ),
        DeclareLaunchArgument(
            "real_start_delay",
            default_value="25.0",
            description="หน่วงกี่วินาทีก่อนต่อแขนจริง (รอ Gazebo โหลดเสร็จก่อน)",
        ),
        sim,
        real_side_delayed,
    ])
