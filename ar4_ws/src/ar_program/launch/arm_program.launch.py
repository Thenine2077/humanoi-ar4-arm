"""เปิดตัวเล่นโปรแกรมท่า (arm_program)

  # แขนจริงสองตัวบน Pi (ar_dual dual_hardware.launch.py) — ค่าเริ่มต้น
  ros2 launch ar_program arm_program.launch.py
  # ซิมแขนเดี่ยว (ar_gz_bringup ar_gz.launch.py) หรือแขนจริงแบบเดี่ยว (ar_hardware.launch.py)
  ros2 launch ar_program arm_program.launch.py layout:=single
  # แขนจริงในชุด ar_mirror dual_real_sim.launch.py (แขนจริงอยู่ใต้ namespace /real)
  ros2 launch ar_program arm_program.launch.py layout:=single arm_ns:=real

ในชุด dual_real_sim ให้ป้อนโปรแกรมเข้า "ซิม" (arm_ns ว่าง) ก็พอ — joint_mirror จะพาแขนจริงตามเอง
ใช้ arm_ns:=real เฉพาะตอนอยากสั่งแขนจริงตรงๆ โดยไม่ผ่านซิม
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    ns = LaunchConfiguration("arm_ns").perform(context).strip("/")
    prefix = f"/{ns}" if ns else ""
    return [Node(
        package="ar_program",
        executable="arm_program",
        output="screen",
        parameters=[{
            "layout": LaunchConfiguration("layout").perform(context),
            "action_name": f"{prefix}/joint_trajectory_controller/follow_joint_trajectory",
            "joint_states_topic": f"{prefix}/joint_states",
            "speed": float(LaunchConfiguration("speed").perform(context)),
            "use_sim_time": LaunchConfiguration("use_sim_time").perform(context) == "true",
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("layout", default_value="dual", choices=["dual", "single"],
                              description="dual = สองแขน ar_dual, single = แขนเดี่ยวแบบเดิม (ซิม/ar_hardware)"),
        DeclareLaunchArgument("arm_ns", default_value="",
                              description="(single เท่านั้น) namespace ของแขน ('' = ซิม/แขนเดี่ยว, real = แขนจริงในชุด dual_real_sim)"),
        DeclareLaunchArgument("speed", default_value="0.19",
                              description="ความเร็วช่วงวิ่งคงที่ rad/s (0.19 ≈ 11°/วิ = J1 ตอน calibrate = เพดาน)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
