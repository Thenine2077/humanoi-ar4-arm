"""แขน AR4 หนึ่งแขน: ros2_control ของแขนนั้นอย่างเดียว (โปรเซสแยก) + ตัวคุมของแขนนั้น

  ros2 launch ar_dual arm_hardware.launch.py arm:=right port:=/dev/serial/by-path/...   # arm reconnect / recal เรียกตัวนี้
  ros2 launch ar_dual arm_hardware.launch.py arm:=right hw:=mock                         # แขนจำลอง

ต้องมี robot_state_publisher + joint_state_publisher เปิดอยู่แล้ว (dual_hardware.launch.py เปิดให้)
ปิดตัวนี้ = ปล่อยพอร์ต Teensy ของแขนนี้ (calibrate ได้) อีกแขนทำงานต่อ · Teensy ถือท่าค้างไว้ แขนไม่ตก
ตัวคุม: <arm>_controller_manager -> <arm>_joint_state_broadcaster (/<arm>/joint_states) + <arm>_arm_controller
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def arm_nodes(context):
    arm = LaunchConfiguration("arm").perform(context)
    if arm not in ("left", "right"):
        raise RuntimeError(f"arm ต้องเป็น left หรือ right (ได้ '{arm}')")
    other = "right" if arm == "left" else "left"
    # robot_description ของตัวคุมนี้มี ros2_control เฉพาะแขนนี้ (อีกแขน hw:=none) — แต่ละตัวโหลดแค่บอร์ดตัวเอง
    xacro_cmd = [PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                 PathJoinSubstitution([FindPackageShare("ar_dual"), "urdf", "humanoi_arms.urdf.xacro"]),
                 f" {arm}_hw:=", LaunchConfiguration("hw"), f" {arm}_port:=", LaunchConfiguration("port"),
                 f" {other}_hw:=none"]
    robot_description = {"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}
    controllers = PathJoinSubstitution([FindPackageShare("ar_dual"), "config", "controllers_per_arm.yaml"])
    cm = f"{arm}_controller_manager"

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", f"/{cm}", "--controller-manager-timeout", "60"])

    return [
        # ros2_control ของ Jazzy อ่านโมเดลจากหัวข้อ robot_description เท่านั้น (พารามิเตอร์ไม่ใช้) — ถ้ารับ
        # /robot_description (โมเดลเต็ม มี ros2_control สองแขน) ตัวคุมนี้จะเปิดพอร์ต Teensy ของอีกแขนด้วย
        # จึงส่งโมเดลเฉพาะแขนนี้ไปหัวข้อ /<arm>/robot_description ด้วย robot_state_publisher อีกตัว (TF ของตัวนี้ทิ้ง)
        Node(package="robot_state_publisher", executable="robot_state_publisher", name=f"{arm}_description_publisher",
             parameters=[robot_description], output="log",
             remappings=[("/robot_description", f"/{arm}/robot_description"),
                         ("/joint_states", f"/{arm}/description_unused/joint_states"),
                         ("/tf", f"/{arm}/description_unused/tf"),
                         ("/tf_static", f"/{arm}/description_unused/tf_static")]),
        Node(package="controller_manager", executable="ros2_control_node", name=cm, output="screen",
             parameters=[controllers],
             remappings=[("/joint_states", f"/{arm}/joint_states"),
                         ("/dynamic_joint_states", f"/{arm}/dynamic_joint_states"),
                         ("/robot_description", f"/{arm}/robot_description"),
                         ("~/robot_description", f"/{arm}/robot_description")]),
        spawner(f"{arm}_joint_state_broadcaster"),
        spawner(f"{arm}_arm_controller"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", description="left | right"),
        DeclareLaunchArgument("hw", default_value="real", description="real = ต่อ Teensy จริง, mock = จำลอง"),
        DeclareLaunchArgument("port", default_value="none", description="พอร์ต Teensy (/dev/serial/by-path/...)"),
        OpaqueFunction(function=arm_nodes),
    ])
