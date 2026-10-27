"""แขน AR4 สองตัวพร้อมกัน: ros2_control แขนละโปรเซส + robot_state_publisher + ตัวรวม joint_states

  ros2 launch ar_dual dual_hardware.launch.py                    # สองแขนจริง (arm start เรียกตัวนี้)
  ros2 launch ar_dual dual_hardware.launch.py right_hw:=mock     # แขนซ้ายจริง แขนขวาจำลอง (ไม่ต่อบอร์ด)
  ros2 launch ar_dual dual_hardware.launch.py left_hw:=mock right_hw:=mock   # จำลองทั้งคู่ ไว้ลองโปรแกรม

แต่ละแขนมีตัวคุมของตัวเอง (launch/arm_hardware.launch.py) — ปิด/คาลิเบรต/เปิดแขนเดียวได้ อีกแขนทำงานต่อ
(arm recal / arm reconnect) · joint_state_publisher รวม /left/joint_states + /right/joint_states เป็น /joint_states
ครบ 12 ข้อ (แขนที่ปิดอยู่ = ค่าล่าสุดค้างไว้ Teensy ถือท่านั้นอยู่)
ตัวคุม: /left_arm_controller/follow_joint_trajectory, /right_arm_controller/follow_joint_trajectory
ข้อต่อ: left_joint_1..6, right_joint_1..6 · TF: torso -> left_base_link / right_base_link -> ...
ไม่ calibrate ที่นี่ — calibrate ก่อนด้วย ar_calibrate.py (arm calibrate left|right) แล้วค่อยเปิดตัวนี้
พอร์ตต้องอ้างด้วย /dev/serial/by-path: Teensy ทั้งสองตัวรายงานหมายเลขเดียวกัน by-id จึงแยกไม่ได้
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = {
    "left_port": ("/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0", "พอร์ต Teensy แขนซ้าย"),
    "right_port": ("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0", "พอร์ต Teensy แขนขวา"),
    "left_hw": ("real", "real = ต่อ Teensy จริง, mock = จำลอง"),
    "right_hw": ("real", "real = ต่อ Teensy จริง, mock = จำลอง"),
}


def generate_launch_description():
    xacro_cmd = [PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                 PathJoinSubstitution([FindPackageShare("ar_dual"), "urdf", "humanoi_arms.urdf.xacro"])]
    for name in ARGS:
        xacro_cmd += [f" {name}:=", LaunchConfiguration(name)]
    robot_description = {"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}
    arm_launch = PathJoinSubstitution([FindPackageShare("ar_dual"), "launch", "arm_hardware.launch.py"])

    def arm(side):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_launch),
            launch_arguments={"arm": side, "hw": LaunchConfiguration(f"{side}_hw"),
                              "port": LaunchConfiguration(f"{side}_port")}.items())

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, (default, desc) in ARGS.items()] + [
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[robot_description], output="both"),
            Node(package="joint_state_publisher", executable="joint_state_publisher", output="log",
                 parameters=[{"source_list": ["/left/joint_states", "/right/joint_states"], "rate": 50.0}]),
            arm("left"),
            arm("right"),
        ])
