"""แขน AR4 สองตัวพร้อมกัน: ros2_control (Teensy สองตัว) + robot_state_publisher + ตัวคุมสองแขน

  ros2 launch ar_dual dual_hardware.launch.py                    # สองแขนจริง (arm start เรียกตัวนี้)
  ros2 launch ar_dual dual_hardware.launch.py right_hw:=mock     # แขนซ้ายจริง แขนขวาจำลอง (ไม่ต่อบอร์ด)
  ros2 launch ar_dual dual_hardware.launch.py left_hw:=mock right_hw:=mock   # จำลองทั้งคู่ ไว้ลองโปรแกรม

ตัวคุม: /left_arm_controller/follow_joint_trajectory, /right_arm_controller/follow_joint_trajectory
ข้อต่อ: left_joint_1..6, right_joint_1..6 · TF: torso -> left_base_link / right_base_link -> ...
ไม่ calibrate ที่นี่ — calibrate ก่อนด้วย ar_calibrate.py (arm calibrate left|right) แล้วค่อยเปิดตัวนี้
พอร์ตต้องอ้างด้วย /dev/serial/by-path: Teensy ทั้งสองตัวรายงานหมายเลขเดียวกัน by-id จึงแยกไม่ได้
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    controllers = PathJoinSubstitution([FindPackageShare("ar_dual"), "config", "controllers.yaml"])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", "/controller_manager", "--controller-manager-timeout", "60"])

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, (default, desc) in ARGS.items()] + [
            Node(package="controller_manager", executable="ros2_control_node",
                 parameters=[robot_description, controllers], output="screen"),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[robot_description], output="both"),
            spawner("joint_state_broadcaster"),
            spawner("left_arm_controller"),
            spawner("right_arm_controller"),
        ])
