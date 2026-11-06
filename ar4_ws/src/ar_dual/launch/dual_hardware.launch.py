"""Both AR4 arms together: ros2_control in one process per arm + robot_state_publisher + joint_states merger

  ros2 launch ar_dual dual_hardware.launch.py                    # both arms real (arm start calls this)
  ros2 launch ar_dual dual_hardware.launch.py right_hw:=mock     # left arm real, right arm simulated (no board)
  ros2 launch ar_dual dual_hardware.launch.py left_hw:=mock right_hw:=mock   # both simulated, for trying programs

Each arm has its own controller (launch/arm_hardware.launch.py) — one arm can be stopped/calibrated/restarted while the other keeps working
(arm recal / arm reconnect) · joint_state_publisher merges /left/joint_states + /right/joint_states into /joint_states
with all 12 joints (a stopped arm = last values held; the Teensy holds that pose)
Controllers: /left_arm_controller/follow_joint_trajectory, /right_arm_controller/follow_joint_trajectory
Joints: left_joint_1..6, right_joint_1..6 · TF: torso -> left_base_link / right_base_link -> ...
No calibration here — calibrate first with ar_calibrate.py (arm calibrate left|right), then start this
motion_log (ar_program) records every motion so arm broke can save it when an incident happens (belt slip, collision ...)
Ports must be given as /dev/serial/by-path: both Teensys report the same serial number, so by-id cannot tell them apart
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = {
    "left_port": ("/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0", "left arm Teensy port"),
    "right_port": ("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0", "right arm Teensy port"),
    "left_hw": ("real", "real = real Teensy, mock = simulated"),
    "right_hw": ("real", "real = real Teensy, mock = simulated"),
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
            Node(package="ar_program", executable="motion_log", output="log"),
        ])
