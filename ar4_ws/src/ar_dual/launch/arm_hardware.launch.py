"""One AR4 arm: ros2_control for that arm only (separate process) + that arm's controllers

  ros2 launch ar_dual arm_hardware.launch.py arm:=right port:=/dev/serial/by-path/...   # arm reconnect / recal call this
  ros2 launch ar_dual arm_hardware.launch.py arm:=right hw:=mock                         # simulated arm

Needs robot_state_publisher + joint_state_publisher already running (dual_hardware.launch.py starts them)
Stopping this = releases this arm's Teensy port (calibrate possible) while the other arm keeps working · the Teensy holds its pose, the arm does not drop
Controllers: <arm>_controller_manager -> <arm>_joint_state_broadcaster (/<arm>/joint_states) + <arm>_arm_controller
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
        raise RuntimeError(f"arm must be left or right (got '{arm}')")
    other = "right" if arm == "left" else "left"
    # this controller's robot_description has ros2_control for this arm only (other arm hw:=none) — each one loads only its own board
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
        # Jazzy's ros2_control reads the model only from the robot_description topic (not the parameter) — if it subscribed to
        # /robot_description (full model, ros2_control for both arms) this controller would also open the other arm's Teensy port,
        # so this arm's model alone is published to /<arm>/robot_description by a second robot_state_publisher (its TF is discarded)
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
        DeclareLaunchArgument("hw", default_value="real", description="real = real Teensy, mock = simulated"),
        DeclareLaunchArgument("port", default_value="none", description="Teensy port (/dev/serial/by-path/...)"),
        OpaqueFunction(function=arm_nodes),
    ])
