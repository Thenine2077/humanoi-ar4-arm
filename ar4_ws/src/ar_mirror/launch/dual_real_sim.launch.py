"""Start the simulated arm (Gazebo) + the real arm (namespace 'real') together, and mirror them so they move together

  ros2 launch ar_mirror dual_real_sim.launch.py
  ros2 launch ar_mirror dual_real_sim.launch.py serial_port:=/dev/ttyACM0 calibrate:=True

Command it through MoveIt (ar_moveit_config, opened in a separate terminal window as usual) on the simulated arm as before
(drag the marker in RViz, then Plan & Execute); the real arm follows within ~1/rate seconds via joint_mirror

Note: the real-arm side is written fresh here (it doesn't include the vendor's ar_hardware_interface/ar_hardware.launch.py
directly) because that file passes "-c /controller_manager" as an absolute path, which can't find
controller_manager in namespace "real" — but it still uses the vendor's data files (urdf xacro,
controllers.yaml, controller_update_rate.yaml) all as is; no vendor file is edited at all

⚠️ MoveIt checks collisions against the sim scene only; it knows nothing about obstacles around the real arm
   Always clear the area around the real arm before running — the arm moves as soon as anyone commands the sim side
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

    # ---------- Simulated arm (Gazebo, default namespace) ----------
    # gui default = false: the Gazebo 3D window rendering through WSLg loads the CPU so heavily that
    # usbip (the Teensy's USB forwarded from Windows) times out and serial drops within a few seconds
    # watch the simulated arm in MoveIt's RViz instead (it shows both the sim and the motion plan anyway)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ar_gz_bringup"), "launch", "ar_gz.launch.py"
            )
        ),
        launch_arguments={"gui": LaunchConfiguration("gz_gui")}.items(),
    )

    # ---------- Real arm (namespace "real") ----------
    # all vendor data files, unmodified — reused as is
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

    # Use our own copy (ar_mirror/config/real_controllers.yaml) instead of the vendor's, because the key in
    # the vendor file is "controller_manager:", which only works in the root namespace — under /real it must be
    # "/real/controller_manager:", otherwise the controller gets no type and the spawner dies
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
        # tf2 always publishes to "/tf" as an absolute topic (namespace doesn't help) — without the remap, the real-arm side
        # would publish frames named link_1..link_6 clashing with the sim's frames of the same names, on a different time base too (the real arm uses
        # wall time, the sim uses sim time), so RViz floods TF_OLD_DATA and picks the one with the newer time
        # (the real arm's) instead of the sim's — split off to /real/tf so they don't clash
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

    # ---------- Mirror: sim /joint_states -> real joint_trajectory_controller ----------
    mirror = Node(
        package="ar_mirror",
        executable="joint_mirror",
        output="screen",
    )

    # Connect serial to the real arm "after" Gazebo has finished loading — if connected at the same time as Gazebo boots
    # (when it uses the most CPU) usbip times out and drops at once ("Error in transmit" ->
    # "read: End of file" -> hardware 'ar' gets deactivated -> controllers can't activate)
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
            description="Open the Gazebo 3D window? (false = saves CPU, avoids serial drops)",
        ),
        DeclareLaunchArgument(
            "real_start_delay",
            default_value="25.0",
            description="Seconds to wait before connecting the real arm (wait for Gazebo to finish loading)",
        ),
        sim,
        real_side_delayed,
    ])
