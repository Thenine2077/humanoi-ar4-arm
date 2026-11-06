"""Start the pose program player (arm_program)

  # two real arms on the Pi (ar_dual dual_hardware.launch.py) — default
  ros2 launch ar_program arm_program.launch.py
  # single-arm sim (ar_gz_bringup ar_gz.launch.py) or a single real arm (ar_hardware.launch.py)
  ros2 launch ar_program arm_program.launch.py layout:=single
  # real arm in the ar_mirror dual_real_sim.launch.py setup (the real arm is under the /real namespace)
  ros2 launch ar_program arm_program.launch.py layout:=single arm_ns:=real

In the dual_real_sim setup, feeding programs to the "sim" (empty arm_ns) is enough — joint_mirror makes the real arm follow
Use arm_ns:=real only when you want to command the real arm directly, bypassing the sim
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
                              description="dual = two arms (ar_dual), single = old single arm (sim/ar_hardware)"),
        DeclareLaunchArgument("arm_ns", default_value="",
                              description="(single only) arm namespace ('' = sim/single arm, real = real arm in the dual_real_sim setup)"),
        DeclareLaunchArgument("speed", default_value="0.19",
                              description="cruise speed in rad/s (0.19 ≈ 11°/s = J1 during calibrate = the ceiling)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
