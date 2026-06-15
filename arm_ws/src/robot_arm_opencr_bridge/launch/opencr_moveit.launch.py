from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_moveit_launch(name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_arm_moveit_config"),
                "launch",
                name,
            ])
        )
    )


def generate_launch_description():
    port = LaunchConfiguration("port")
    execute = LaunchConfiguration("execute")
    max_step_deg = LaunchConfiguration("max_step_deg")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("max_step_deg", default_value="10.0"),

        include_moveit_launch("static_virtual_joint_tfs.launch.py"),
        include_moveit_launch("rsp.launch.py"),
        include_moveit_launch("move_group.launch.py"),
        include_moveit_launch("moveit_rviz.launch.py"),

        Node(
            package="robot_arm_opencr_bridge",
            executable="opencr_trajectory_bridge",
            name="opencr_trajectory_bridge",
            output="screen",
            parameters=[{
                "controller_name": "RM_controller",
                "execute": execute,
                "port": port,
                "baud": 115200,
                "max_step_deg": max_step_deg,
                "joint_min_deg": [-90.0, -90.0, -90.0, -90.0, -90.0, -90.0],
                "joint_max_deg": [90.0, 90.0, 90.0, 90.0, 90.0, 90.0],
                "motor_joint_names": [
                    "base_yaw_joint",
                    "shoulder_pitch_joint",
                    "upper_arm_roll_joint",
                    "elbow_pitch_joint",
                    "elbow_roll_joint",
                    "wrist_pitch_joint",
                ],
                "motor_signs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "motor_offsets_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }],
        ),
    ])
