"""Launch RViz2 with the HanMole navigation display config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("hanmole_navigation"), "rviz", "navigation.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Navigation RViz display config file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Whether RViz should use simulation time.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_navigation",
                output="screen",
                arguments=["--display-config", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
