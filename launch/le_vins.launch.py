from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    configfile = LaunchConfiguration("configfile")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("le_vins"), "config", "visualization.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("configfile", default_value=""),
            Node(
                package="le_vins",
                executable="le_vins_ros",
                name="le_vins_ros",
                output="screen",
                arguments=[configfile, "1"],
                parameters=[{"is_read_bag": False}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_map_broadcaster",
                arguments=["0", "0", "0", "-1", "0", "0", "0", "map", "world"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="visualisation",
                output="log",
                arguments=["-d", rviz_config],
            ),
        ]
    )
