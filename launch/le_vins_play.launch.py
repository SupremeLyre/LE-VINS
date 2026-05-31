from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    configfile = LaunchConfiguration("configfile")
    default_configfile = PathJoinSubstitution(
        [FindPackageShare("le_vins"), "config", "le_vins_robot.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("le_vins"), "config", "visualization.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("configfile", default_value=default_configfile),
            Node(
                package="le_vins",
                executable="le_vins_ros",
                name="le_vins_ros",
                output="screen",
                arguments=[configfile, "0"],
                parameters=[{"is_read_bag": False}],
                on_exit=Shutdown(),
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
