import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("valve_subtype_resolver"),
        "config",
        "valve_subtype_resolver_params.yaml",
    )

    drone_arg = DeclareLaunchArgument(
        "drone",
        default_value="moby",
        description="Robot name, prepended to TF frame IDs (e.g. moby, orca)",
    )

    return LaunchDescription(
        [
            drone_arg,
            Node(
                package="valve_subtype_resolver",
                executable="valve_subtype_resolver_node",
                name="valve_subtype_resolver_node",
                parameters=[config, {"drone": LaunchConfiguration("drone")}],
                output="screen",
            ),
        ]
    )
