import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("valve_detection"),
        "config",
        "valve_detection_params.yaml",
    )

    container = ComposableNodeContainer(
        name="valve_detection_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="valve_detection",
                plugin="valve_detection::ValvePoseNode",
                name="valve_pose_node",
                parameters=[cfg],
            )
        ],
        output="screen",
    )

    return LaunchDescription([container])
