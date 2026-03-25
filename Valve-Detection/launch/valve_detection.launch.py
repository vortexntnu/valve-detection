import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('valve_detection'),
        'config',
        'valve_detection_params.yaml',
    )

    drone_arg = DeclareLaunchArgument(
        'drone',
        default_value='moby',
        description='Robot name, prepended to TF frame IDs (e.g. moby, orca)',
    )

    debug_visualize_arg = DeclareLaunchArgument(
        'debug_visualize',
        default_value='false',
        description='Enable debug visualization topics (valve_pose, valve_poses, annulus, plane)',
    )

    container = ComposableNodeContainer(
        name='valve_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='valve_detection',
                plugin='valve_detection::ValvePoseNode',
                name='valve_pose_node',
                parameters=[
                    cfg,
                    {
                        'drone': LaunchConfiguration('drone'),
                        'debug_visualize': LaunchConfiguration('debug_visualize'),
                    },
                ],
            )
        ],
        output='screen',
    )

    return LaunchDescription([drone_arg, debug_visualize_arg, container])
