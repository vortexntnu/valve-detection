import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    valve_detection_node = Node(
        package='valve_detection',
        executable='valve_detection_node',
        name='valve_detection_node',
        parameters=[
            os.path.join(
                get_package_share_directory('valve_detection'),
                'config',
                'valve_detection_params.yaml',
            ),
            {'use_sim_time': True},
        ],
        output='screen',
    )
    return LaunchDescription([valve_detection_node])
