"""Standalone launch for apriltag_sim.

Publishes camera_optical_frame -> tag_name TFs from the sim ground-truth pose
and map-frame tag positions declared in config/apriltag_sim.yaml. Mimics
apriltag-detector so downstream consumers (amr_api::_getHomeTags, cart_detect,
tf_location) work unchanged in sim.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('apriltag_sim'),
        'config',
        'apriltag_sim.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to apriltag_sim parameter YAML.',
        ),
        Node(
            package='apriltag_sim',
            executable='apriltag_sim_node',
            name='apriltag_sim',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
