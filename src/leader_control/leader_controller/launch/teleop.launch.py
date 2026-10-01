"""Start one Feetech owner, calibrated leader state and disarmed Mirabo teleop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory('leader_controller'),
        'config', 'teleop.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('teleop_config', default_value=teleop_config),
        LogInfo(msg=['Teleop parameter file: ', LaunchConfiguration('teleop_config')]),
        Node(
            package='feetech_driver', executable='feetech_node',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'enable_commands': False,
            }],
        ),
        Node(package='leader_state', executable='leader_state'),
        Node(package='leader_controller', executable='teleop',
             parameters=[LaunchConfiguration('teleop_config')]),
    ])
