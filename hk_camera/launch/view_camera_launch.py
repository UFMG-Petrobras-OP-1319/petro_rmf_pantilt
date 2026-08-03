"""Start the HIKROBOT industrial camera node + rviz2 to view the live image.

Usage:
    ros2 launch hk_camera view_camera_launch.py
    ros2 launch hk_camera view_camera_launch.py ExposureTime:=10000
    ros2 launch hk_camera view_camera_launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# frame_id of the camera image; must match what hk_camera.cpp publishes
CAMERA_FRAME = 'hk_camera'


def generate_launch_description():
    pkg_share = get_package_share_directory('hk_camera')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'hk_camera.rviz')

    args = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Also start rviz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='Path to the rviz2 config file'),
        # Exposure time is in microseconds. The 50000us (50ms) default overexposes
        # indoors and drags the frame rate below 20Hz.
        DeclareLaunchArgument('ExposureTime', default_value='10000',
                              description='Exposure time in microseconds'),
        DeclareLaunchArgument('width', default_value='1920',
                              description='Image width'),
        DeclareLaunchArgument('height', default_value='1200',
                              description='Image height'),
        DeclareLaunchArgument('FrameRate', default_value='30',
                              description='Acquisition frame rate in Hz'),
        DeclareLaunchArgument('GainAuto', default_value='2',
                              description='Auto gain: 0=off, 1=once, 2=continuous'),
    ]

    camera_node = Node(
        package='hk_camera',
        executable='hk_camera',
        name='hk_camera',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'ExposureTime': LaunchConfiguration('ExposureTime'),
            'FrameRate': LaunchConfiguration('FrameRate'),
            'GainAuto': LaunchConfiguration('GainAuto'),
        }],
    )

    # The rviz2 Fixed Frame must exist in the TF tree, otherwise the display errors out
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hk_camera_static_tf',
        arguments=['--frame-id', 'map', '--child-frame-id', CAMERA_FRAME],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [camera_node, static_tf_node, rviz_node])
