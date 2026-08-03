"""Start the HIKVISION network camera (RTSP) node + rviz2 to view the live image.

For HIKVISION network cameras (IP cameras) that stream over RTSP, not for GigE Vision
industrial cameras. Use view_camera_launch.py for industrial cameras.

Usage:
    ros2 launch hk_camera view_rtsp_camera_launch.py password:=YOUR_PASSWORD
    ros2 launch hk_camera view_rtsp_camera_launch.py password:=xxx channel:=102
    ros2 launch hk_camera view_rtsp_camera_launch.py password:=xxx rviz:=false
    ros2 launch hk_camera view_rtsp_camera_launch.py password:=xxx ptz:=false

PTZ control (the node starts together with this launch file by default):
    # pan right at 40% speed
    ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.4}}'
    # stop
    ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{}'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# frame_id of the published image; must match the TF used in the rviz config
CAMERA_FRAME = 'hk_camera'


def generate_launch_description():
    pkg_share = get_package_share_directory('hk_camera')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'hk_camera.rviz')

    args = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Also start rviz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='Path to the rviz2 config file'),
        DeclareLaunchArgument('host', default_value='192.168.1.64',
                              description='Camera IP address'),
        DeclareLaunchArgument('port', default_value='554',
                              description='RTSP port'),
        DeclareLaunchArgument('username', default_value='admin',
                              description='Login user name'),
        DeclareLaunchArgument('password', default_value='',
                              description='Login password (required, HIKVISION uses Digest auth)'),
        DeclareLaunchArgument('channel', default_value='101',
                              description='Stream channel: 101=main stream, 102=sub stream'),
        DeclareLaunchArgument('rtsp_url', default_value='',
                              description='Full RTSP URL; when set it overrides host/username/etc.'),
        DeclareLaunchArgument('use_tcp', default_value='true',
                              description='Carry RTSP over TCP (more stable) instead of UDP'),
        DeclareLaunchArgument('publish_rate', default_value='0.0',
                              description='Publish rate in Hz; 0 follows the camera frame rate'),
        DeclareLaunchArgument('ptz', default_value='true',
                              description='Also start the PTZ control node'),
        DeclareLaunchArgument('http_port', default_value='80',
                              description='ISAPI HTTP port (used for PTZ), not the RTSP port 554'),
        DeclareLaunchArgument('ptz_channel', default_value='1',
                              description='ISAPI PTZ channel number, normally 1'),
    ]

    camera_node = Node(
        package='hk_camera',
        executable='hk_camera_rtsp_pub',
        name='hk_camera_rtsp',
        output='screen',
        # String parameters must declare value_type=str explicitly. Otherwise values like
        # channel:=101 or an all-digit password are inferred as integers by launch and the
        # node throws a type error on startup.
        parameters=[{
            'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'username': ParameterValue(LaunchConfiguration('username'), value_type=str),
            'password': ParameterValue(LaunchConfiguration('password'), value_type=str),
            'channel': ParameterValue(LaunchConfiguration('channel'), value_type=int),
            'rtsp_url': ParameterValue(LaunchConfiguration('rtsp_url'), value_type=str),
            'use_tcp': ParameterValue(LaunchConfiguration('use_tcp'), value_type=bool),
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
            'frame_id': CAMERA_FRAME,
        }],
    )

    # PTZ goes over ISAPI (HTTP); it is a separate connection from the RTSP video stream
    ptz_node = Node(
        package='hk_camera',
        executable='hk_camera_ptz',
        name='hk_camera_ptz',
        output='screen',
        parameters=[{
            'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
            'port': ParameterValue(LaunchConfiguration('http_port'), value_type=int),
            'username': ParameterValue(LaunchConfiguration('username'), value_type=str),
            'password': ParameterValue(LaunchConfiguration('password'), value_type=str),
            'channel': ParameterValue(LaunchConfiguration('ptz_channel'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('ptz')),
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

    return LaunchDescription(args + [camera_node, ptz_node, static_tf_node, rviz_node])
