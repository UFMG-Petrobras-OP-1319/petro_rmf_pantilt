"""Start the HIKVISION network camera (RTSP) node + PTZ control + rviz2.

For HIKVISION network cameras (IP cameras) that stream over RTSP, not for GigE Vision
industrial cameras. Use view_camera_launch.py for industrial cameras.

All parameters are read from config/network_camera.yaml. Edit that file to set your
camera IP and password once; command line arguments override it when given.

Usage:
    ros2 launch hk_camera view_rtsp_camera_launch.py
    ros2 launch hk_camera view_rtsp_camera_launch.py password:=YOUR_PASSWORD
    ros2 launch hk_camera view_rtsp_camera_launch.py channel:=102
    ros2 launch hk_camera view_rtsp_camera_launch.py rviz:=false
    ros2 launch hk_camera view_rtsp_camera_launch.py ptz:=false
    ros2 launch hk_camera view_rtsp_camera_launch.py config_file:=/path/to/my_camera.yaml

PTZ control (the node starts together with this launch file by default):
    # pan right at 40% speed
    ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.4}}'
    # stop
    ros2 topic pub -1 /hk_camera_ptz/cmd_vel geometry_msgs/msg/Twist '{}'
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch files are executed by path, so their own directory is not on sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hk_camera_config import load_node_parameters, override_parameters  # noqa: E402

CAMERA_NODE = 'hk_camera_rtsp'
PTZ_NODE = 'hk_camera_ptz'

# Launch argument -> (node it belongs to, parameter name in the config file).
# Every one of these defaults to '' meaning "use whatever the config file says".
OVERRIDES = {
    'host': [(CAMERA_NODE, 'host'), (PTZ_NODE, 'host')],
    'username': [(CAMERA_NODE, 'username'), (PTZ_NODE, 'username')],
    'password': [(CAMERA_NODE, 'password'), (PTZ_NODE, 'password')],
    'port': [(CAMERA_NODE, 'port')],
    'channel': [(CAMERA_NODE, 'channel')],
    'rtsp_url': [(CAMERA_NODE, 'rtsp_url')],
    'use_tcp': [(CAMERA_NODE, 'use_tcp')],
    'publish_rate': [(CAMERA_NODE, 'publish_rate')],
    'http_port': [(PTZ_NODE, 'port')],
    'ptz_channel': [(PTZ_NODE, 'channel')],
}


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)

    camera_params = load_node_parameters(config_file, CAMERA_NODE)
    ptz_params = load_node_parameters(config_file, PTZ_NODE)
    params = {CAMERA_NODE: camera_params, PTZ_NODE: ptz_params}
    override_parameters(params, OVERRIDES, context)

    # The rviz2 Fixed Frame must exist in the TF tree, otherwise the display errors out.
    # Use whichever frame_id the camera node actually stamps on its images.
    camera_frame = camera_params.get('frame_id', 'hk_camera')

    camera_node = Node(
        package='hk_camera',
        executable='hk_camera_rtsp_pub',
        name=CAMERA_NODE,
        output='screen',
        parameters=[camera_params],
    )

    # PTZ goes over ISAPI (HTTP); it is a separate connection from the RTSP video stream
    ptz_node = Node(
        package='hk_camera',
        executable='hk_camera_ptz',
        name=PTZ_NODE,
        output='screen',
        parameters=[ptz_params],
        condition=IfCondition(LaunchConfiguration('ptz')),
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hk_camera_static_tf',
        arguments=['--frame-id', 'map', '--child-frame-id', camera_frame],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [camera_node, ptz_node, static_tf_node, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('hk_camera')
    default_config = os.path.join(pkg_share, 'config', 'network_camera.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'hk_camera.rviz')

    args = [
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Parameter file the nodes are configured from'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Also start rviz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='Path to the rviz2 config file'),
        DeclareLaunchArgument('ptz', default_value='true',
                              description='Also start the PTZ control node'),
    ]

    # These all default to empty, which means "keep the value from config_file".
    descriptions = {
        'host': 'Camera IP address',
        'username': 'Login user name',
        'password': 'Login password (HIKVISION uses Digest auth)',
        'port': 'RTSP port',
        'channel': 'Stream channel: 101=main stream, 102=sub stream',
        'rtsp_url': 'Full RTSP URL; when set it overrides host/username/etc.',
        'use_tcp': 'Carry RTSP over TCP (more stable) instead of UDP',
        'publish_rate': 'Publish rate in Hz; 0 follows the camera frame rate',
        'http_port': 'ISAPI HTTP port (used for PTZ), not the RTSP port 554',
        'ptz_channel': 'ISAPI PTZ channel number, normally 1',
    }
    args += [
        DeclareLaunchArgument(name, default_value='',
                              description=f'{text} (overrides config_file)')
        for name, text in descriptions.items()
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
