"""Start the HIKROBOT industrial camera node + rviz2 to view the live image.

All camera settings are read from config/industrial_camera.yaml. Edit that file to
change the defaults; command line arguments override it when given.

Usage:
    ros2 launch hk_camera view_camera_launch.py
    ros2 launch hk_camera view_camera_launch.py ExposureTime:=20000
    ros2 launch hk_camera view_camera_launch.py rviz:=false
    ros2 launch hk_camera view_camera_launch.py config_file:=/path/to/my_camera.yaml
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

CAMERA_NODE = 'hk_camera'

# frame_id of the camera image; must match what hk_camera.cpp publishes
CAMERA_FRAME = 'hk_camera'

# Launch argument -> (node it belongs to, parameter name in the config file).
# Every one of these defaults to '' meaning "use whatever the config file says".
OVERRIDES = {
    'width': [(CAMERA_NODE, 'width')],
    'height': [(CAMERA_NODE, 'height')],
    'ExposureTime': [(CAMERA_NODE, 'ExposureTime')],
    'FrameRate': [(CAMERA_NODE, 'FrameRate')],
    'GainAuto': [(CAMERA_NODE, 'GainAuto')],
}


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)

    camera_params = load_node_parameters(config_file, CAMERA_NODE)
    override_parameters({CAMERA_NODE: camera_params}, OVERRIDES, context)

    camera_node = Node(
        package='hk_camera',
        executable='hk_camera',
        name=CAMERA_NODE,
        output='screen',
        parameters=[camera_params],
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

    return [camera_node, static_tf_node, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('hk_camera')
    default_config = os.path.join(pkg_share, 'config', 'industrial_camera.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'hk_camera.rviz')

    args = [
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Parameter file the camera node is configured from'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Also start rviz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='Path to the rviz2 config file'),
    ]

    # These all default to empty, which means "keep the value from config_file".
    descriptions = {
        'width': 'Image width',
        'height': 'Image height',
        'ExposureTime': 'Exposure time in microseconds',
        'FrameRate': 'Acquisition frame rate in Hz',
        'GainAuto': 'Auto gain: 0=off, 1=once, 2=continuous',
    }
    args += [
        DeclareLaunchArgument(name, default_value='',
                              description=f'{text} (overrides config_file)')
        for name, text in descriptions.items()
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
