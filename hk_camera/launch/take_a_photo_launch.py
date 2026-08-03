from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Start the camera node
    camera_pub_node = Node(
        package='hk_camera',
        executable='hk_camera_compressed_pub_test_reliable'
    )

    launch_description = LaunchDescription([camera_pub_node])
    return launch_description