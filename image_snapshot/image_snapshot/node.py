"""
Serve hk_camera_interfaces/TakePhoto for an image topic.

hk_camera_rtsp_pub answers ~/take_photo for the PTZ camera. Cameras without such a
service (the RealSense driver, simulated cameras) get this node next to them, so whoever
takes a photo calls the same service whatever the camera is.

Parameters:
  image_topic        topic to follow, relative names resolve in the node's namespace
  transport          raw | compressed | auto (compressed when the topic ends in /compressed)
  max_frame_age_sec  refuse a photo when the newest frame is older than this; 0 disables
"""

from hk_camera_interfaces.srv import TakePhoto
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image

from . import frames


class ImageSnapshot(Node):
    """Keep the newest frame of one topic and write it to disk on request."""

    def __init__(self):
        super().__init__('image_snapshot')
        self.image_topic = self.declare_parameter('image_topic', 'image').value
        transport = self.declare_parameter('transport', 'auto').value
        self.max_frame_age = float(self.declare_parameter('max_frame_age_sec', 2.0).value)

        if transport == 'auto':
            transport = 'compressed' if self.image_topic.endswith('/compressed') else 'raw'
        if transport not in ('raw', 'compressed'):
            raise ValueError(f'transport must be raw, compressed or auto, got {transport!r}')
        self.compressed = transport == 'compressed'

        self.last_frame = None
        self.last_frame_received = None
        # Best effort reads reliable publishers too, and never stalls a camera on us
        self.create_subscription(
            CompressedImage if self.compressed else Image,
            self.image_topic,
            self._on_frame,
            qos_profile_sensor_data,
        )
        self.create_service(TakePhoto, '~/take_photo', self._take_photo)
        self.get_logger().info(
            f'Serving {self.resolve_service_name("~/take_photo")} from '
            f'{self.resolve_topic_name(self.image_topic)} ({transport})'
        )

    def _on_frame(self, msg):
        self.last_frame = msg
        self.last_frame_received = self.get_clock().now()

    def _fail(self, response, message):
        response.success = False
        response.message = message
        self.get_logger().error(f'take_photo failed: {message}')
        return response

    def _take_photo(self, request, response):
        path = request.save_path
        if not path:
            return self._fail(response, 'save_path is empty.')
        if self.last_frame is None:
            return self._fail(response, f'No frame received on {self.image_topic} yet.')

        age = frames.frame_age(
            self.last_frame_received.nanoseconds / 1e9,
            self.get_clock().now().nanoseconds / 1e9,
        )
        if self.max_frame_age > 0.0 and age > self.max_frame_age:
            return self._fail(
                response,
                f'The newest frame on {self.image_topic} is {age:.1f}s old; '
                'the camera stream has stopped.',
            )

        frame = self.last_frame
        try:
            if self.compressed:
                frames.write_compressed(path, frame.data, frame.format)
            else:
                frames.write_raw(
                    path, frame.data, frame.height, frame.width, frame.step, frame.encoding
                )
        except (OSError, ValueError) as error:
            return self._fail(response, str(error))

        stamp = frame.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.last_frame_received.to_msg()
        response.success = True
        response.message = f'Image saved to {path}'
        response.stamp = stamp
        self.get_logger().info(
            f'Photo (captured at {Time.from_msg(stamp).nanoseconds / 1e9:.3f}s) saved to {path}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
