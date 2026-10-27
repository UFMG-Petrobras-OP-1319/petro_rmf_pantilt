"""
Frame handling for the snapshot service, kept free of ROS so it can be tested alone.

A snapshot keeps the newest frame of a topic as it arrived and only converts it when a
photo is asked for, so following a 30 Hz camera costs nothing but a reference.
"""

import cv2
import numpy as np

# Raw encodings OpenCV can write once converted to BGR
_TO_BGR = {
    'bgr8': None,
    'rgb8': cv2.COLOR_RGB2BGR,
    'bgra8': cv2.COLOR_BGRA2BGR,
    'rgba8': cv2.COLOR_RGBA2BGR,
    'mono8': cv2.COLOR_GRAY2BGR,
}


def raw_to_bgr(data, height: int, width: int, step: int, encoding: str) -> np.ndarray:
    """
    Turn the bytes of a sensor_msgs/Image into a BGR array.

    Raises ValueError for an encoding a photo cannot be written from (depth, Bayer).
    """
    key = encoding.lower()
    if key not in _TO_BGR:
        raise ValueError(f'cannot take a photo from a {encoding!r} image')
    channels = 1 if key == 'mono8' else (4 if key in ('bgra8', 'rgba8') else 3)
    rows = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    image = rows[:, : width * channels].reshape(height, width, channels)
    conversion = _TO_BGR[key]
    if conversion is None:
        return image
    if key == 'mono8':
        image = image.reshape(height, width)
    return cv2.cvtColor(image, conversion)


def write_raw(path: str, data, height: int, width: int, step: int, encoding: str) -> None:
    """Write a raw frame to path; the extension picks the format."""
    image = raw_to_bgr(data, height, width, step, encoding)
    if not cv2.imwrite(path, image):
        raise OSError(f'could not write {path!r}; check that its directory exists')


def write_compressed(path: str, data, image_format: str) -> None:
    """
    Write a sensor_msgs/CompressedImage to path.

    A JPEG frame saved as .jpg is copied byte for byte; anything else is decoded and
    written in the format the extension asks for.
    """
    payload = bytes(data)
    is_jpeg = payload[:2] == b'\xff\xd8'
    if is_jpeg and path.lower().endswith(('.jpg', '.jpeg')):
        with open(path, 'wb') as file:
            file.write(payload)
        return
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'cannot decode a {image_format!r} compressed frame')
    if not cv2.imwrite(path, image):
        raise OSError(f'could not write {path!r}; check that its directory exists')


def frame_age(received_sec: float, now_sec: float) -> float:
    """Seconds since a frame arrived, never negative (clocks can step back in simulation)."""
    return max(0.0, now_sec - received_sec)
