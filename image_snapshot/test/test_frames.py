"""Frame conversions of the snapshot service."""

import cv2
from image_snapshot import frames
import numpy as np
import pytest


def _raw(image: np.ndarray, padding: int = 0):
    """Lay an image out as sensor_msgs/Image bytes, with optional row padding."""
    height, width = image.shape[:2]
    row = image.reshape(height, -1)
    step = row.shape[1] + padding
    padded = np.zeros((height, step), dtype=np.uint8)
    padded[:, : row.shape[1]] = row
    return padded.tobytes(), height, width, step


def test_rgb_frames_are_written_as_bgr(tmp_path):
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    rgb[..., 0] = 200  # red in RGB
    data, height, width, step = _raw(rgb)
    path = str(tmp_path / 'photo.png')

    frames.write_raw(path, data, height, width, step, 'rgb8')

    written = cv2.imread(path)
    assert written.shape == (4, 6, 3)
    assert written[0, 0].tolist() == [0, 0, 200]


def test_row_padding_is_dropped():
    bgr = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    data, height, width, step = _raw(bgr, padding=7)

    assert np.array_equal(frames.raw_to_bgr(data, height, width, step, 'bgr8'), bgr)


@pytest.mark.parametrize('encoding', ['mono8', 'rgba8', 'bgra8'])
def test_other_colour_encodings_convert(encoding):
    channels = {'mono8': 1, 'rgba8': 4, 'bgra8': 4}[encoding]
    image = np.full((2, 2, channels), 50, dtype=np.uint8)
    data, height, width, step = _raw(image)

    assert frames.raw_to_bgr(data, height, width, step, encoding).shape == (2, 2, 3)


def test_depth_frames_are_refused():
    with pytest.raises(ValueError, match='32FC1'):
        frames.raw_to_bgr(b'\x00' * 16, 2, 2, 8, '32FC1')


def test_jpeg_frames_are_copied_untouched(tmp_path):
    ok, jpeg = cv2.imencode('.jpg', np.full((8, 8, 3), 90, dtype=np.uint8))
    assert ok
    path = tmp_path / 'photo.jpg'

    frames.write_compressed(str(path), jpeg.tobytes(), 'jpeg')

    assert path.read_bytes() == jpeg.tobytes()


def test_compressed_frames_are_reencoded_for_another_extension(tmp_path):
    ok, png = cv2.imencode('.png', np.full((8, 8, 3), 90, dtype=np.uint8))
    assert ok
    path = tmp_path / 'photo.jpg'

    frames.write_compressed(str(path), png.tobytes(), 'png')

    assert path.read_bytes()[:2] == b'\xff\xd8'


def test_unwritable_path_is_an_error():
    data, height, width, step = _raw(np.zeros((2, 2, 3), dtype=np.uint8))
    with pytest.raises(OSError):
        frames.write_raw('/nonexistent/dir/photo.jpg', data, height, width, step, 'bgr8')


def test_frame_age_never_negative():
    assert frames.frame_age(10.0, 12.5) == 2.5
    assert frames.frame_age(10.0, 9.0) == 0.0
