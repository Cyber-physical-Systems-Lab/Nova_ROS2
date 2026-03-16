import math
from typing import Union

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def _parse_device(value: object) -> Union[int, str]:
    text = str(value)
    try:
        return int(text)
    except ValueError:
        return text


def _param_text(node: Node, name: str) -> str:
    return str(node.get_parameter(name).value)


def _param_int(node: Node, name: str) -> int:
    return int(node.get_parameter(name).value or 0)


def _param_float(node: Node, name: str) -> float:
    return float(node.get_parameter(name).value or 0.0)


def _default_calibration_path() -> str:
    return get_package_share_directory('robotiq_2f85_bringup') + '/config/camera_calibration.yaml'


def _load_camera_info(calibration_file: str, frame_id: str) -> CameraInfo:
    with open(calibration_file, 'r', encoding='utf-8') as handle:
        calibration = yaml.safe_load(handle)

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(calibration['image_width'])
    msg.height = int(calibration['image_height'])
    msg.distortion_model = str(calibration['distortion_model'])
    msg.d = [float(value) for value in calibration['distortion_coefficients']['data']]
    msg.k = [float(value) for value in calibration['camera_matrix']['data']]
    msg.r = [float(value) for value in calibration['rectification_matrix']['data']]
    msg.p = [float(value) for value in calibration['projection_matrix']['data']]
    return msg


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('camera_publisher')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('calibration_file', _default_calibration_path())

        device = _parse_device(_param_text(self, 'device'))
        topic = _param_text(self, 'topic')
        camera_info_topic = _param_text(self, 'camera_info_topic')
        self._frame_id = _param_text(self, 'frame_id')
        width = _param_int(self, 'width')
        height = _param_int(self, 'height')
        fps = _param_float(self, 'fps')
        calibration_file = _param_text(self, 'calibration_file')

        self._publisher = self.create_publisher(Image, topic, 10)
        self._camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self._camera_info = _load_camera_info(calibration_file, self._frame_id)

        if isinstance(device, str) and device.startswith('/dev/video'):
            self._capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        else:
            self._capture = cv2.VideoCapture(device)
        if not self._capture.isOpened():
            raise RuntimeError(f'Unable to open camera source: {device}')

        # Match the camera profile reported by v4l2-ctl.
        fourcc_fn = getattr(cv2, 'VideoWriter_fourcc', None)
        if callable(fourcc_fn):
            fourcc_value = fourcc_fn(*'YUYV')
            if isinstance(fourcc_value, (int, float)):
                self._capture.set(cv2.CAP_PROP_FOURCC, float(fourcc_value))
        if width > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0.0:
            self._capture.set(cv2.CAP_PROP_FPS, fps)

        timer_period = 1.0 / fps if fps > 0.0 and math.isfinite(fps) else 1.0 / 30.0
        self._timer = self.create_timer(timer_period, self._publish_frame)

        self.get_logger().info(
            f'Publishing camera frames from {device} to {topic} and camera info to '
            f'{camera_info_topic} using calibration {calibration_file}'
        )
        self.get_logger().info(
            'Requested capture mode: YUYV '
            f'{int(width)}x{int(height)} @ {fps:.1f} Hz'
        )

    def destroy_node(self):
        if hasattr(self, '_capture') and self._capture is not None:
            self._capture.release()
        return super().destroy_node()

    def _publish_frame(self) -> None:
        success, frame = self._capture.read()
        if not success or frame is None:
            self.get_logger().warning('Failed to read frame from camera', throttle_duration_sec=5.0)
            return

        msg = Image()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        self._publisher.publish(msg)

        self._camera_info.header.stamp = stamp
        self._camera_info.header.frame_id = self._frame_id
        self._camera_info.width = msg.width
        self._camera_info.height = msg.height
        self._camera_info_publisher.publish(self._camera_info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()