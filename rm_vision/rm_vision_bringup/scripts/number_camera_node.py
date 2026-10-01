#!/usr/bin/env python3
"""V4L2 number camera publisher. Timestamps are taken immediately after capture."""

import cv2
from pathlib import Path
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class NumberCamera(Node):
    def __init__(self):
        super().__init__('number_camera')
        device = self.declare_parameter('device', 'auto').value
        camera_name_match = self.declare_parameter(
            'camera_name_match', 'USB 2.0 Camera: HD USB Camera').value
        if device == 'auto':
            candidates = []
            for node in sorted(Path('/dev').glob('video*')):
                sysfs = Path('/sys/class/video4linux') / node.name
                try:
                    name = (sysfs / 'name').read_text().strip()
                    index = int((sysfs / 'index').read_text().strip())
                except (OSError, ValueError):
                    continue
                if camera_name_match in name and index == 0:
                    candidates.append(str(node))
            if len(candidates) != 1:
                raise RuntimeError(
                    f'Expected one capture node matching {camera_name_match!r}, '
                    f'found {candidates}; check /dev/video* passthrough or set device explicitly')
            device = candidates[0]
        width = int(self.declare_parameter('width', 0).value)
        height = int(self.declare_parameter('height', 0).value)
        fps = float(self.declare_parameter('fps', 30.0).value)
        pixel_format = self.declare_parameter('pixel_format', 'MJPG').value
        self.frame_id = self.declare_parameter('frame_id', 'number_camera').value
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open number camera {device}')
        if len(pixel_format) != 4:
            raise RuntimeError('pixel_format must be a four-character V4L2 code')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        if width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, fps)
        actual = (self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                  self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                  self.cap.get(cv2.CAP_PROP_FPS))
        self.get_logger().info(f'{device}: actual width/height/fps={actual}')
        if width > 0 and int(actual[0]) != width or height > 0 and int(actual[1]) != height:
            self.get_logger().warning('Requested resolution was not accepted by camera')
        self.pub = self.create_publisher(
            Image, '/number_camera/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.failures = 0
        self.timer = self.create_timer(1.0 / max(fps, 1.0), self.capture)

    def capture(self):
        ok, frame = self.cap.read()
        stamp = self.get_clock().now().to_msg()
        if not ok:
            self.failures += 1
            if self.failures >= 5:
                self.get_logger().error('Number camera failed to read 5 consecutive frames')
                raise RuntimeError('Number camera read failed')
            return
        self.failures = 0
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = NumberCamera()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.cap.release()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
