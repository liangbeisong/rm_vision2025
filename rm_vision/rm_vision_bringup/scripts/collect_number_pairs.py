#!/usr/bin/env python3
"""Save synchronized camera pairs for coarse alignment calibration."""

import argparse
import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Collector(Node):
    def __init__(self, output, distance, max_pairs):
        super().__init__('number_pair_collector')
        self.output = output
        self.distance = distance
        self.max_pairs = max_pairs
        self.count = 0
        self.bridge = CvBridge()
        main = Subscriber(self, Image, '/image_raw', qos_profile=qos_profile_sensor_data)
        number = Subscriber(self, Image, '/number_camera/image_raw',
                            qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([main, number], 20, 0.03)
        self.sync.registerCallback(self.save)

    def save(self, main, number):
        if self.count >= self.max_pairs:
            return
        name = f'{self.distance:g}m_{self.count:04d}'
        main_img = self.bridge.imgmsg_to_cv2(main, desired_encoding='bgr8')
        number_img = self.bridge.imgmsg_to_cv2(number, desired_encoding='bgr8')
        cv2.imwrite(str(self.output / f'{name}_main.png'), main_img)
        cv2.imwrite(str(self.output / f'{name}_number.png'), number_img)
        record = {
            'name': name, 'distance_m': self.distance,
            'main_stamp_ns': int(main.header.stamp.sec) * 10**9 + main.header.stamp.nanosec,
            'number_stamp_ns': int(number.header.stamp.sec) * 10**9 + number.header.stamp.nanosec,
            'main_size': [main.width, main.height],
            'number_size': [number.width, number.height],
        }
        with (self.output / 'pairs.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record) + '\n')
        self.count += 1
        self.get_logger().info(f'Saved {name} ({self.count}/{self.max_pairs})')
        if self.count >= self.max_pairs:
            raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distance-m', type=float, required=True)
    parser.add_argument('--count', type=int, default=20)
    args = parser.parse_args()
    if args.distance_m <= 0 or args.count <= 0:
        parser.error('distance and count must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Collector(args.output, args.distance_m, args.count)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
