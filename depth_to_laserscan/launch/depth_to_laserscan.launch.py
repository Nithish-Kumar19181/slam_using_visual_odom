#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthToLaserScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_laserscan')

        # Camera intrinsics (modify if needed)
        self.fx = 617.0
        self.fy = 617.0
        self.cx = 320.0
        self.cy = 240.0

        # LaserScan settings
        self.angle_min = -1.57
        self.angle_max = 1.57
        self.range_min = 0.3
        self.range_max = 5.0

        # Topics
        # MODIFIED: Use the correct depth image topic published by your RealSense node
        self.depth_topic = '/camera/depth/image_rect_raw'  # <-- CHANGED LINE
        self.scan_topic = '/scan'                          # <-- CHANGED LINE

        # ROS interfaces
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.pub = self.create_publisher(LaserScan, self.scan_topic, 10)

    def depth_callback(self, msg):
        self.get_logger().info('Received a depth image!')  # <-- ADDED LINE for debug

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if len(cv_image.shape) != 2:
            self.get_logger().error('Depth image is not 2D!')
            return

        height, width = cv_image.shape
        center_row_idx = height // 2
        band_height = 1  # Use more if needed

        rows = np.arange(center_row_idx - band_height // 2,
                         center_row_idx + band_height // 2 + 1)

        depths = cv_image[rows, :].astype(np.float32) / 1000.0  # in meters
        depth_row = np.mean(depths, axis=0)

        # Clean bad depth data
        depth_row = np.nan_to_num(depth_row, nan=0.0, posinf=0.0, neginf=0.0)
        depth_row[depth_row < self.range_min] = 0.0
        depth_row[depth_row > self.range_max] = 0.0

        u_coords = np.arange(width)
        X = (u_coords - self.cx) * depth_row / self.fx
        Z = depth_row

        angles = np.arctan2(X, Z)

        valid_angle = np.logical_and(angles >= self.angle_min, angles <= self.angle_max)
        valid_range = np.logical_and(depth_row > self.range_min, depth_row < self.range_max)
        valid = np.logical_and(valid_angle, valid_range)

        ranges = np.full(width, float('inf'))
        ranges[valid] = np.sqrt(X[valid]**2 + Z[valid]**2)

        # Debug print
        self.get_logger().info(f'Valid points: {np.sum(valid)} / {width}')

        scan = LaserScan()
        scan.header = msg.header
        # MODIFIED: Set frame id to match depth image frame or TF
        scan.header.frame_id = "camera_link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / float(width)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()

        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()