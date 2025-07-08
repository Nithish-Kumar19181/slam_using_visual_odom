#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

class DepthToLaserScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_laserscan_node')

        # --- Parameters for Virtual LaserScan (mimicking RPLIDAR C1, adjusted for D455 range) ---
        # New parameter for desired publishing frequency
        self.declare_parameter('publish_frequency', 1.0) # Hz (e.g., 10.0 for 10Hz)
        
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 5.0) # Realistic maximum reliable range for D455
        self.declare_parameter('scan_height', 10)
        self.declare_parameter('output_frame_id', 'base_link') # Changed from camera_link to base_link based on latest prompt context
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.scan_time = 1.0 / self.publish_frequency # Calculate scan_time based on publish_frequency
        
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.scan_height = self.get_parameter('scan_height').get_parameter_value().integer_value
        self.output_frame_id = self.get_parameter('output_frame_id').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None
        self.camera_info_received = False

        self.angle_min = 0.0                
        self.angle_max = 2.0 * math.pi      
        self.angle_increment = math.radians(0.9)
        self.num_beams = int(round((self.angle_max - self.angle_min) / self.angle_increment))

        self.bridge = CvBridge()
        self.latest_depth_msg = None
        # --- NEW: Store timestamp of the last processed message ---
        self.last_processed_stamp_sec = 0
        self.last_processed_stamp_nanosec = 0

        # --- Subscriptions ---
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            image_qos_profile
        )

        # --- Publisher ---
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        # --- Timer for publishing LaserScan ---
        # Timer period is now 1.0 / self.publish_frequency
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_scan)

        self.get_logger().info(f"Depth to LaserScan node started.")
        self.get_logger().info(f"Subscribing to Depth: {self.depth_topic} and CameraInfo: {self.camera_info_topic}")
        self.get_logger().info(f"Publishing to Scan: {self.scan_topic} in frame: {self.output_frame_id} at {self.publish_frequency} Hz.")
        self.get_logger().info(f"Virtual lidar mimicking RPLIDAR C1 with {self.num_beams} beams.")
        self.get_logger().info(f"Range: {self.range_min:.2f} to {self.range_max:.2f} m, Angle Increment: {math.degrees(self.angle_increment):.4f} deg")


    def camera_info_callback(self, msg: CameraInfo):
        """Callback for CameraInfo messages to get intrinsics."""
        if not self.camera_info_received:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            self.camera_info_received = True
            self.get_logger().info(f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
            
            if self.camera_info_sub is not None: 
                self.destroy_subscription(self.camera_info_sub)
                self.camera_info_sub = None 

    def depth_callback(self, msg: Image):
        """Callback for depth image messages. Stores the latest message."""
        self.latest_depth_msg = msg

    def publish_scan(self):
        """Processes the latest depth image and publishes a LaserScan."""
        if not self.camera_info_received or self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return

        if self.latest_depth_msg is None:
            return

        msg = self.latest_depth_msg 

        # --- NEW: Check if this message has already been processed ---
        current_stamp_sec = msg.header.stamp.sec
        current_stamp_nanosec = msg.header.stamp.nanosec
        
        if (current_stamp_sec == self.last_processed_stamp_sec and
            current_stamp_nanosec == self.last_processed_stamp_nanosec):
            # No new message since last publish, return without re-processing
            return
        
        # Update last processed stamp
        self.last_processed_stamp_sec = current_stamp_sec
        self.last_processed_stamp_nanosec = current_stamp_nanosec
        # --- END NEW ---

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if len(cv_image.shape) != 2:
            self.get_logger().error(f'Depth image is not 2D! Shape: {cv_image.shape}')
            return

        height, width = cv_image.shape
        
        ranges_output = np.full(self.num_beams, float('inf'), dtype=np.float32)

        center_row_idx = int(self.cy) 
        rows_to_process = np.arange(
            max(0, int(center_row_idx - self.scan_height / 2)),
            min(height, int(center_row_idx + self.scan_height / 2 + 1))
        )
        if not rows_to_process.size > 0:
            self.get_logger().warn(f"No valid rows to process for scan_height {self.scan_height} and cy {self.cy}. Image height: {height}. Consider adjusting scan_height or camera intrinsics.")
            return

        for v_idx in rows_to_process:
            for u_idx in range(width):
                depth_val_raw = cv_image[v_idx, u_idx]
                
                if depth_val_raw == 0: 
                    continue
                
                Z_camera = float(depth_val_raw) * 0.001 

                if Z_camera < self.range_min or Z_camera > self.range_max:
                    continue

                X_camera = (u_idx - self.cx) * Z_camera / self.fx 
                
                horizontal_range = math.sqrt(X_camera**2 + Z_camera**2)
                
                angle_radians_raw = math.atan2(X_camera, Z_camera) 

                angle_normalized = (angle_radians_raw + 2.0 * math.pi) % (2.0 * math.pi)
                
                if horizontal_range < self.range_min or horizontal_range > self.range_max:
                    continue
                
                if self.angle_min <= angle_normalized < self.angle_max:
                    beam_idx = int(round((angle_normalized - self.angle_min) / self.angle_increment))
                    
                    if 0 <= beam_idx < self.num_beams:
                        if horizontal_range < ranges_output[beam_idx]:
                            ranges_output[beam_idx] = horizontal_range

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp 
        scan.header.frame_id = self.output_frame_id 

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0             
        scan.scan_time = self.scan_time       # Now correctly derived from publish_frequency
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges_output.tolist()  
        scan.intensities = []                 

        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()