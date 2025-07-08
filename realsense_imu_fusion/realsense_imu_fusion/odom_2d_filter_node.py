#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
from rclpy.parameter import Parameter

class Odom2DFilterNode(Node):
    def __init__(self):
        super().__init__('odom_2d_filter_node')
        
        # Parameters with type hints
        self.declare_parameter('input_topic', '/odom_cam')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        
        # Get parameters with proper type handling
        input_topic: str = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic: str = self.get_parameter('output_topic').get_parameter_value().string_value
        self.output_frame: str = self.get_parameter('output_frame').get_parameter_value().string_value
        self.child_frame: str = self.get_parameter('child_frame').get_parameter_value().string_value
        
        # Subscriber and Publisher
        self.sub_odom = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            10
        )
        
        self.pub_odom = self.create_publisher(
            Odometry,
            output_topic,
            10
        )
        
        self.br = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info(f"Converting 3D odometry from {input_topic} to 2D odometry on {output_topic}")

    def odom_callback(self, msg: Odometry) -> None:
        # Process position (ignore Z)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Process orientation (extract yaw only)
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        q_2d = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        
        # Create and publish 2D odometry message
        odom_2d = Odometry()
        odom_2d.header = msg.header
        odom_2d.header.frame_id = self.output_frame
        odom_2d.child_frame_id = self.child_frame
        
        # Position (2D)
        odom_2d.pose.pose.position.x = x
        odom_2d.pose.pose.position.y = y
        odom_2d.pose.pose.position.z = 0.0
        
        # Orientation (yaw only)
        odom_2d.pose.pose.orientation.x = q_2d[0]
        odom_2d.pose.pose.orientation.y = q_2d[1]
        odom_2d.pose.pose.orientation.z = q_2d[2]
        odom_2d.pose.pose.orientation.w = q_2d[3]
        
        # Velocity (2D)
        odom_2d.twist.twist.linear.x = msg.twist.twist.linear.x
        odom_2d.twist.twist.linear.y = msg.twist.twist.linear.y
        odom_2d.twist.twist.linear.z = 0.0
        odom_2d.twist.twist.angular.x = 0.0
        odom_2d.twist.twist.angular.y = 0.0
        odom_2d.twist.twist.angular.z = msg.twist.twist.angular.z
        
        # Publish the 2D odometry
        self.pub_odom.publish(odom_2d)
        
        # Also publish as TF for compatibility
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.output_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q_2d[0]
        t.transform.rotation.y = q_2d[1]
        t.transform.rotation.z = q_2d[2]
        t.transform.rotation.w = q_2d[3]
        
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Odom2DFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()