#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque

GRAVITY = 9.81

class ImuIntegrator(Node):
    def __init__(self):
        super().__init__('imu_integrator_node')
        self.subscription = self.create_subscription(Imu, '/filtered_imu', self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = None

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.velocity_history = deque(maxlen=10)
        self.orientation = np.array([0, 0, 0, 1]) # Quaternion xyzw

        # Low-pass filter for acceleration
        self.accel_lp = np.zeros(3)
        self.alpha_accel_lp = 0.2  # 0 < alpha < 1; lower is smoother

        # Use norm history for improved bias estimation
        self.accel_norm_history = deque(maxlen=50)

        # For ZUPT
        self.stationary_threshold = 0.12  # m/s^2, tune as needed
        self.zupt_duration = 0.15         # seconds
        self.stationary_time = 0.0

        # IMU orientation initialized?
        self.orient_initialized = False

    def imu_callback(self, msg):
        # Time step
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = curr_time
            return
        dt = curr_time - self.prev_time
        if dt <= 0.0 or dt > 1.0:
            self.prev_time = curr_time
            return
        self.prev_time = curr_time

        # Orientation (quaternion to rotation matrix)
        q = msg.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        self.orientation = np.array([q.x, q.y, q.z, q.w])

        # Get acceleration in IMU frame
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Low-pass filter acceleration (reduce noise)
        self.accel_lp = self.alpha_accel_lp * accel + (1 - self.alpha_accel_lp) * self.accel_lp

        # Transform to world frame
        accel_world = rot.apply(self.accel_lp)

        # Subtract gravity
        # Gravity should be strictly vertical in world frame
        gravity_vec = np.array([0, 0, GRAVITY])
        accel_world_no_gravity = accel_world - gravity_vec

        # Bias estimation (use norm history for improved detection)
        self.accel_norm_history.append(np.linalg.norm(accel_world))
        if len(self.accel_norm_history) == self.accel_norm_history.maxlen:
            mean_norm = np.mean(self.accel_norm_history)
            if abs(mean_norm - GRAVITY) < 0.10:
                self.accel_bias = 0.99 * self.accel_bias + 0.01 * accel_world_no_gravity

        # Remove bias
        accel_corrected = accel_world_no_gravity - self.accel_bias

        # Only estimate planar odometry (2D)
        accel_corrected[2] = 0.0

        # ZUPT: detect stationary, zero velocity if detected
        if np.linalg.norm(accel_corrected[:2]) < self.stationary_threshold:
            self.stationary_time += dt
        else:
            self.stationary_time = 0.0

        # Integrate velocity
        self.velocity += accel_corrected * dt

        # ZUPT: forcibly zero velocity if stationary for long enough
        if self.stationary_time > self.zupt_duration:
            self.velocity[:2] = 0.0

        # Velocity smoothing
        self.velocity_history.append(self.velocity.copy())
        smoothed_velocity = np.mean(self.velocity_history, axis=0)

        # Integrate position
        self.position += smoothed_velocity * dt
        self.position[2] = 0.0
        self.velocity[2] = 0.0

        # Odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation = msg.orientation
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular = msg.angular_velocity
        self.odom_pub.publish(odom)

        # TF broadcast
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        t.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuIntegrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()