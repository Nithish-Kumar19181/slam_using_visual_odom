#!/usr/bin/env python3
# Acts as the high-level controller for the robot.
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoystickCommandMapper(Node):
    def __init__(self):
        super().__init__('joystick_command_mapper')
        # Create a subscription to the joystick topic
        self.subscription = self.create_subscription(
            Joy,
            '/joystick/joy',
            self.joy_callback,
            10)
        # Create a publisher for serial commands to the ESP
        self.publisher = self.create_publisher(String, '/serial_commands', 10)
        
        # Initialize previous button states to detect rising edges (presses)
        # Assuming joystick has up to 11 buttons (index 0 to 10)
        self.prev_button_states = [0] * 11
        
        self.get_logger().info("Joystick Command Mapper Initialized")

    def joy_callback(self, msg: Joy):
        # Extract joystick axes data for the left stick (assumed index 0 = X, 1 = Y)
        x_axis = msg.axes[0]
        y_axis = msg.axes[1]

        # Extract button states
        lb_button = msg.buttons[4]    # L1 button
        rb_button = msg.buttons[5]    # R1 button
        y_button = msg.buttons[3]     # Y button
        a_button = msg.buttons[0]     # A button
        x_button = msg.buttons[2]     # X button (renamed to avoid conflict with x_axis variable)

        # Initialize angle variables; calculate only if needed for 'w' or 'r' commands
        angle = 0.0
        angle_from_straight = 0.0
        # Check if joystick is moved significantly for walk/rotate commands
        joystick_active = (abs(x_axis) > 0.1 or abs(y_axis) > 0.1)

        if joystick_active and (lb_button or x_button):
            # Calculate angle in radians from the joystick's X and Y axes
            angle = math.atan2(y_axis, x_axis)
            # Adjust angle relative to a 'straight ahead' direction (usually +Y axis is 0 or pi/2)
            angle_from_straight = angle - math.pi / 2
            # Normalize angle to be within [-pi, pi] for consistent representation
            if angle_from_straight > math.pi:
                angle_from_straight -= 2 * math.pi
            elif angle_from_straight < -math.pi:
                angle_from_straight += 2 * math.pi
            
            angle_from_straight = -angle_from_straight
            # if abs(angle_from_straight) < 1e-4:
            #     angle_from_straight = 0.0

        command = None # Initialize command to None

        # Determine command based on button presses (rising edge detection)
        # For 'w' (walk) and 'r' (rotate), also check if joystick is active
        
        # 'w' command (walk) triggered by LB button press if joystick active
        if lb_button and not self.prev_button_states[4] and joystick_active:
            command = f"w {angle_from_straight:.2f}"
        # 'r' command (rotate) triggered by X button press if joystick active
        elif x_button and not self.prev_button_states[2] and joystick_active:
            command = f"r {angle_from_straight:.2f}"
        # 's' command triggered by RB button press
        elif rb_button and not self.prev_button_states[5]:
            command = "s"
        # 'h' command triggered by Y button press
        elif y_button and not self.prev_button_states[3]:
            command = "h"
        # 'd' command triggered by A button press
        elif a_button and not self.prev_button_states[0]:
            command = "d"

        # If a command was generated, publish it
        if command:
            self.get_logger().info(f"Sending Command: {command}")
            msg_out = String()
            msg_out.data = command
            self.publisher.publish(msg_out)

        # Update previous button states for the next callback iteration
        # Convert msg.buttons (which is a tuple) to a list for mutable assignment
        self.prev_button_states = list(msg.buttons)

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    # Create an instance of the JoystickCommandMapper node
    node = JoystickCommandMapper()
    # Keep the node alive and processing callbacks
    rclpy.spin(node)
    # Clean up and destroy the node when it's shut down
    node.destroy_node()
    # Shut down the ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
