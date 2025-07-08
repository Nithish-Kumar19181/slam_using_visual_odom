import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.subscription = self.create_subscription(
            String,
            '/serial_commands',
            self.listener_callback,
            10)
        
        # Replace with your correct serial port and baudrate
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.get_logger().info("Serial Bridge Initialized")

    def listener_callback(self, msg):
        data = msg.data + '\n'  # Add newline for ESP to read line
        self.ser.write(data.encode('utf-8'))
        self.get_logger().info(f"Sent to ESP: {data.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
