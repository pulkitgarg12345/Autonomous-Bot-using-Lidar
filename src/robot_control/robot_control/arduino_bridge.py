import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Open Serial Port
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)  # wait for Arduino reset

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_cmd_vel,
            10
        )

    def callback_cmd_vel(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Decide command
        cmd = 'S'  # default Stop

        if linear > 0.1:
            cmd = 'F'
        elif linear < -0.1:
            cmd = 'B'
        elif angular > 0.1:
            cmd = 'L'
        elif angular < -0.1:
            cmd = 'R'

        # Send to Arduino
        self.arduino.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd}")

def main():
    rclpy.init()
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
