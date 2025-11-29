#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # ---- SERIAL PORT (update if needed) ----
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)

        # ---- ROS PUB/SUB ----
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub_enc = self.create_publisher(String, 'encoder_data', 10)

        # ---- Start serial reading thread ----
        thread = threading.Thread(target=self.read_serial)
        thread.daemon = True
        thread.start()

        self.get_logger().info("ROS2 <-> Arduino Motor Bridge Started")

    # ---- Convert /cmd_vel to single char commands ----
    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0.1:
            cmd = 'F'
        elif linear < -0.1:
            cmd = 'B'
        elif angular > 0.2:
            cmd = 'L'
        elif angular < -0.2:
            cmd = 'R'
        else:
            cmd = 'S'

        # Send command to Arduino
        self.ser.write(cmd.encode())

    # ---- Read Encoders from Arduino ----
    def read_serial(self):
        while True:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    msg = String()
                    msg.data = line   # Keep as string e.g. "123,456"
                    self.pub_enc.publish(msg)
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
