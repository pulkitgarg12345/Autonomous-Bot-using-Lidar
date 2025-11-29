#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf_transformations
from tf2_ros import TransformBroadcaster
import math

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')

        # --- parameters ---
        self.declare_parameter('wheel_radius', 0.065)   # meters
        self.declare_parameter('wheel_base', 0.17)      # meters
        self.declare_parameter('ppr', 294)              # pulses per revolution
        self.declare_parameter('gear_ratio', 1.0)       # if gearbox present
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ppr = float(self.get_parameter('ppr').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # --- pose and last counts ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = self.get_clock().now()

        # --- publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- subscribe to encoder counts ---
        self.sub = self.create_subscription(String, '/encoder_data', self.encoder_callback, 50)

        self.get_logger().info(f"EncoderOdom started: wheel_r={self.wheel_radius} m, base={self.wheel_base} m, ppr={self.ppr}")

    # --- convert encoder counts to meters ---
    def counts_to_distance(self, counts):
        revolutions = counts / self.ppr
        wheel_revs = revolutions / self.gear_ratio
        return wheel_revs * (2.0 * math.pi * self.wheel_radius)

    def encoder_callback(self, msg: String):
        now = self.get_clock().now()
        try:
            left_s, right_s = msg.data.strip().split(',')
            left = int(left_s)
            right = int(right_s)
        except Exception as e:
            self.get_logger().warning(f"Bad encoder message '{msg.data}': {e}")
            return

        if self.last_left is None or self.last_right is None:
            self.last_left = left
            self.last_right = right
            self.last_time = now
            return

        # --- compute deltas ---
        delta_left = left - self.last_left
        delta_right = right - self.last_right

        # wrap-around protection
        if abs(delta_left) > 1e6: delta_left = 0
        if abs(delta_right) > 1e6: delta_right = 0

        # --- update last counts ---
        self.last_left = left
        self.last_right = right

        # --- delta time ---
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0: dt = 1e-3
        self.last_time = now

        # --- distances ---
        dist_left = self.counts_to_distance(delta_left)
        dist_right = self.counts_to_distance(delta_right)

        # --- differential drive kinematics ---
        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        # --- update pose ---
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta = self._normalize_angle(self.theta + delta_theta)

        # --- compute velocities ---
        vx = delta_s / dt
        vth = delta_theta / dt

        # --- debug ---
        self.get_logger().info(f"delta_s={delta_s:.4f} m, delta_theta={delta_theta:.4f} rad, dt={dt:.4f} s, vx={vx:.4f} m/s, vth={vth:.4f} rad/s")

        # --- create odom message ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # --- required 36-element covariance ---
        odom.pose.covariance = [0.0001 if i%7==0 else 0.0 for i in range(36)]
        odom.twist.covariance = [0.0001 if i%7==0 else 0.0 for i in range(36)]

        # --- linear & angular velocities ---
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        # --- publish ---
        self.odom_pub.publish(odom)

        # --- broadcast TF ---
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)

    def _normalize_angle(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
