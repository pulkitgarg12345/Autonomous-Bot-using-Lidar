import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class FixedMapPublisher(Node):
    def __init__(self):
        super().__init__('fixed_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.width = 20
        self.height = 20
        self.resolution = 0.05

        # Create a fixed map: 0=free, 100=obstacle
        # Example: border is obstacle, inside is free
        self.map_data = []
        for y in range(self.height):
            for x in range(self.width):
                if x == 0 or x == self.width-1 or y == 0 or y == self.height-1:
                    self.map_data.append(100)  # border obstacle
                elif (x == 5 and y > 5 and y < 15) or (y == 10 and x > 10 and x < 18):
                    self.map_data.append(100)  # some inner obstacles
                else:
                    self.map_data.append(0)    # free space

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.map_data
        self.publisher_.publish(msg)
        self.get_logger().info('Published fixed map')

def main(args=None):
    rclpy.init(args=args)
    node = FixedMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
