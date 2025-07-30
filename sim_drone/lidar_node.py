import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Float32MultiArray

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_status_logger')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'drone_position',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        x, y, z = msg.data
        self.get_logger().info(f'[LIDAR STATUS]: {y:.2f} m\n')


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
