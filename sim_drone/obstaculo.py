import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import math

class Obstaculos(Node):
    def __init__(self):
        super().__init__('obstaculos_node')

        # Objeto fixo no espaço
        self.obstaculo_x = 0.0
        self.obstaculo_y_min = 0.0
        self.obstaculo_y_max = 10.0
        self.obstaculo_z = 5.0

        # Publisher da distância ao obstáculo
        self.publisher_ = self.create_publisher(Float32, '/obstacle_distance', 10)

        # Subscriber da posição do drone
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'drone_position',
            self.position_callback,
            10
        )

    def position_callback(self, msg):
        # Atualiza a posição do drone
        self.drone_position = msg.data
        self.publish_distance()

    def publish_distance(self):
        x, _, z = self.drone_position 
        print(f"[Obstáculo] Posição do drone: x={x}, z={z}")
        distancia = math.sqrt((x - self.obstaculo_x)**2 + (z - self.obstaculo_z)**2)

        msg = Float32()
        msg.data = distancia
        self.publisher_.publish(msg)
        print(f"[Obstáculo] Distância publicada: {distancia}")

def main(args=None):
    rclpy.init(args=args)
    node = Obstaculos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
