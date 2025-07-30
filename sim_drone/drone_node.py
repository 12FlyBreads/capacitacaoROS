import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Float32MultiArray
import time

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')

        # Estado inicial do drone
        self.state = 'IDLE'  # Estados: IDLE → TAKEOFF → MISSION → LAND

        # Posição inicial
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Potência dos motores (simples, 4 motores)
        self.motor_power = [0.0, 0.0, 0.0, 0.0]

        # Leitura do sensor LiDAR (simulada)
        self.lidar_height = 0.0

        # Distância simulada até obstáculo
        self.obstacle_distance = float('inf')  # começa sem obstáculos

        # Publicador de status de posição do drone
        self.position_publisher = self.create_publisher(
            Float32MultiArray, 
            'drone_position', 
            10
        )

        # Subscrição ao sensor LiDAR
        self.subscription = self.create_subscription(
            Float32,
            '/obstacle_distance',
            self.lidar_callback,
            10
        )

        # Timer para atualização (10Hz)
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Drone node iniciado. Aguardando decolagem...')
        time.sleep(1.0)
        self.state = 'TAKEOFF'

    def lidar_callback(self, msg):
        """Recebe distância do obstáculo."""
        self.obstacle_distance = msg.data

    def publish_status(self, status):
        msg = Float32MultiArray()
        msg.data = [self.x, self.y, self.z]
        self.position_publisher.publish(msg)

    def update(self):
        """Função chamada periodicamente para simular o comportamento do drone."""
        # Publica a posição atual do drone
        self.publish_status([self.x, self.y, self.z])
        """Após a publicação da posição, o sensor lidar é atualizado, publicando a altura atual do drone."""
        
        # Estado TAKEOFF → sobe até y = 5
        if self.state == 'TAKEOFF':
            if self.y < 5.0:
                self.y += 0.2
                self.motor_power = [0.6, 0.6, 0.6, 0.6]
            else:
                self.y = 5.0
                self.motor_power = [0.5, 0.5, 0.5, 0.5]
                self.state = 'MISSION'
                self.get_logger().info('Takeoff completo. Iniciando missão...')

        # Estado MISSION → avança no eixo z, desvia obstáculos
        elif self.state == 'MISSION':
            self.z += 0.1  # avança lentamente
            self.motor_power = [0.5, 0.5, 0.6, 0.6]  # motores ajustados para avanço (BR e BL mais potentes = pitch positivo)

            # Se obstáculo muito próximo, desvia em x
            if self.obstacle_distance < 2.0:
                self.x += 0.2  # desvia para a direita
                self.motor_power = [0.6, 0.5, 0.5, 0.6] # motores ajustados para desvio (FL e BL mais potentes = roll positivo)
            else:
                # retorna lentamente ao centro se sem obstáculos
                if self.x > 0.0:
                    self.x -= 0.1
                    self.motor_power = [0.5, 0.6, 0.6, 0.5]  # motores ajustados para correção (BR e BL mais potentes = pitch negativo)

            # Simulação: após certo z, termina missão e pousa
            if self.z > 10.0:
                self.state = 'LAND'
                self.get_logger().info('Missão concluída. Iniciando pouso...')

        # Estado LAND → desce até y = 0
        elif self.state == 'LAND':
            if self.y > 0.0:
                self.y -= 0.2
                self.motor_power = [0.4, 0.4, 0.4, 0.4]
            else:
                self.y = 0.0
                self.motor_power = [0.0, 0.0, 0.0, 0.0]
                self.state = 'IDLE'
                self.get_logger().info('Pouso concluído. Simulação finalizada.')

        # Exibir dados no terminal
        self.print_status()

    def print_status(self):
        """Mostra no terminal as informações atuais do drone."""
        self.get_logger().info(
            f"Estado: {self.state}\n"
            f"Posição: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}\n"
            f"Motores: {self.motor_power}\n"
            f"Distância até obstáculo: {self.obstacle_distance:.2f} m\n"
            "---------------------------"
        )


def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
