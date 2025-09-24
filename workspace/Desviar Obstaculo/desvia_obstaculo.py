import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DesviaObstaculo(Node):
    
    def __init__(self):
        super().__init__('desvia_obstaculo')
        self.get_logger().info('Inicializando desvia_obstaculo...')

        # Assinaturas e publicadores
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Estado do robô
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Meta
        self.goal_x = 8.0
        self.goal_y = 0.0
        self.goal_tolerance = 0.2

        # Parâmetros de controle
        self.max_lin = 0.35
        self.max_ang = 1.0
        self.kp_lin = 0.5
        self.kp_ang = 1.2
        self.obstacle_dist = 1.0  # Distância para detecção de obstáculo
        self.side_dist_threshold = 0.5 # Distância para paredes laterais

        # Leituras do Laser
        self.dist_front = float('inf')
        self.dist_left = float('inf')
        self.dist_right = float('inf')
        self.laser_scan_msg = None

    # ====================== Callbacks ======================

    def odom_callback(self, msg: Odometry):
        """Callback da odometria: atualiza posição e orientação do robô."""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        self.robot_x = p.x
        self.robot_y = p.y

        # Conversão de quaternion para yaw
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * 2 + o.z * 2)
        self.robot_yaw = math.atan2(siny, cosy)

        self.odom_received = True
        self.process()

    def laser_callback(self, msg: LaserScan):
        """Callback do LaserScan: armazena a mensagem para ser processada em 'process'."""
        self.laser_scan_msg = msg
        try:
            self.dist_front = self.read_distance(msg, 0.0)
            self.dist_left = self.read_distance(msg, math.pi / 2)
            self.dist_right = self.read_distance(msg, -math.pi / 2)
        except Exception as e:
            self.get_logger().warn(f'Erro ao processar LaserScan: {e}')

    # ====================== Utilitários ======================

    def read_distance(self, scan: LaserScan, angle: float) -> float:
        """Lê a distância no ângulo especificado (em rad) e valida o valor."""
        index = int(round((angle - scan.angle_min) / scan.angle_increment))
        index = max(0, min(index, len(scan.ranges) - 1))
        distance = scan.ranges[index]

        # Verifica se o valor é válido e está dentro dos limites do sensor
        if math.isinf(distance) or math.isnan(distance) or distance < scan.range_min or distance > scan.range_max:
            return float('inf')
        return distance

    def normalize_angle(self, angle: float) -> float:
        """Normaliza ângulo para o intervalo [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def pub_vel(self, linear: float, angular: float):
        """Publica um comando de velocidade."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    # ====================== Controle principal ======================

    def process(self):
        if not self.odom_received or self.laser_scan_msg is None:
            self.pub_vel(0.0, 0.0)
            return

        # 1. Checar se a meta foi alcançada
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal <= self.goal_tolerance:
            self.pub_vel(0.0, 0.0)
            self.get_logger().info('Meta alcançada!')
            return

        # 2. Lógica de desvio de obstáculo (prioridade máxima)
        # Verifica a distância em um ângulo mais amplo para evitar colisões
        front_ranges = self.laser_scan_msg.ranges[0:30] + self.laser_scan_msg.ranges[-30:]
        min_front_dist = min([d for d in front_ranges if math.isfinite(d)] or [float('inf')])
        
        if min_front_dist < self.obstacle_dist:
            self.get_logger().info(f'Obstáculo detectado à frente ({min_front_dist:.2f}m). Desviando.')

            # *Nova lógica de desvio de emergência:*
            # Se a leitura lateral é infinita, significa que o sensor não vê nada (está em um beco sem saída ou no meio de um objeto)
            # Neste caso, o robô faz um giro fixo para a direita para tentar "enxergar" o lado do obstáculo.
            if math.isinf(self.dist_left) or math.isinf(self.dist_right):
                self.get_logger().warn('Leitura lateral infinita. Realizando giro de emergência para a direita.')
                self.pub_vel(0.0, -0.7)
                return

            # Lógica de desvio padrão: Gira para onde houver mais espaço
            if self.dist_left > self.dist_right:
                self.pub_vel(0.0, 0.5)  # Gira para a esquerda
            else:
                self.pub_vel(0.0, -0.5) # Gira para a direita
            return

        # 3. Lógica de correções laterais
        if self.dist_left < self.side_dist_threshold:
            self.get_logger().debug('Muito perto da parede esquerda. Corrigindo para a direita.')
            self.pub_vel(0.1, -0.3)
            return

        if self.dist_right < self.side_dist_threshold:
            self.get_logger().debug('Muito perto da parede direita. Corrigindo para a esquerda.')
            self.pub_vel(0.1, 0.3)
            return

        # 4. Lógica de navegação para o alvo (baixa prioridade)
        # Se não há obstáculos, navega normalmente
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - self.robot_yaw)

        ang_cmd = self.kp_ang * yaw_error
        # Adiciona uma lógica para reduzir a velocidade linear se o erro angular for grande
        lin_cmd = self.kp_lin * dist_to_goal * (1 - abs(yaw_error) / math.pi)

        # Garante que a velocidade linear não seja negativa
        lin_cmd = max(0.0, min(self.max_lin, lin_cmd))
        ang_cmd = max(-self.max_ang, min(self.max_ang, ang_cmd))

        self.pub_vel(lin_cmd, ang_cmd)
        self.get_logger().debug(f'Navegando: dist={dist_to_goal:.2f}, yaw_err={yaw_error:.2f}, lin={lin_cmd:.2f}, ang={ang_cmd:.2f}')


# ====================== Execução ======================

def main(args=None):
    rclpy.init(args=args)
    node = DesviaObstaculo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()