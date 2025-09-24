import math   #Biblioteca do Python para funções matemáticas 
import rclpy  #Biblioteca principal do ROS2 em Python 
from rclpy.node import Node   #Classe base para criar nós ROS2 em Python
from sensor_msgs.msg import LaserScan   #Mensagem do ROS2 para leituras de sensores (Sensor Laser do Turtlebot)
from nav_msgs.msg import Odometry       #Mensagem do ROS2 para informações de posição/orientação do robô
from geometry_msgs.msg import Twist     #Mensagem do ROS2 para comandos de velocidade linear e angular

# ====================== Função principal (main) ======================

def main(args=None):
    rclpy.init(args=args)      #Inicializa a comunicação com o ROS 2
    node = RobotControl()      #Cria uma instância do nó (classe RobotControl)

    try:
        rclpy.spin(node)       #Mantém o nó ativo, processando os callbacks
    except KeyboardInterrupt:  #Interrompe com Ctrl+C no terminal
        pass                   #Saída do Loop
    finally:
        node.pub_vel(0.0, 0.0) #Garante que o robô fique parado com as velocidades em 0
        node.destroy_node()    #Destruição do Nó
        rclpy.shutdown()       #Encerra a comunicação com o ROS2

# ====================== Classe principal ======================

class RobotControl(Node):                                      #Define a classe RobotControl herdando de Node (ROS2)
    def __init__(self):                                        #def para construção de classe
        super().__init__('robot_control')                      #Inicializa o nó com o nome "robot_control"
        self.get_logger().info('Inicializando o nó para desviar de obstáculos')  #Mensagem de inicialização no terminal

        # Assinaturas e publicadores
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)   #Assina o tópico /scan (dados do LiDAR)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)     #Assina o tópico /odom (dados da odometria)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)             #Publica comandos no tópico /cmd_vel

        # Estado do robô
        self.robot_x = 0.0                 #Posição X atual do robô
        self.robot_y = 0.0                 #Posição Y atual do robô
        self.robot_yaw = 0.0               #Ângulo de orientação (yaw) do robô
        self.odom_received = False         #Flag para indicar se já recebeu dados de odometria

        # Meta
        self.goal_x = 8.0                  #Coordenada X do objetivo
        self.goal_y = 0.0                  #Coordenada Y do objetivo
        self.goal_tolerance = 0.2          #Tolerância de distância para considerar objetivo atingido

        # Parâmetros de controle
        self.max_lin = 0.35                #Velocidade linear máxima (m/s)
        self.max_ang = 1.0                 #Velocidade angular máxima (rad/s)
        self.kp_lin = 0.5                  #Ganho proporcional para controle linear
        self.kp_ang = 1.2                  #Ganho proporcional para controle angular
        self.obstacle_dist = 1.5           #Distância mínima para detectar obstáculo à frente (m)
        self.side_dist_threshold = 0.4     #Distância mínima para considerar correção lateral (m)

        # Leituras do Laser
        self.dist_front = float('inf')     #Distância medida à frente (inicialmente infinita)
        self.dist_left = float('inf')      #Distância medida à esquerda (inicialmente infinita)
        self.dist_right = float('inf')     #Distância medida à direita (inicialmente infinita)
        self.laser_scan_msg = None         #Armazena a última mensagem recebida do LiDAR

    # ====================== Utilitários ======================

    def pub_vel(self, linear: float, angular: float): #Publica velocidades linear e angular no tópico /cmd_vel
        """Publica um comando de velocidade."""
        cmd = Twist()  #Cria uma mensagem do tipo Twist
        cmd.linear.x = linear #Define a velocidade linear no eixo X
        cmd.angular.z = angular #Define a velocidade angular em torno do eixo Z (giro)
        self.cmd_pub.publish(cmd)  #Publica a mensagem no tópico /cmd_vel

    def normalize_angle(self, angle: float) -> float: #Normaliza um ângulo
        """Normaliza ângulo para o intervalo [-pi, pi].""" #Faz o ajuste do ângulo para ficar entre -π e π
        return (angle + math.pi) % (2 * math.pi) - math.pi 

    def read_distance(self, scan: LaserScan, angle: float) -> float: #Lê a distância em um ângulo específico do LiDAR
        """Lê a distância no ângulo especificado (em rad) e valida o valor."""
        index = int(round((angle - scan.angle_min) / scan.angle_increment)) #Calcula o índice correspondente ao ângulo
        index = max(0, min(index, len(scan.ranges) - 1)) #Garante que o índice esteja dentro dos limites
        distance = scan.ranges[index]  #Obtém a distância no índice calculado

        # Valida se a leitura é infinita, inválida ou fora do alcance do sensor
        if math.isinf(distance) or math.isnan(distance) or distance < scan.range_min or distance > scan.range_max:
            return float('inf')   #Retorna infinito se a leitura for inválida
        return distance   #Caso contrário, retorna a distância válida

    # ====================== Callbacks ======================

    def laser_callback(self, msg: LaserScan):  #Recebe os dados do LiDAR
        """Callback do LaserScan."""
        self.laser_scan_msg = msg              #Armazena a última mensagem do sensor
        try:
            self.dist_front = self.read_distance(msg, 0.0)          #Lê distância à frente
            self.dist_left = self.read_distance(msg, math.pi / 2)   #Lê distância à esquerda
            self.dist_right = self.read_distance(msg, -math.pi / 2) #Lê distância à direita
        except Exception as e:                                      #Se ocorrer erro na leitura
            self.get_logger().warn(f'Erro ao processar LaserScan: {e}')

    def odom_callback(self, msg: Odometry):   #Recebe os dados de posição e orientação
        """Callback da odometria."""
        p = msg.pose.pose.position            #Posição do robô
        o = msg.pose.pose.orientation         #Orientação (quaternion)

        self.robot_x = p.x                    #Atualiza X
        self.robot_y = p.y                    #Atualiza Y

        # Converte quaternion para yaw
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y ** 2 + o.z ** 2)
        self.robot_yaw = math.atan2(siny, cosy)

        self.odom_received = True             #Marca que odometria foi recebida
        self.process()                        #Chama o controle principal

    # ====================== Controle principal ======================

    def process(self):  #Lógica principal de movimento
        if not self.odom_received or self.laser_scan_msg is None:  #Se não recebeu odometria ou leitura do laser
            self.pub_vel(0.0, 0.0)           #Mantém o robô parado
            return

        # Calcula a distância até a meta
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_to_goal = math.hypot(dx, dy)    #Distância euclidiana

        # Verifica se atingiu a meta
        if dist_to_goal <= self.goal_tolerance:
            self.pub_vel(0.0, 0.0)           #Para o robô
            self.get_logger().info('Objetivo realizado com sucesso!')  #Mensagem no terminal
            return

        # Analisa os valores do LiDAR à frente (±30°)
        front_ranges = self.laser_scan_msg.ranges[0:30] + self.laser_scan_msg.ranges[-30:]
        min_front_dist = min([d for d in front_ranges if math.isfinite(d)] or [float('inf')])

        # Se obstáculo for detectado à frente
        if min_front_dist < self.obstacle_dist:
            self.get_logger().info(f'Obstáculo detectado à frente ({min_front_dist:.2f}m). Realizar a manobra Desviar.')

            if math.isinf(self.dist_left) or math.isinf(self.dist_right):  #Se não detecta nada nas laterais
                self.get_logger().warn('Nenhum obstáculo detectado nas laterais. Realizando giro de segurança para a direita.')
                self.pub_vel(0.0, -0.7)  #Giro de emergência
                return

            # Decide para qual lado girar (esquerda ou direita)
            if self.dist_left > self.dist_right:
                self.pub_vel(0.0, 0.5)  #Gira para a esquerda
            else:
                self.pub_vel(0.0, -0.5) #Gira para a direita
            return

        # Correção se estiver muito perto da parede esquerda
        if self.dist_left < self.side_dist_threshold:
            self.get_logger().debug('Perigo! Muito perto da parede esquerda, virando para a direita.')
            self.pub_vel(0.1, -0.3)
            return

        # Correção se estiver muito perto da parede direita
        if self.dist_right < self.side_dist_threshold:
            self.get_logger().debug('Perigo! Muito perto da parede direita, virando para a esquerda.')
            self.pub_vel(0.1, 0.3)
            return

        # Caso não haja obstáculos → navegação normal até o alvo
        desired_yaw = math.atan2(dy, dx)                   #Ângulo desejado até a meta
        yaw_error = self.normalize_angle(desired_yaw - self.robot_yaw)  #Erro angular entre orientação e meta

        ang_cmd = self.kp_ang * yaw_error                  #Controle proporcional angular
        lin_cmd = self.kp_lin * dist_to_goal * (1 - abs(yaw_error) / math.pi) #Controle proporcional linear

        # Limita velocidades para não ultrapassar os máximos
        lin_cmd = max(0.0, min(self.max_lin, lin_cmd))
        ang_cmd = max(-self.max_ang, min(self.max_ang, ang_cmd))

        # Publica comando de movimento
        self.pub_vel(lin_cmd, ang_cmd)
        self.get_logger().debug(
            f'Navegando: dist={dist_to_goal:.2f}, yaw_err={yaw_error:.2f}, lin={lin_cmd:.2f}, ang={ang_cmd:.2f}'
        )

# ====================== Execução direta ======================

if __name__ == '__main__':  #Se o arquivo for executado diretamente
    main()                  #Chama a função principal
