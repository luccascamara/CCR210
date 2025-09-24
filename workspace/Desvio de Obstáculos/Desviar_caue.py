import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3 
from nav_msgs.msg import Odometry

import numpy
import tf_transformations

#--------------------------------------------------------------

class RobotControlCasa(Node):

    def __init__(self):
        super().__init__('robot_control_casa')
        self.get_logger().info('Inicializando o nó! Olá...')
        
        self.subscriber_scan = self.create_subscription(LaserScan,'/scan',self.laser_callback, 10)
        self.subscriber_odom = self.create_subscription(Odometry,'/odom',self.odom_callback, 10)
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.distancia_objetivo = 1.0
        self.distancia_direita = 0.0
        self.distancia_frente = 0.0

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0

        self.goal_x = 8.0
        self.goal_y = 0.0

        self.laser_data = None

        self.p_gain = 0.05
        self.i_gain = 0.001
        self.d_gain = 0.01

        self.error = 0.0
        self.integral = 0.0
        self.old_error = 0.0

    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

    def run(self):
        rclpy.spin(self)

# --------------------- Callbacks ---------------------

    def laser_callback(self, msg):
        self.laser_data = msg
        # Mantém compatibilidade com controle PID se quiser usar
        direita_validos = [v for v in msg.ranges[80:100] if numpy.isfinite(v)]
        self.distancia_direita = numpy.mean(direita_validos) if direita_validos else self.distancia_objetivo
        self.distancia_frente = msg.ranges[0] if numpy.isfinite(msg.ranges[0]) else self.distancia_objetivo

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        (_, _, self.yaw) = tf_transformations.euler_from_quaternion([self.x, self.y, self.z, self.w]) 

        self.process()

#--------------------- PubVel ---------------------

    def pub_vel(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        msg = Twist(linear=Vector3(x=lx, y=ly, z=lz), 
                    angular=Vector3(x=ax, y=ay, z=az)
        )
        self.publisher_vel.publish(msg)

# --------------------- Process ---------------------

    def process(self):    
        # VFH simplificado
        if self.laser_data is None:
            return

        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        goal_angle = numpy.arctan2(dy, dx)
        rel_goal_angle = self.normalize_angle(goal_angle - self.yaw)

        ranges = numpy.array(self.laser_data.ranges)
        angles = numpy.linspace(self.laser_data.angle_min, self.laser_data.angle_max, len(ranges))
        threshold = 0.8  # metros
        free_angles = angles[ranges > threshold]

        if len(free_angles) > 0:
            best_angle = free_angles[numpy.argmin(numpy.abs(free_angles - rel_goal_angle))]
        else:
            best_angle = 0.0  # Se tudo bloqueado, segue em frente devagar

        dist_to_goal = numpy.hypot(dx, dy)
        linear_speed = min(0.5, dist_to_goal)
        angular_speed = 1.0 * best_angle

        # Para se estiver próximo do objetivo
        if dist_to_goal < 0.2:
            linear_speed = 0.0
            angular_speed = 0.0
            self.get_logger().info('Objetivo alcançado!')

        self.pub_vel(lx=linear_speed, az=angular_speed)
        self.get_logger().info(f'|| X: {self.pose_x:.2f} || Y: {self.pose_y:.2f} || Yaw: {self.yaw:.2f} || Distância ao objetivo: {dist_to_goal:.2f}')

    def normalize_angle(self, angle):
        return numpy.arctan2(numpy.sin(angle), numpy.cos(angle))

# --------------------- Main ---------------------

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlCasa()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()