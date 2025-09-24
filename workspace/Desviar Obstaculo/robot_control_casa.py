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
        self.distancia_direita = numpy.array(msg.ranges[80:100]).mean()
        self.distancia_frente = msg.ranges[0]

        self.error = self.distancia_objetivo - self.distancia_direita
        self.integral += self.error
        self.dif_erro = self.error - self.old_error
        self.old_error = self.error

        self.power = (self.p_gain * self.error) + (self.i_gain * self.integral) + (self.d_gain * self.dif_erro)
        
        self.process(self.power)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        self.yaw = tf_transformations.euler_from_quaternion([self.x, self.y, self.z, self.w]) 

        self.process(self.power)

#--------------------- PubVel ---------------------

    def pub_vel(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        msg = Twist(linear=Vector3(x=lx, y=ly, z=lz), 
                    angular=Vector3(x=ax, y=ay, z=az)
        )
        self.publisher_vel.publish(msg)

# --------------------- Process ---------------------

    def process(self, power):    

        self.pub_vel(lx=0.1, az=power)
        self.get_logger().info(f'|| Direita: {self.distancia_direita}|| Frente: {self.distancia_frente}|| Power: {power}')

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