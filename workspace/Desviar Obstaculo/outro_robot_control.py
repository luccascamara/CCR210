import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3 

import numpy

class RobotControl(Node):

    def __init__(self):
        super().__init__('robot_control')
        self.get_logger().info('Inicializando o nó! Olá...')
        
        self.subscriber_scan = self.create_subscription(LaserScan,'/scan',self.laser_callback, 10)
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.distancia_objetivo = 1.0
        self.distancia_direita = 0.0
        self.distancia_frente = 0.0

        self.p_gain = 2.00 # valores iniciais
        self.i_gain = 0.01 # que precisam
        self.d_gain = 1.00 # ser ajustados

        self.error = 0.0
        self.dif_erro = 0.0
        self.integral = 0.0
        self.old_error = 0.0

    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

    def run(self):
        rclpy.spin(self)

    def laser_callback(self, msg):

        self.distancia_direita = min(msg.ranges[270:300])
        self.distancia_frente = msg.ranges[0]

        self.get_logger().debug(f'distancia_direita: {self.distancia_direita}')
        self.get_logger().debug(f'distancia_frente: {self.distancia_frente}')

        self.process()

    def vel(self,
                lx=0.0,ly=0.0,lz=0.0,
                ax=0.0,ay=0.0,az=0.0):
        msg = Twist(
            linear=Vector3(x=lx,y=ly,z=lz), 
            angular=Vector3(x=ax,y=ay,z=az)
        )
        self.publisher_vel.publish(msg)


    def process(self):    
        # implementar pid

        if (self.distancia_frente < 0.5):
            self.pub_vel(lx=0.0, az=1.5)
        else:
            self.error = self.distancia_objetivo - self.distancia_direita
            self.integral = self.integral + self.error
            self.dif_erro = self.error - self.old_error
            self.old_error = self.error

            power = self.p_gain*self.error + self.i_gain*self.integral + self.d_gain*self.dif_erro
            self.get_logger().info(f'power: {power}')

            self.vel(lx=0.2, az=power) # valores iniciais que precisam ser ajustados
        pass



def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()   




