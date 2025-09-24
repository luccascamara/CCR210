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

        self.distancia_objetivo = 1
        self.distancia_direita = numpy.array(self.laser[80:100]).mean()
        self.distancia_frente = 0.0

        self.p_gain = 0.1 # valores iniciais
        self.i_gain = 0.0 # que precisam
        self.d_gain = 0.0 # ser ajustados

        self.error = self.distancia_objetivo - self.distancia_direita
        self.dif_erro = 0.0
        self.integral = 0.0
        self.old_error = 0.0

        power = self.p_gain*self.error
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = power
        self.pub_cmd_vel.publish(cmd)

    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

    def run(self):
        rclpy.spin(self)

    def laser_callback(self, msg):
 
        self.distancia_direita = msg.ranges[90]
        self.distancia_frente = msg.ranges[0]

        self.get_logger().debug(f'distancia_direita: {self.distancia_direita}')
        self.get_logger().debug(f'distancia_frente: {self.distancia_frente}')

        self.process()

    def pub_vel(self,
                lx=0.0,ly=0.0,lz=0.0,
                ax=0.0,ay=0.0,az=0.0):
        msg = Twist(
            linear=Vector3(x=lx,y=ly,z=lz), 
            angular=Vector3(x=ax,y=ay,z=az)
        )
        self.publisher_vel.publish(msg)


    def process(self):    
        # implementar pid
        self.pub_vel(lx=0.1, az=0.0) # valores iniciais que precisam ser ajustados
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