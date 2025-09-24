import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3 
import tf_transformations

import numpy as np

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('vhf')
        self.get_logger().info('Inicializando o nó! Olá...')
        
        self.subscriber_scan = self.create_subscription(LaserScan,'/scan',self.laser_callback, 10)
        self.subscriber_odom = self.create_subscription(Odometry,'/odom',self.odom_callback, 10)
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.dT = 0.1
        self.timer = self.create_timer(self.dT,self.timer_callback)
        
        self.n_fields = 45 # valor impar
        self.threshold   = 2.0
        self.dest_pos_x = 8.0
        self.dest_pos_y = 0.0
        self.robo_pos_x = None
        self.robo_pos_y = None
        self.robo_ang_z = None
        self.lasers     = None


    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

    def run(self):
        rclpy.spin(self)

    def laser_callback(self, msg):
        self.lasers = msg.ranges

    def odom_callback(self, msg):
        self.robo_pos_x = msg.pose.pose.position.x
        self.robo_pos_y = msg.pose.pose.position.y
    
        roll , pitch , yaw = tf_transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.robo_ang_z = yaw *180/np.pi

    def pub_vel(self,
                lx=0.0,ly=0.0,lz=0.0,
                ax=0.0,ay=0.0,az=0.0):
        msg = Twist(
            linear=Vector3(x=lx,y=ly,z=lz), 
            angular=Vector3(x=ax,y=ay,z=az)
        )
        self.publisher_vel.publish(msg)

    def timer_callback(self):
        if(self.lasers == None or self.robo_pos_x == None or self.robo_pos_x == None or self.robo_ang_z == None):
            return

        n_beams = len(self.lasers)
        n_beams_by_field = int(n_beams/self.n_fields)

        # continuar ...


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()   