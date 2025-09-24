#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def timer_callback():
    global publisher, i
    msg = String()
    msg.data = f'Mensagem: {i}'
    i += 1
    node.get_logger().info(f'Publicando mensagem: {msg.data}')
    publisher.publish(msg)

def main(args=None):
  
    global node, publisher, i
    i = 0

    # Inicializando ROS
    rclpy.init(args=args)

    # Inicializando nó
    node = Node('topic')
    node.get_logger().info('Inicializando o nó!')
   
    publisher = node.create_publisher(String, 'topic', 10)
    node.create_timer(1,timer_callback)

    # Executando nó
    node.get_logger().info('Executando o nó!')
    rclpy.spin(node)

    # Finalizando nó
    node.get_logger().info('Finalizando o nó!')
    node.destroy_node()
  
    # Finalizando ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()




