#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_forward)
        self.velocity = Twist()

    def move_forward(self):
        self.get_logger().info('Moving Forward...')
        self.velocity.linear.x = 0.5  # Velocidad lineal hacia adelante
        self.velocity.angular.z = 0.0  # Sin rotación
        self.publisher_.publish(self.velocity)
        time.sleep(2)  # Moverse por 2 segundos
        
        self.stop_rover()  # Detener el rover
        self.rotate_left()  # Girar a la izquierda
        
    def stop_rover(self):
        self.get_logger().info('Stopping Rover...')
        self.velocity.linear.x = 0.0  # Detener el movimiento
        self.velocity.angular.z = 0.0
        self.publisher_.publish(self.velocity)
        time.sleep(1)

    def rotate_left(self):
        self.get_logger().info('Rotating Left...')
        self.velocity.linear.x = 0.0  # Sin movimiento lineal
        self.velocity.angular.z = 0.5  # Rotar hacia la izquierda
        self.publisher_.publish(self.velocity)
        time.sleep(2)  # Rotar por 2 segundos
        
        self.stop_rover()  # Detener el rover

def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    
    rclpy.spin(rover_controller)
    
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
