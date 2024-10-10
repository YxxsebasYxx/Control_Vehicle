#!/usr/bin/env python3

from pymavlink import mavutil
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DronePublisher(Node):
    def __init__(self):
        super().__init__('drone_info_publisher')
        self.publisher_ = self.create_publisher(String, 'drone_info', 10)
        self.timer = self.create_timer(3.0, self.publish_drone_info)
        self.the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
        self.the_connection.wait_heartbeat()
        self.get_logger().info("Conectado a MAVLink")

    def publish_drone_info(self):
        msg = String()
        
        # Obtener datos de MAVLink (ejemplo de lectura de estado de actitud)
        attitude = self.the_connection.recv_match(type='ATTITUDE', blocking=True)
        if attitude:
            roll = attitude.roll
            pitch = attitude.pitch
            yaw = attitude.yaw

            msg.data = f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publicando: {msg.data}')
        else:
            self.get_logger().info("No se recibieron datos de actitud")

def main(args=None):
    rclpy.init(args=args)
    drone_publisher = DronePublisher()

    try:
        rclpy.spin(drone_publisher)
    except KeyboardInterrupt:
        pass

    drone_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
