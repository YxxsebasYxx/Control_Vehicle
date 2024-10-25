#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('xbox_teleop')

        # Configuración del perfil QoS con Durability Transient Local para publicación
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Configuración del perfil QoS con Durability Volatile para suscripción
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publicadores con Durability Transient Local
        self.pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile_pub)
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos_profile_pub)

        # Suscriptor con Durability Volatile
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sub)

        self.arm_toggle = False

    def joy_callback(self, joy):
        twist = Twist()

        # Asigna los ejes del joystick a los comandos de movimiento
        twist.linear.x = joy.axes[1]  # Eje izquierdo Y
        twist.linear.y = joy.axes[0]  # Eje izquierdo X
        twist.linear.z = joy.axes[4]  # Eje derecho Y
        twist.angular.z = joy.axes[3]  # Eje derecho X

        self.pub.publish(twist)

        # Botón A para armar/desarmar
        if joy.buttons[0] == 1:  # Botón 'A' en el controlador Xbox
            self.arm_toggle = not self.arm_toggle
            arm_msg = Bool()
            arm_msg.data = self.arm_toggle
            self.arm_pub.publish(arm_msg)
            self.get_logger().info(f"Arm toggle is now: {self.arm_toggle}")

def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
