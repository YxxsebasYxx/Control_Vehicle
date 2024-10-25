#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('teleop_turtle')

        # Obtén los parámetros de los ejes y las escalas desde el parámetro del servidor
        self.declare_parameter('axis_linear', 1)  # Eje para moverse adelante/atrás
        self.declare_parameter('axis_angular', 2)  # Eje para la rotación
        self.declare_parameter('scale_linear', 1.0)  # Escala para el movimiento lineal
        self.declare_parameter('scale_angular', 1.0)  # Escala para la rotación

        self.linear_axis = self.get_parameter('axis_linear').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('axis_angular').get_parameter_value().integer_value
        self.linear_scale = self.get_parameter('scale_linear').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('scale_angular').get_parameter_value().double_value

        # Publicador en el tópico de velocidad para turtlesim
        self.vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Suscriptor al tópico 'joy' para recibir los comandos del joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy):
        # Crear un mensaje de tipo Twist para mover la tortuga
        twist = Twist()

        # Asignar los valores de los ejes del joystick a los movimientos
        twist.linear.x = self.linear_scale * joy.axes[self.linear_axis]  # Adelante/Atrás
        twist.angular.z = self.angular_scale * joy.axes[self.angular_axis]  # Rotación

        # Publicar los comandos de movimiento
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_turtle = TeleopTurtle()
    rclpy.spin(teleop_turtle)

    # Cerrar el nodo correctamente
    teleop_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
