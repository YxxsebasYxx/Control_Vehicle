#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
import time


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.state = None
        self.connected = False

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Control variables
        self.pose = PoseStamped()
        self.velocity = TwistStamped()

        # Initial values for position and velocity
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0  # Set a safe takeoff altitude
        self.velocity.twist.linear.x = 0.0
        self.velocity.twist.linear.y = 0.0
        self.velocity.twist.linear.z = 0.0

    def state_cb(self, msg):
        self.state = msg
        if self.state.connected:
            self.connected = True

    def wait_for_connection(self):
        self.get_logger().info("Waiting for FCU connection...")
        while rclpy.ok() and not self.connected:
            rclpy.spin_once(self)
        self.get_logger().info("FCU connected")

    def change_mode(self, mode):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        mode_req = SetMode.Request()
        mode_req.custom_mode = mode
        future = self.set_mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"Mode changed to {mode}")
        else:
            self.get_logger().error(f"Failed to change mode to {mode}")

    def arm_drone(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arming_client.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone armed")
        else:
            self.get_logger().error("Failed to arm the drone")

    def takeoff(self):
        self.get_logger().info("Starting takeoff sequence...")
        self.change_mode("OFFBOARD")
        time.sleep(2)  # Ensure the mode change is complete
        self.get_logger().info("Publicando setpoints antes de armar el dron...")
        for _ in range(100):  # Publicar 100 setpoints durante 5 segundos
            self.pose_pub.publish(self.pose)
            rclpy.spin_once(self)
            time.sleep(0.05)  # Espera 50ms entre publicaciones

        self.get_logger().info("Intentando armar el dron...")
        self.arm_drone()

    
        # Publish initial setpoints for stability before takeoff
        for _ in range(100):
            self.pose_pub.publish(self.pose)
            rclpy.spin_once(self)
            time.sleep(0.05)

        self.get_logger().info("Taking off...")

        # Publicar la posición hasta alcanzar la altura deseada
        target_altitude = 10.0  # Altura de despegue deseada
        while rclpy.ok():
            self.pose.pose.position.z = target_altitude
            self.pose_pub.publish(self.pose)
            rclpy.spin_once(self)
            time.sleep(0.1)

            # Comprobar si el dron ha alcanzado la altura deseada
            if abs(self.pose.pose.position.z - target_altitude) < 0.5:
                self.get_logger().info("Drone has reached target altitude.")
                break

    def move(self, Px, Py, Pz, Vx, Vy, Vz):
        self.get_logger().info(f"Moving to position: ({Px}, {Py}, {Pz})")
        self.pose.pose.position.x = float(Px)
        self.pose.pose.position.y = float(Py)
        self.pose.pose.position.z = float(Pz)

        for _ in range(100):
            self.pose_pub.publish(self.pose)
            rclpy.spin_once(self)
            time.sleep(0.05)

        self.get_logger().info(f"Moving with velocity: ({Vx}, {Vy}, {Vz})")
        self.velocity.twist.linear.x = float(Vx)
        self.velocity.twist.linear.y = float(Vy)
        self.velocity.twist.linear.z = float(Vz)

        for _ in range(100):
            self.vel_pub.publish(self.velocity)
            rclpy.spin_once(self)
            time.sleep(0.05)



def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()

    # Esperar conexión
    drone_controller.wait_for_connection()

    command_list = []
    verification = True
    while verification:
        print("""
        Opción 0: Takeoff
        Opción 1: Mover en X
        Opción 2: Mover en Y
        Opción 3: Mover en Z
        Opción 4: Velocidad en X
        Opción 5: Velocidad en Y
        Opción 6: Velocidad en Z 
        Opción 7: Ejecutar""")
    
        var = input("Ingrese la opción: ")

        if var == "0":
            command_list.append(('takeoff', None))
        elif var == "1":
            Px = float(input("Ingrese la posición en X: "))
            command_list.append(('move', (Px, 0, 2, 0, 0, 0)))
        elif var == "2":
            Py = float(input("Ingrese la posición en Y: "))
            command_list.append(('move', (0, Py, 2, 0, 0, 0)))
        elif var == "3":
            Pz = float(input("Ingrese la posición en Z: "))
            command_list.append(('move', (0, 0, Pz, 0, 0, 0)))
        elif var == "4":
            Vx = float(input("Ingrese la velocidad en X: "))
            command_list.append(('move', (0, 0, 2, Vx, 0, 0)))
        elif var == "5":
            Vy = float(input("Ingrese la velocidad en Y: "))
            command_list.append(('move', (0, 0, 2, 0, Vy, 0)))
        elif var == "6":
            Vz = float(input("Ingrese la velocidad en Z: "))
            command_list.append(('move', (0, 0, 2, 0, 0, Vz)))
        elif var == "7":
            print("Ejecutando los comandos:")
            for command in command_list:
                if command[0] == 'takeoff':
                    drone_controller.takeoff()
                elif command[0] == 'move':
                    Px, Py, Pz, Vx, Vy, Vz = command[1]
                    drone_controller.move(Px, Py, Pz, Vx, Vy, Vz)
            command_list.clear()  # Clear commands after execution
        else:
            print("Opción incorrecta. Saliendo del programa.")
            verification = False

    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

