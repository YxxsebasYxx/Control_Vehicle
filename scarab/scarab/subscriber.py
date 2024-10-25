#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import time

class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients for arming and mode change
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Drone control variables
        self.pose = PoseStamped()
        self.pose.pose.position.z = 2.0  # Initial safe height
        self.is_armed = False
    import time

    def publish_setpoints(self):
        self.get_logger().info('Publicando setpoints...')
    
    # Publica setpoints por 5 segundos antes de cambiar a OFFBOARD
        for _ in range(10):  # Envía durante 5 segundos
            self.pose.pose.position.x = 0.0
            self.pose.pose.position.y = 0.0
            self.pose.pose.position.z = 1.0  # Despegue a 1 metro
            self.pose_pub.publish(self.pose)
            time.sleep(0.5)

        self.get_logger().info('Setpoints publicados.')


    def joy_callback(self, joy_msg):
        # Map buttons and axes from the joystick
        if joy_msg.buttons[0]:  # Assuming button 0 (A button) is for takeoff
            if not self.is_armed:
                self.takeoff()

        # Process joystick axes to control drone movements (e.g., with left stick)
        self.pose.pose.position.x = 5.0 * joy_msg.axes[1]  # forward/backward
        self.pose.pose.position.y = 5.0 * joy_msg.axes[0]  # left/right
        self.pose.pose.position.z += 0.1 * joy_msg.axes[4]  # up/down
        self.pose_pub.publish(self.pose)
        self.get_logger().info(f'Joystick: {joy_msg.axes}, {joy_msg.buttons}')


    def arm_drone(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arming_client.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Drone armed')
            self.is_armed = True
        else:
            self.get_logger().error('Failed to arm the drone')

    def change_mode(self, mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = mode
        future = self.set_mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Mode changed to {mode}')
        else:
            self.get_logger().error(f'Failed to change mode to {mode}')


    def takeoff(self):
        self.get_logger().info('Preparing for takeoff...')
        self.pose.pose.position.z = 0.0  # Inicializa en el suelo
        self.publish_setpoints()  # CORRECTO: llamando a la función con 'self'
    
    # Paso 1: Publicar mensajes de posición durante 2 segundos antes de cambiar a OFFBOARD
        self.change_mode('OFFBOARD')
        self.arm_drone()
    
    # Paso 2: Subir el dron a 10 metros
        self.pose.pose.position.z = 5.0
        self.pose_pub.publish(self.pose)

        self.get_logger().info('Takeoff command sent')




def main(args=None):
    rclpy.init(args=args)
    drone_teleop = DroneTeleop()
    rclpy.spin(drone_teleop)
    drone_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
