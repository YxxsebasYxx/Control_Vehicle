# Control_Vehicle
Este repositorio se realizon con la finalidad de compartir el codigo base para manipular de forma teleoperada los drones simulados en gazebo

#PX4_ROS_COM

# Teleoperacion con mando con turtlesim 

ros2 launch px4_ros_com vehicle_command_publisher.launch.py

# Teleoperacion con mando con la simulacion de ros2_px4_offboard_example_ws

ros2 launch px4_ros_com teleop_dron.launch.py

# URDF_BASICS

# Launch para lanzar el modelo del carrito

ros2 launch urdf_basics launch_sim.launch.py 

# Launch para lanzar la teleoperacion con el mando

ros2 launch urdf_basics dron_xbox.launch.py 
