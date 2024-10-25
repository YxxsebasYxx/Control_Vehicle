#!/usr/bin/env python3

import subprocess
import time
import os

# Define las rutas
qgc_path = os.path.expanduser("~/Downloads")
px4_path = os.path.expanduser("~/PX4-Autopilot")

# Lista de comandos
commands = [
    # Comando para lanzar Gazebo con PX4
    f"cd {px4_path} && ros2 launch px4 mavros_posix_sitl.launch.py",
    f"cd {px4_path} &&  make px4_sitl_default gazebo",
    # Comando para lanzar QGroundControl
    f"cd {qgc_path} && ./QGroundControl.AppImage",
]

# Ejecuta cada comando en una nueva terminal
processes = []
for command in commands:
    try:
        # Abre cada comando en una nueva pestaña de terminal
        process = subprocess.Popen(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
        processes.append(process)
        time.sleep(1)  # Pausa para dar tiempo a que cada proceso inicie correctamente
    except subprocess.CalledProcessError as e:
        print(f"Error ejecutando el comando: {command}")
        print(e)
