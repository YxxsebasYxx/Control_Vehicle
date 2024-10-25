#!/usr/bin/env python3

import subprocess
import time

# Expande la ruta del usuario
gazebo_world_path = "~/ardupilot_gazebo/worlds/iris_ardurover.world"
ardupilot_path = "~/ardupilot/Rover"
qgc_path = "~/Downloads"
MS_path = "~/MissionPlanner-latest"

# Lista de comandos
commands = [
    # Comando para lanzar Gazebo con el plugin que permite la interacción con ROS 2
    f"gazebo --verbose {gazebo_world_path}",
    
    # Comando para ejecutar ArduPilot y conectarlo a Gazebo
    f"cd {ardupilot_path} && sim_vehicle.py -v Rover --console --out=127.0.0.1:14550 --out=127.0.0.1:14551",
    
    # Comando para lanzar QGroundControl
    #f"cd {qgc_path} && ./QGroundControl.AppImage"

    f"cd {MS_path} && MONO_LOG_LEVEL=debug mono MissionPlanner.exe"
]

# Ejecuta cada comando en una nueva pestaña de terminal
processes = []
for command in commands:
    try:
        # Abre cada comando en una nueva pestaña del terminal
        process = subprocess.Popen(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
        processes.append(process)
        time.sleep(1)  # Pausa para dar tiempo a que cada proceso inicie correctamente
    except subprocess.CalledProcessError as e:
        print(f"Error ejecutando el comando: {command}")
        print(e)
