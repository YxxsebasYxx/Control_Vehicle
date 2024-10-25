#!/usr/bin/env python3

from pymavlink import mavutil
import time
import sys

def main():
    Px = 0
    Py = 0
    Pz = 0
    Vx = 0
    Vy = 0
    Vz = 0
    # Conectar a MAVLink
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    # Esperar a recibir un heartbeat
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))

    # Definir máscara de control de posición
    position_mask = int(0b110111111000)  # Solo posición, ignorar otras características

    def change_mode(mode):
    # Verificar si el modo es válido
        if mode not in the_connection.mode_mapping():
            print(f'Unknown mode: {mode}')
            print('Try:', list(the_connection.mode_mapping().keys()))
            sys.exit(1)

        mode_id = the_connection.mode_mapping()[mode]
    # Enviar comando para cambiar el modo
        the_connection.mav.set_mode_send(
            the_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    
    def takeoff():
    # Cambiar al modo GUIDED antes de armar
        change_mode('GUIDED')
    # Esperar un momento para asegurarse de que el modo ha cambiado
        time.sleep(3)

    # Enviar el comando de armado
        the_connection.mav.command_long_send(
            the_connection.target_system,    # target system
            the_connection.target_component, # target component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command (arm/disarm)
            0,                               # Confirmation
            1, 0, 0, 0, 0, 0, 0)             # Parametros (1 para armar, el resto son ceros)
        print("Arm command sent")

    # Esperar unos segundos para asegurarse de que el dron está armado
        the_connection.motors_armed_wait()
        print("Dron armado, listo para el despegue")

    # Esperar un momento antes de enviar el comando de despegue
        time.sleep(4)

    # Enviar el comando de despegue
        altitude = 10  # Altitud de despegue en metros
        the_connection.mav.command_long_send(
            the_connection.target_system,    # target system
            the_connection.target_component, # target component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Comando para despegar
            0,                               # Confirmation
            0, 0, 0, 0, 0, 0, altitude)      # Parametros: 0s para los demás, y la altitud final
        print(f"Takeoff command sent, altitude: {altitude} meters")

    # Esperar un momento después del despegue
        time.sleep(2)
    def move():
        # Enviar el comando de movimiento
        the_connection.mav.set_position_target_local_ned_send(
            10,  
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            position_mask,
            Px, Py, -Pz, 
            Vx, Vy, Vz,  
            0, 0, 0,  
            0, 0  
        )
        print(f"Moviendo a posición: ({Px}, {Py}, {Pz}) con velocidad ({Vx}, {Vy}, {Vz})")
        time.sleep(10)  # Esperar para que el movimiento tenga lugar


    class Menu():
        def __init__(self, case1, case2, case3, case4, case5, case6):
            self.case1 = case1
            self.case2 = case2
            self.case3 = case3
            self.case4 = case4
            self.case5 = case5
            self.case6 = case6

    menu = Menu("X", "Y", "Z", "x", "y", "z")


    verification = True
    while verification:
        print(f"""
        Opcion 0: Takeoff
        Opción 1: Mover en {menu.case1}
        Opción 2: Mover en {menu.case2}
        Opción 3: Mover en {menu.case3}
        Opción 4: Velocidad de {menu.case4}
        Opción 5: Velocidad de {menu.case5}
        Opción 6: Velocidad de {menu.case6} 
        Opcion 7: Ejecutar""")
    
        var = str(input("Ingrese la opción del planeador: "))

        if var == "1":
            var1 = float(input(f"Ingrese cuanto quiere mover en {menu.case1}: "))
            Px = var1
        elif var == "0":
            takeoff()
        elif var == "2":
            var2 = float(input(f"Ingrese cuanto quiere mover en {menu.case2}: "))
            Py = var2
        elif var == "3":
            var3 = float(input(f"Ingrese cuanto quiere mover en {menu.case3}: "))
            Pz = var3
        elif var == "4":
            var4 = float(input(f"Ingrese la velocidad en {menu.case4}: "))
            Vx = var4
        elif var == "5":
            var5 = float(input(f"Ingrese la velocidad en {menu.case5}: "))
            Vy = var5
        elif var == "6":
            var6 = float(input(f"Ingrese la velocidad en {menu.case6}: "))
            Vz = var6
        elif var == "7":
            move()  # Asegúrate de que las variables tengan valores válidos antes de llamar a move()
        else:
            print("Opción incorrecta. Saliendo del programa.")
            verification = False

if __name__=='__main__':
    main()