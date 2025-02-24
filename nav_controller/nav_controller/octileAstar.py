import rclpy
import os
import csv
import math
from nav_controller.funcionesAstar import *

def heuristic(a, b):
    D = 1
    D2 = math.sqrt(2)
    
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

heuristic.__name__ = "Astar-Octile"

def main(args=None):

    file_path = "tablaDatos.csv"

    # Comprobar si el archivo ya existe y tiene contenido
    file_exists = os.path.exists(file_path) and os.path.getsize(file_path) > 0

    with open(file_path, mode="a", newline="") as file:  
        writer = csv.writer(file)

        # Si el archivo no existe o está vacío, escribir los encabezados
        if not file_exists:
            writer.writerow(["Simulacion", "X Deseada", "Y Deseada", "Tiempo Path Building", "Tiempo de Trayectoria", "Tiempo Total", "Distancia Total","Numero de Giros"])


    rclpy.init(args=args)
    navigation_control = navigationControl(heuristic)
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
