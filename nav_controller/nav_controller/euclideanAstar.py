import rclpy
import numpy as np
import csv
import os

from nav_controller.funcionesAstar import *

def heuristic(a, b):
    # Implementacion Euclidiana
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

heuristic.__name__ = "Astar-Euclidiana"

def main(args=None):

    file_path = "tablaDatos.csv"

    # Comprobar si el archivo ya existe y tiene contenido
    file_exists = os.path.exists(file_path) and os.path.getsize(file_path) > 0

    with open(file_path, mode="a", newline="") as file:  
        writer = csv.writer(file)

        # Si el archivo no existe o está vacío, escribir los encabezados
        if not file_exists:
            writer.writerow(["X Deseada", "Y Deseada", "Tiempo Path Building", "Tiempo de Trayectoria", "Tiempo Total", "Distancia Total"])


    rclpy.init(args=args)
    navigation_control = navigationControl(heuristic)
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
