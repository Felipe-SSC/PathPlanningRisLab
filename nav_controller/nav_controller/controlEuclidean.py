import rclpy
import numpy as np
import csv

from nav_controller.funciones import *


def heuristic(a, b):
    # Implementacion Euclidiana
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def main(args=None):

    with open("tabla_euclidean.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["X Deseada", "Y Deseada", "Tiempo Path Building", "Tiempo de Trayectoria", "Tiempo Total"])


    rclpy.init(args=args)
    navigation_control = navigationControl(heuristic)
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
