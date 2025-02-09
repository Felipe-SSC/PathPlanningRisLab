import rclpy
import csv

from nav_controller.funciones import *

def heuristic(a, b):
    # Implementacion Manhattan
    absX = abs(a[0] - b[0])
    absY = abs(a[1] - b[1])
    manhattan = absX + absY
    return manhattan

def main(args=None):

    with open("tablaDatos.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["X Deseada", "Y Deseada", "Tiempo Path Building", "Tiempo de Trayectoria", "Tiempo Total", "Distancia Total"])


    rclpy.init(args=args)
    navigation_control = navigationControl(heuristic)
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()