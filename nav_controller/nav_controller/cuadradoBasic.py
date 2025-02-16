#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

def euler_from_quaternion(quaternion):
       x = quaternion[0]
       y = quaternion[1]
       z = quaternion[2]
       w = quaternion[3]
       #sinr_cosp = 2 * (w * x + y * z)
       #cosr_cosp = 1 - 2 * (x * x + y * y)
       #roll = np.arctan2(sinr_cosp, cosr_cosp)    # Solo nos interesa el yaw
       #sinp = 2 * (w * y - z * x)
       #pitch = np.arcsin(sinp)
       siny_cosp = 2 * (w * z + x * y)
       cosy_cosp = 1 - 2 * (y * y + z * z)
       yaw = np.arctan2(siny_cosp, cosy_cosp)
   
       return yaw

def normalizeAngle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


class Trajectory(Node):
    def __init__(self, nombreNodo):
        super().__init__(nombreNodo)

        #----------Pathing-----------
        self.longitud = 2
        self.velTrayectoria = 0.2
        self.tm = 1
        self.NptosLado = round(self.longitud / (self.velTrayectoria * self.tm))
        self.wfilter = 0
        
        self.pathMapeo = [2.00, 2.00, 0, 2.00, 2.00, 0], [0, 2.00, 2.00, 2.00, 0, 0]

        # Variables de posicion
        self.posX = 0
        self.posY = 0
        self.yaw = 0
        self.posX_deseada = 0
        self.posY_deseada = 0

        # Velocidades
        self.vel_linear_x = 0.4
        self.vel_angular_z = 0.2

        # Errores
        self.linear_error = 0.08
        self.angular_error = 0.05

        # Indice para recorrer puntos del pathing
        self.index = 0
        self.fase = 1

        # Suscriber al odometry
        self.suscriber = self.create_subscription(
            Odometry, 
            '/odom', 
            self.listener_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # self.suscriberClock = self.create_subscription(
        #     Clock, 
        #     '/clock', 
        #     self.listenerClock_callback, 
        #     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # Publisher al cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Tipo de mensaje que se publica en cmd_vel
        self.twist = Twist()

        # Creacion de timer --> para publicar timer_callback
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.final_yaw_reached = False

    def timer_callback(self):
        self.posX_list, self.posY_list = self.pathMapeo
        if self.index < len(self.posX_list):  # Asegurarse de no exceder los límites
            self.posX_deseada = self.posX_list[self.index]
            self.posY_deseada = self.posY_list[self.index]

            # Fase 1 : Rotacion 
            if self.fase == 1:
                self.anguloDeseado = math.atan2(self.posY_deseada - self.posY, self.posX_deseada - self.posX)
                self.angFaltante = self.yaw - self.anguloDeseado
                self.angFaltanteNorm = normalizeAngle(self.angFaltante)

                if abs(self.angFaltanteNorm) > self.angular_error:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.vel_angular_z if self.angFaltanteNorm < 0 else -self.vel_angular_z

                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.fase = 2                  # Pasamos a la fase 2

            # Fase 2 : Movimiento Recta
            if self.fase == 2:
                self.distanciaVector = math.sqrt((self.posX - self.posX_deseada)**2 + (self.posY - self.posY_deseada)**2)
                
                if self.distanciaVector > self.linear_error:
                    self.twist.linear.x = self.vel_linear_x
                    self.twist.angular.z = 0.0
                
                else:   # Si estamos dentro del error
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.fase = 1                  # Pasamos a la fase 1
                    
                    self.index += 1                # Siguiente posición deseada
        elif not self.final_yaw_reached:  # Fase final: Ajustar yaw a 0
            self.anguloDeseado = 0.0  # Apuntar a yaw = 0
            self.angFaltante = self.yaw - self.anguloDeseado
            self.angFaltanteNorm = normalizeAngle(self.angFaltante)

            if abs(self.angFaltanteNorm) > self.angular_error:
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.vel_angular_z if self.angFaltanteNorm < 0 else -self.vel_angular_z
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.final_yaw_reached = True  # Marcar que se alcanzó el yaw final
                self.get_logger().info("Trayectoria completada y orientación final ajustada.")
        elif self.final_yaw_reached:
            # Mantener el robot detenido después de completar la trayectoria y el ajuste de yaw
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist) # Asegurarse de que se publiquen las velocidades finales
        
        self.publisher.publish(self.twist)

    def listener_callback(self, msg):
        self.posX = msg.pose.pose.position.x
        self.posY = msg.pose.pose.position.y
        quaternion = [msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w]
        
        self.yaw = euler_from_quaternion(quaternion)
    
    #def listenerClock_callback(self, msg):
    #    self.get_logger().info('Clock: "%f"' % msg.clock.sec)

def main(args=None) -> None:

    rclpy.init(args=args)
    robot = Trajectory('nodeTrajectoryBasic')
    
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()