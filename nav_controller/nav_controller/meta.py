import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        
        self.initial_pose_publisher = self.create_publisher(PoseStamped, 'initial_pose', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Temporizador para enviar datos una vez
        self.timer = self.create_timer(1.0, self.timer_callback)  # Esperar 1 segundo antes de enviar
        self.has_published_initial = False
        self.has_published_goal = False

    def timer_callback(self):
        # Enviar /initial_pose si no se ha enviado
        if not self.has_published_initial:
            self.send_initial_pose()
            self.get_logger().info(f"Pose Estimada enviada a /initial_pose")
            self.has_published_initial = True
            return  # Salir para esperar el siguiente ciclo

        # Enviar /goal_pose si no se ha enviado
        if not self.has_published_goal:
            time.sleep(1.0)  # Esperar 1 segundo después de enviar /initial_pose
            self.send_goal_pose()
            self.get_logger().info(f"Pose de destino enviada a /goal_pose")
            self.has_published_goal = True

            # Detener el nodo después de enviar ambos mensajes
            rclpy.shutdown()

    def send_initial_pose(self):
        # Crear un mensaje PoseStamped para /initial_pose
        initial_pose = PoseStamped()

        # Llenar el header
        initial_pose.header.stamp = self.get_clock().now().to_msg()  # Tiempo actual
        initial_pose.header.frame_id = 'odom'  # Marco de referencia

        # Llenar la pose con los valores proporcionados
        initial_pose.pose.position.x = 8.490727149709247e-05
        initial_pose.pose.position.y = -4.8196835380291676e-06
        initial_pose.pose.position.z = 0.00852962402478179

        initial_pose.pose.orientation.x = 0.0002232152957241116
        initial_pose.pose.orientation.y = 0.0028908176015816594
        initial_pose.pose.orientation.z = 7.922726896504319e-06
        initial_pose.pose.orientation.w = 0.999995796634044

        # Publicar el mensaje
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info('Initial Pose enviada:')
        self.get_logger().info(f'Header: {initial_pose.header}')
        self.get_logger().info(f'Pose: {initial_pose.pose}')

    def send_goal_pose(self):
        # Crear un mensaje PoseStamped para /goal_pose
        goal_pose = PoseStamped()

        # Llenar el header
        goal_pose.header.stamp = self.get_clock().now().to_msg()  # Tiempo actual
        goal_pose.header.frame_id = 'map'  # Marco de referencia

        # Pose test1
        # goal_pose.pose.position.x = -0.013509
        # goal_pose.pose.position.y = 2.0156
        # goal_pose.pose.position.z = 0.0

        #pose test2
        # goal_pose.pose.position.x = 2.0
        # goal_pose.pose.position.y = 4.0
        # goal_pose.pose.position.z = 0.0

        #Pose test3
        goal_pose.pose.position.x = -2.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Publicar el mensaje
        self.goal_pose_publisher.publish(goal_pose)
        self.get_logger().info('Goal Pose enviada:')
        self.get_logger().info(f'Header: {goal_pose.header}')
        self.get_logger().info(f'Pose: {goal_pose.pose}')

def main(args=None):
    rclpy.init(args=args)

    try:
        # Crear el nodo
        node = PosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Manejar la interrupción del usuario (Ctrl+C)
    finally:
        # Cerrar el nodo correctamente
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()