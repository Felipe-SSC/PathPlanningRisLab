import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Suscriptor a /odom
        self.odometry_suscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Publicadores
        self.initial_pose_publisher = self.create_publisher(PoseStamped, 'initial_pose', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Variables para almacenar la posición y orientación inicial
        self.initial_x = None
        self.initial_y = None
        self.initial_orientation = None  # Guardar la orientación en cuaterniones

        # Bandera para indicar si se envió la posición inicial
        self.initial_pose_sent = False

    def odom_callback(self, msg):
        """Extrae la posición y orientación inicial del robot desde /odom"""
        if self.initial_x is None and self.initial_y is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.initial_orientation = msg.pose.pose.orientation  # Guardar la orientación
            self.get_logger().info(f'Posición inicial guardada: ({self.initial_x}, {self.initial_y})')

    def send_initial_pose(self):
        """Publica la posición inicial en /initial_pose"""
        if self.initial_x is None or self.initial_y is None or self.initial_orientation is None:
            self.get_logger().warn('No se ha recibido la odometría aún.')
            return

        initial_pose = PoseStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'  # Enviar en el marco del mapa si usas AMCL

        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y
        initial_pose.pose.position.z = 0.0

        # Incluir la orientación del robot en cuaterniones
        initial_pose.pose.orientation = self.initial_orientation

        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info('Initial Pose enviada correctamente.')
        self.initial_pose_sent = True  # Evita reenviar la pose inicial

    def send_goal_pose(self):
        """Publica la meta en /goal_pose"""
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.position.x = 0.0  # Puedes modificar esta meta
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0

        # Mantener la orientación del robot en la pose inicial
        goal_pose.pose.orientation = self.initial_orientation if self.initial_orientation else self.get_identity_orientation()

        self.goal_pose_publisher.publish(goal_pose)
        self.get_logger().info('Goal Pose enviada correctamente.')

    def get_identity_orientation(self):
        """Devuelve una orientación neutra (sin rotación) en cuaterniones"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        return q

def main(args=None):
    rclpy.init(args=args)

    node = PosePublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)  # Procesa callbacks

            if node.initial_x is not None and not node.initial_pose_sent:
                node.send_initial_pose()
                time.sleep(1)  # Espera antes de enviar el goal
                node.send_goal_pose()
                break  # Termina después de enviar

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
