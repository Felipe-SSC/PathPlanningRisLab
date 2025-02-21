from rclpy.node import Node
import heapq
import rclpy
import math
from nav_msgs.msg import OccupancyGrid , Odometry, Path
from geometry_msgs.msg import PoseStamped , Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile
import csv
import os

from nav_controller.funcionesAstar import euler_from_quaternion, bspline_planning, aproximar_a_cero, costmap, pure_pursuit

#Nombre:
SIMULACION = 'Djikstra'



def dijkstra(self, array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    
    
    closed_set = set()  # Nodos ya evaluados
    came_from = {}      # Diccionario para reconstruir el camino
    gscore = {start: 0} # Costo acumulado desde el inicio
    open_list = []      # Cola de prioridad para nodos pendientes
    
    # Inicializar con el nodo de inicio
    heapq.heappush(open_list, (gscore[start], start))
    
    #mientras falten nodos por revisar
    while open_list:
        # Obtener el nodo con el menor costo acumulado
        current = heapq.heappop(open_list)[1]
        
        # Si llegamos al objetivo, reconstruir y devolver el camino
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path = path + [start]
            path = path[::-1]  # Invertir para obtener el camino desde el inicio
            return path
        
        # Marcar el nodo actual como evaluado
        closed_set.add(current)
        
        # Explorar los vecinos
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            
            # Verificar si el vecino está dentro de los límites del array
            if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                # Si el vecino es un obstáculo, ignorarlo
                if array[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                # Si el vecino está fuera de los límites, ignorarlo
                continue
            
            # Calcular el costo acumulado tentativo para el vecino
            move_cost = 1 if abs(i) + abs(j) == 1 else 1.41
            tentative_g_score = gscore[current] + move_cost

            # Si el vecino ya fue evaluado y el nuevo costo no es mejor, ignorarlo
            if neighbor in closed_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
            
            # Si el nuevo costo es mejor o el vecino no está en la lista abierta
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_list]:
                came_from[neighbor] = current  # Registrar el camino
                gscore[neighbor] = tentative_g_score  # Actualizar el costo acumulado
                heapq.heappush(open_list, (gscore[neighbor], neighbor))  # Agregar a la cola
    
    # Si no se encontró un camino al objetivo, buscar el nodo más cercano
    if goal not in came_from:
        self.get_logger().info("No se encontro la ruta a la meta!!")
        closest_node = None
        closest_dist = float('inf')
        for node in closed_set:
            # Calcular la distancia Manhattan al objetivo
            dist = abs(node[0] - goal[0]) + abs(node[1] - goal[1])
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            # Reconstruir el camino hasta el nodo más cercano
            path = []
            while closest_node in came_from:
                path.append(closest_node)
                closest_node = came_from[closest_node]
            path = path + [start]
            path = path[::-1]  # Invertir para obtener el camino desde el inicio
            return path
    
    # Si no se encontró ningún camino, devolver False
    return False

class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.listener_callback, 10)
        self.subscription = self.create_subscription(Odometry,'odom',self.info_callback,10)
        self.subscription = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10))
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.pursue_publisher_ = self.create_publisher(Marker, '/pure_pursuit_target', 10)
        self.robot_path_publisher = self.create_publisher(Path, '/robot_path', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.flag = 0
        self.time_moverse = 0
        self.timeGoal = 0
        self.pathBuildTime = 0
        self.move_0 = 0
        self.move_1 = 0
        self.goal_0 = 0
        self.goal_1 = 0
        self.path_0 = 0
        self.path_1 = 0
        self.total_length = 0 
        self.puntos_recorridos = []
        self.giros = 0
    
    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'  # O el marco adecuado
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=0.0)
        marker.scale.x = 0.1  # Tamaño de la esfera
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Opacidad
        marker.color.r = 1.0  # Color rojo
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.pursue_publisher_.publish(marker)


    def goal_pose_callback(self,msg):  #Funcion que se suscribe al /goal_pose y printea la posicion de destino.
        
        self.goal_0 = self.get_clock().now()
        
        self.goal = (msg.pose.position.x,msg.pose.position.y)
        
        self.get_logger().info(f'Goal Pose recibida: {self.goal[0],self.goal[1]}')
        self.flag = 1  ##Una vez se recibe un goal_pose se pasa al estado 1.

    def listener_callback(self,msg):  ##Trabaja con la Occupancy Grid obtenida de 'map'. 
        if self.flag == 1:
            resolution = msg.info.resolution  ##Resolution del mapa en metros por celda  (Cuantos metros son una celda)
            
            ##Posiciones origen en metros
            originX = msg.info.origin.position.x  
            originY = msg.info.origin.position.y


            #Conversion de cordenadas a indice del Costmap
            column = int((self.x- originX)/resolution) 
            row = int((self.y- originY)/resolution) 
            
            #Conversion de las coordenadas del objetivo a indices del Costmap
            columnH = int((self.goal[0]- originX)/resolution)
            rowH = int((self.goal[1]- originY)/resolution)
            self.get_logger().info(f'El goal es: {self.goal[0],self.goal[1]}')

            #Para trabajar con el costmap, se crea una instancia con el nombre 'data'
            data = costmap(msg.data,msg.info.width,msg.info.height,resolution) 
            data[row][column] = 0 #Convierte Posicion del robot a 0
            data[data < 0] = 1 #Celdas desconocidas se convierten en obstáculos
            data[data > 5] = 1  #Celdas con ALTO COSTO se convierten en obstáculos

            self.get_logger().info('Generando un PATH con Dijkstra')
            
            self.path_0 = self.get_clock().now()
            
            path = dijkstra(self, data,(row,column),(rowH,columnH)) #Busqueda de ruta con Dijkstra
            ## -> Resulta en una lista de indices (fila, columna) que representa la ruta.
            self.path_1 = self.get_clock().now()

            self.pathBuildTime = ((self.path_1 - self.path_0).nanoseconds / 1e9)
            
            path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path] #Convertir indices a coordenadas (x, y)
            
            self.path = bspline_planning(path) #Corrección de ruta con BSpline. (Suavizar la ruta)
            print("Ubicacion del Robot: ",self.x,self.y)
            self.giros= self.count_turns(self.path)
            
            #Mods para el path:
            # Publicar el path en el tópico 'planned_path'
            path_msg = Path()
            path_msg.header.frame_id = "map"  # El frame debe coincidir con el frame de tu mapa
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for point in self.path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # Sin rotación
                path_msg.poses.append(pose)

            ##calculo de distancia path
            self.path_length(self.path)
            self.get_logger().info(f'La distancia total es: {self.total_length}')
            
            

            self.path_publisher.publish(path_msg)

            self.i = 0
            self.move_0 = self.get_clock().now()
            self.get_logger().info('Path generado! Publicando twist')
            self.flag = 2

    def path_length(self, path):
        self.total_length = 0
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            self.total_length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return self.total_length
    
    def count_turns(self, path):
            if len(path) < 3:  # Se necesitan al menos 3 puntos para detectar un giro
                return 0

            turns = 0
            prev_dx, prev_dy = path[1][0] - path[0][0], path[1][1] - path[0][1]

            for i in range(1, len(path) - 1):
                dx, dy = path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]

                if (dx, dy) != (prev_dx, prev_dy):  # Si cambia la dirección, es un giro
                    turns += 1

                prev_dx, prev_dy = dx, dy  # Actualizar la dirección previa

            return turns

    def printData(self):
        self.get_logger().info(f'Guardando datos en el archivo')

        with open('tablaDatos.csv', mode="a", newline="") as file:  
            writer = csv.writer(file)

            # Escribir los datos de la simulación
            writer.writerow([
                f'{SIMULACION}',
                aproximar_a_cero(self.goal[0]),
                aproximar_a_cero(self.goal[1]),
                self.pathBuildTime,
                self.time_moverse,
                self.timeGoal,
                self.total_length,
                self.giros
            ])
            writer.writerow([]) #en blanco


    def timer_callback(self):  ##Funcion para publicar las velocidades calculadas en Pure Pursuit. Ademas verifica si el robot ha llegado al ultimo pose del PATH   
        if self.flag == 2:
            twist = Twist()
            twist.linear.x , twist.angular.z,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,self.i, navigationControl=self)
            
            #----------------------------------------------------------------------------------------
            # Creacion de path recorrido por el robot en rviz
            robot_path_msg = Path()
            robot_path_msg.header.frame_id = "map"  # Asegúrate de que el frame sea el correcto para tu caso
            robot_path_msg.header.stamp = self.get_clock().now().to_msg()

            # Agregar los puntos recorridos al path
            for punto in self.puntos_recorridos:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(punto[0])
                pose.pose.position.y = float(punto[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # Sin rotación (puedes ajustarlo si es necesario)
                robot_path_msg.poses.append(pose)

            self.puntos_recorridos.append((self.x, self.y))
            #----------------------------------------------------------------------------------------           
            # Publicar el path de los puntos recorridos
            
            self.robot_path_publisher.publish(robot_path_msg)

            if(abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1])< 0.05):   #mi posicion (x, y) == la última pos. del PATH (x, y) (con error de 0.05) *****
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.flag = 0

                self.move_1 = self.get_clock().now()

                self.time_moverse = ((self.move_1 - self.move_0).nanoseconds / 1e9)

                self.goal_1 = self.get_clock().now()
                
                self.timeGoal = ((self.goal_1 - self.goal_0).nanoseconds / 1e9)
                #CALCULO LONGITUD PATH ROBOT 
                self.puntos_recorridos = []
                self.printData()
                print("Objetivo alcanzado!!\nEsperando nuevo objetivo...")

            self.vel_publisher.publish(twist) 

    def info_callback(self,msg):  #Funcion que recibe parámetros de odometría suscribiendose a /odom
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

def main(args=None):

    file_path = "tablaDatos.csv"

    # Comprobar si el archivo ya existe y tiene contenido
    file_exists = os.path.exists(file_path) and os.path.getsize(file_path) > 0

    with open(file_path, mode="a", newline="") as file:  
        writer = csv.writer(file)

        # Si el archivo no existe o está vacío, escribir los encabezados
        if not file_exists:
            writer.writerow(["Simulación","X Deseada", "Y Deseada", "Tiempo Path Building", "Tiempo de Trayectoria", "Tiempo Total", "Distancia Total","Numero de Giros"])


    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()