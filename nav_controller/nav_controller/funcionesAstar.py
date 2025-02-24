from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid , Odometry, Path
from geometry_msgs.msg import PoseStamped , Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import scipy.interpolate as si
from rclpy.qos import QoSProfile
import csv

lookahead_distance = 0.35  #previamente en 0.15 (0.35 esta bien quizas un poco menos). Distancia entre el robot y la posicion objetivo en Pure Pursuit
speed = 0.2 # velocidad del robot
expansion_size = 3 #distancia de la muralla para el costmap!!! (mas alto mas se aleja de la muralla, funciona bien con 4)

def euler_from_quaternion(x,y,z,w):
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z
 
def astar(array, start, goal, heuristic):
    """Implementación de A* con soporte para Manhattan, Euclidean y Octile."""

    if heuristic.__name__ == "Astar-Manhattan":
        neighbors = [(0,1), (0,-1), (1,0), (-1,0)]  # Solo ortogonales
        cost = lambda dx, dy: 1  # Costo constante
    else:
        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]  # Ortogonales + diagonales
        cost = lambda dx, dy: math.sqrt(dx**2 + dy**2)  # Costo real del movimiento

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    explored_cells = 0
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        explored_cells += 1
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            
            return path[::-1], explored_cells

        close_set.add(current)

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if not (0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]):
                continue

            if array[neighbor[0]][neighbor[1]] == 1:
                continue

            tentative_g_score = gscore[current] + cost(dx, dy)

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False


def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def pure_pursuit(current_x, current_y, current_heading, path,index, navigationControl):     ##Funcion que calcula las velocidades necesarias para seguir un path y llegar a un punto.
    global lookahead_distance
    closest_point = None              
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y) ##HAY QUE VARIAR LAS DISTANCIAS PARA VER QUE ES MEJOR!! (https://youtu.be/xqjVTE7QvOg?t=187)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)  #calcula el angulo al que debe apuntar el robot
        desired_steering_angle = target_heading - current_heading #el angulo que debe cambiar el robot para llegar a la siguiente posicion
        navigationControl.publish_marker(closest_point[0], closest_point[1])
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
        navigationControl.publish_marker(path[-1][0], path[-1][1])

    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

def bspline_planning(array):
    #hacemos que no haga nada para analizar el path puro sin suavizar.
    path = array
    """
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 3  # Usamos spline cúbica en lugar de cuadrática
        t = range(len(x))

        # Generamos las splines sin modificar coeficientes manualmente
        x_tup = si.splrep(t, x, k=N, s=0)
        y_tup = si.splrep(t, y, k=N, s=0)

        # Número de puntos interpolados más controlado
        ipl_t = np.linspace(0.0, len(x) - 1, len(x) * 2) #(antes len*5)
        rx = si.splev(ipl_t, x_tup)
        ry = si.splev(ipl_t, y_tup)

        path = [(rx[i], ry[i]) for i in range(len(rx))]
    except Exception as e:
        print("Error en B-Spline:", e)
        path = array  # Si falla, devolvemos el path original
    """
    return path


def aproximar_a_cero(valores, epsilon=1e-6):
  
    # Si valores es un número único
    if isinstance(valores, (int, float, np.float64)):
        return 0 if abs(valores) < epsilon else valores

    # Si valores es un iterable
    return [0 if abs(v) < epsilon else v for v in valores]
    
class navigationControl(Node):
    def __init__(self, heuristic):
        super().__init__('Navigation')
        self.heuristic = heuristic
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
        self.total_length = 0.0
        self.puntos_recorridos = []
        self.giros = 0
        self.explored_cells = 0

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


            #Conversion de cordenadas a indice del celdas
            #representan la celda en la que esta el robot
            column = int((self.x- originX)/resolution) 
            row = int((self.y- originY)/resolution) 
            
            #Conversion de las coordenadas del objetivo a indices del celdas
            columnH = int((self.goal[0]- originX)/resolution)
            rowH = int((self.goal[1]- originY)/resolution)


            #Para trabajar con el costmap, se crea una instancia con el nombre 'data'
            data = costmap(msg.data,msg.info.width,msg.info.height,resolution) 
            data[row][column] = 0 #Convierte Posicion del robot a 0
            data[data < 0] = 1 #Celdas desconocidas se convierten en obstáculos
            data[data > 5] = 1  #Celdas con ALTO COSTO se convierten en obstáculos

            self.get_logger().info('Generando un PATH con A*')
            
            self.path_0 = self.get_clock().now()
            
            path, self.explored_cells = astar(data,(row,column),(rowH,columnH), self.heuristic) #Busqueda de ruta con A*
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

    def printData(self, heuristic):
        self.get_logger().info(f'Guardando datos en el archivo')

        with open('tablaDatos.csv', mode="a", newline="") as file:  
            writer = csv.writer(file)

            # Escribir los datos de la simulación
            writer.writerow([
                f'{heuristic.__name__}',
                aproximar_a_cero(self.goal[0]),
                aproximar_a_cero(self.goal[1]),
                self.pathBuildTime, # tiempo que se calcula el path
                self.time_moverse, # tiempo que se mueve el robot
                self.timeGoal,  # desde que se recibe el goalpose hasta que llega al destino.
                self.total_length, # distancia del path generado
                self.giros, # número de giros del path
                self.explored_cells
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

                self.puntos_recorridos = []
                self.printData(self.heuristic)
                print("Objetivo alcanzado!!\nEsperando nuevo objetivo...")

            self.vel_publisher.publish(twist) 

    def info_callback(self,msg):  #Funcion que recibe parámetros de odometría suscribiendose a /odom
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        

