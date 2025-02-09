import heapq

def dijkstra(array, start, goal, heuristic):
    # Lista de direcciones posibles (vecinos)
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    
    # Conjuntos y diccionarios necesarios para el algoritmo
    closed_set = set()  # Conjunto de nodos procesados
    came_from = {}      # Diccionario para rastrear el camino
    gscore = {start: 0} # Diccionario para los costos g (costos de desplazamiento)
    open_list = []      # Lista abierta (prioridad por el costo g)
    
    # Insertamos el nodo inicial con su costo g
    heapq.heappush(open_list, (gscore[start], start))
    
    while open_list:
        # Extraemos el nodo con el menor costo g
        current = heapq.heappop(open_list)[1]
        
        # Si llegamos al objetivo, reconstruimos el camino
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path = path + [start]
            path = path[::-1]  # Invertimos el camino
            return path
        
        closed_set.add(current)  # Marcamos el nodo como procesado
        
        # Iteramos sobre los vecinos del nodo actual
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            
            # Verificamos que el vecino esté dentro de los límites del array
            if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                if array[neighbor[0]][neighbor[1]] == 1:
                    continue  # Si el vecino es un obstáculo, lo saltamos
            else:
                continue  # Si el vecino está fuera de los límites del mapa, lo saltamos
            
            # Si el vecino ya está en closed_set, verificamos si el nuevo camino es más corto
            if neighbor in closed_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            
            # Si el vecino no está en open_list o el camino es mejor, actualizamos
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_list]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                
                # Insertamos el vecino en open_list con el costo g actualizado
                heapq.heappush(open_list, (gscore[neighbor], neighbor))

    # Si no se encontró un camino, buscamos el nodo más cercano al objetivo
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in closed_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            path = []
            while closest_node in came_from:
                path.append(closest_node)
                closest_node = came_from[closest_node]
            path = path + [start]
            path = path[::-1]
            return path
    
    return False





def greedy(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
    ''' 
    Performs Greedy shortest path algorithm search on a costmap with a given start and goal node
    '''

    # Initialize open_list with the start node and its heuristic cost
    open_list = []
    closed_list = set()
    parents = dict()
    h_costs = dict()

    start_cost = euclidean_distance(start_index, goal_index, width)
    h_costs[start_index] = start_cost
    open_list.append([start_index, start_cost])

    shortest_path = []
    path_found = False
    rospy.loginfo('Greedy: Done with initialization')

    # Main loop: process nodes until the open_list is empty
    while open_list:

        # Sort open_list based on the lowest heuristic cost
        open_list.sort(key=lambda x: x[1])
        current_node = open_list.pop(0)[0]

        # Mark the current node as processed
        closed_list.add(current_node)

        # Optional: visualize closed nodes
        grid_viz.set_color(current_node, "pale yellow")

        # Check if we reached the goal node
        if current_node == goal_index:
            path_found = True
            break

        # Get neighbors of the current node
        neighbors = find_neighbors(current_node, width, height, costmap, resolution)

        for neighbor_index, step_cost in neighbors:

            # Skip already processed neighbors
            if neighbor_index in closed_list:
                continue

            # Calculate the heuristic cost for the neighbor
            h_cost = euclidean_distance(neighbor_index, goal_index, width)

            # Check if the neighbor is already in the open_list
            in_open_list = False
            for idx, element in enumerate(open_list):
                if element[0] == neighbor_index:
                    in_open_list = True
                    break

            # CASE 1: Neighbor already in open_list
            if in_open_list:
                if h_cost < h_costs[neighbor_index]:
                    # Update h_cost and parent
                    h_costs[neighbor_index] = h_cost
                    parents[neighbor_index] = current_node
                    # Update open_list with new h_cost
                    open_list[idx] = [neighbor_index, h_cost]

            # CASE 2: Neighbor not in open_list
            else:
                h_costs[neighbor_index] = h_cost
                parents[neighbor_index] = current_node
                open_list.append([neighbor_index, h_cost])

                # Optional: visualize frontier nodes
                grid_viz.set_color(neighbor_index, 'orange')

    rospy.loginfo('Greedy: Done traversing nodes in open_list')

    if not path_found:
        rospy.logwarn('Greedy: No path found!')
        return shortest_path

    # Reconstruct the path by working backwards from the goal
    node = goal_index
    shortest_path.append(goal_index)
    while node != start_index:
        node = parents[node]
        shortest_path.append(node)

    # Reverse the path to get it from start to goal
    shortest_path = shortest_path[::-1]
    rospy.loginfo('Greedy: Done reconstructing path')

    return shortest_path, None





#-------------------------------------------------------------------------------------
#MAnhattan:
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



#--------------------------------------------------------
def jump_point_search(array, start, goal, heuristic):
    def jump(current, direction):
        x, y = current
        dx, dy = direction

        while True:
            x += dx
            y += dy

            if not (0 <= x < array.shape[0] and 0 <= y < array.shape[1]) or array[x, y] == 1:
                return None

            if (x, y) == goal:
                return (x, y)

            if dx != 0 and dy != 0:
                if (array[x - dx, y] == 1 and array[x - dx, y + dy] != 1) or \
                   (array[x, y - dy] == 1 and array[x + dx, y - dy] != 1):
                    return (x, y)
                if jump((x, y), (dx, 0)) or jump((x, y), (0, dy)):
                    return (x, y)
            else:
                if dx != 0:
                    if (array[x, y - 1] == 1 and array[x + dx, y - 1] != 1) or \
                       (array[x, y + 1] == 1 and array[x + dx, y + 1] != 1):
                        return (x, y)
                else:
                    if (array[x - 1, y] == 1 and array[x - 1, y + dy] != 1) or \
                       (array[x + 1, y] == 1 and array[x + 1, y + dy] != 1):
                        return (x, y)

    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for direction in [(1, 0), (0, 1), (-1, 0), (0, -1)]:  # Solo ortogonales
            jump_point = jump(current, direction)
            if jump_point:
                new_cost = gscore[current] + heuristic(current, jump_point)
                if jump_point not in gscore or new_cost < gscore[jump_point]:
                    gscore[jump_point] = new_cost
                    fscore[jump_point] = new_cost + heuristic(jump_point, goal)
                    heapq.heappush(open_list, (fscore[jump_point], jump_point))
                    came_from[jump_point] = current

    return []
