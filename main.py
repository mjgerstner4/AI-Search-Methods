import csv
import time
import math
from collections import deque, defaultdict
import heapq


def calculate_distance(coord1, coord2):
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    radius_of_earth = 6371
    distance = radius_of_earth * c

    return distance



def heuristic_func(node, goal):
    node_coords = coordinates_dict[node]
    goal_coords = coordinates_dict[goal]
    dx = node_coords[0] - goal_coords[0]
    dy = node_coords[1] - goal_coords[1]
    return math.sqrt(dx**2 + dy**2)



def blind_brute_force_search(graph, start, end, coordinates_dict):
    start_time = time.time()
    def generate_paths(path, current_node):
        if current_node == end:
            return [path + [current_node]]
        
        paths = []
        for neighbor in graph[current_node]:
            if neighbor not in path:
                new_path = path + [current_node]
                paths.extend(generate_paths(new_path, neighbor))
        
        return paths
    
    paths = generate_paths([], start)
    
    if not paths:
        print("No route found.")
    else:
        end_time = time.time()
        elapsed_time = end_time - start_time
        path = min(paths, key=lambda path: calculate_distance(coordinates_dict[path[0]], coordinates_dict[path[-1]]))
        total_distance = calculate_distance(coordinates_dict[path[0]], coordinates_dict[path[-1]])
        
        print("Route found:", path)
        print(f"Time elapsed: {elapsed_time}")
        print(f"Total distance: {total_distance}")



def bfs_search(graph, start, end, coordinates_dict):
    start_time = time.time()

    visited = set()
    queue = deque([[start]])
    path = None

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == end:
            break

        if node not in visited:
            for neighbor in graph[node]:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)

            visited.add(node)

    if path is None:
        print("No route found.")
    else:
        end_time = time.time()
        elapsed_time = end_time - start_time

        total_distance = 0
        for i in range(len(path) - 1):
            city1 = path[i]
            city2 = path[i + 1]
            distance = calculate_distance(coordinates_dict[city1], 
                                          coordinates_dict[city2])
            total_distance += distance

        print("Route found:", path)
        print(f"Time elapsed: {elapsed_time}")
        print(f"Total distance: {total_distance}")




def dfs_search(graph, start, end, coordinates_dict):
    start_time = time.time()

    visited = set()
    stack = [[start]]
    path = None
    node = None

    while stack:
        path = stack.pop()
        node = path[-1]

        if node == end:
            break

        if node not in visited:
            for neighbor in graph[node]:
                new_path = list(path)
                new_path.append(neighbor)
                stack.append(new_path)

            visited.add(node)

    end_time = time.time()
    elapsed_time = end_time - start_time

    if path and node == end:
        total_distance = 0
        for i in range(len(path) - 1):
            city1 = path[i]
            city2 = path[i + 1]
            distance = calculate_distance(coordinates_dict[city1], 
                                          coordinates_dict[city2])
            total_distance += distance

        print("Route found:", path)
        print(f"Time elapsed: {elapsed_time}")
        print(f"Total distance: {total_distance}")
    else:
        print("No route found.")



def id_dfs_search(graph, start, end, coordinates_dict):
    start_time = time.time()
    def depth_limited_dfs(node, depth_limit, current_depth, path):
        if current_depth > depth_limit:
            return None
        if node == end:
            return path + [node]
        
        for neighbor in graph[node]:
            if neighbor not in path:
                new_path = path + [node]
                result = depth_limited_dfs(neighbor, depth_limit, current_depth + 1, new_path)
                if result:
                    return result
        return None

    max_depth = 0
    while True:
        result = depth_limited_dfs(start, max_depth, 0, [])
        end_time = time.time()
        elapsed_time = end_time - start_time
        if result:
            total_distance = sum(
                calculate_distance(coordinates_dict[result[i]], coordinates_dict[result[i + 1]])
                for i in range(len(result) - 1)
            )
            print("Route found:", result)
            print(f"Time elapsed: {elapsed_time}")
            print(f"Total distance: {total_distance}")
            return
        max_depth += 1

        if max_depth > len(graph):
            print("No route found.")
            return



def best_first_search(graph, start, end, coordinates_dict, heuristic_func):
    start_time = time.time()
    visited = set()
    priority_queue = [(heuristic_func(start, end), start)]
    came_from = {}
    
    while priority_queue:
        end_time = time.time()
        elapsed_time = end_time - start_time
        _, current = heapq.heappop(priority_queue)
        if current == end:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            total_distance = sum(
                calculate_distance(coordinates_dict[path[i]], coordinates_dict[path[i + 1]])
                for i in range(len(path) - 1)
            )
            print("Route found:", path)
            print(f"Time elapsed: {elapsed_time}")
            print(f"Total distance: {total_distance}")
            return
        
        visited.add(current)
        
        for neighbor in graph[current]:
            if neighbor not in visited:
                heapq.heappush(priority_queue, (heuristic_func(neighbor, end), neighbor))
                came_from[neighbor] = current
    
    print("No route found.")




def a_star_search(graph, start, end, coordinates_dict, heuristic_func):
    start_time = time.time()
    visited = set()
    priority_queue = [(0, start)]
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    came_from = {}
    
    while priority_queue:
        end_time = time.time()
        elapsed_time = end_time - start_time
        _, current = heapq.heappop(priority_queue)
        if current == end:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            total_distance = sum(
                calculate_distance(coordinates_dict[path[i]], coordinates_dict[path[i + 1]])
                for i in range(len(path) - 1)
            )
            print("Route found:", path)
            print(f"Time elapsed: {elapsed_time}")
            print(f"Total distance: {total_distance}")
            return
        
        visited.add(current)
        
        for neighbor in graph[current]:
            if neighbor in visited:
                continue
            
            tentative_g_score = g_score[current] + calculate_distance(coordinates_dict[current], coordinates_dict[neighbor])
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic_func(neighbor, end)
                heapq.heappush(priority_queue, (f_score, neighbor))
    
    print("No route found.")



def load_adjacencies(filename):
    adjacency_data = defaultdict(list)
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            city_names = line.split()
            if len(city_names) != 2:
                print(f"Invalid data format in line: {line}. Skipping.")
                continue
            city1, city2 = city_names
            adjacency_data[city1].append(city2)
            adjacency_data[city2].append(city1)
    return adjacency_data


  
def load_coordinates(filename):
    coordinates_data = {}
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            city, latitude, longitude = row
            coordinates_data[city] = (float(latitude), float(longitude))
    return coordinates_data



adjacency_file = 'Adjacencies.txt'
coordinates_file = 'coordinates.csv'
adjacency_graph = load_adjacencies(adjacency_file)
coordinates_dict = load_coordinates(coordinates_file)



while True:
    start_city = input("Enter starting town: ")
    end_city = input("Enter ending town: ")
    search_method = input("Select search method (BFS/DFS/BLIND/ID-DFS/BEST-FIRST/A-STAR): ")

    if start_city not in coordinates_dict or end_city not in coordinates_dict:
        print("Invalid city names. Please choose cities from the database.")
        continue

    if search_method.upper() == "BFS":
        route = bfs_search(adjacency_graph, start_city, end_city, coordinates_dict)
    elif search_method.upper() == "DFS":
        route = dfs_search(adjacency_graph, start_city, end_city, coordinates_dict)
    elif search_method.upper() == "BLIND":
        route = blind_brute_force_search(adjacency_graph, start_city, end_city, coordinates_dict)
    elif search_method.upper() == "ID-DFS":
        route = id_dfs_search(adjacency_graph, start_city, end_city, coordinates_dict)
    elif search_method.upper() == "BEST-FIRST":
        route = best_first_search(adjacency_graph, start_city, end_city, coordinates_dict, heuristic_func)
    elif search_method.upper() == "A-STAR":
        route = a_star_search(adjacency_graph, start_city, end_city, coordinates_dict, heuristic_func)
    else:
        print("Invalid search method. Please choose BFS or DFS.")
        continue

    another_search = input("Do you want to try another search method? (yes/no): ")
    if another_search.lower() != "yes":
        break