from collections import deque
from queue import Queue, PriorityQueue

################ Global helper functions ################

# Return True if an edge (u,v) is valid, meaning it is not a self-loop
def valid_edge(k_u, k_v):

    # If (u,v) is a self-loop, return False
    if k_u == k_v: 
        return False
    return True
    
################ Vertex Class ################
# Initialize with key, value, neighbors dictionary with keys as neighbor keys and values as edge weights (default empty)
# Attributes: key, value, neighbors ({k_v: weight of edge between them})
class Vertex:
    def __init__(self, key, value, neighbors=None):
        self.key = key
        self.value = value
        self.neighbors = {} if neighbors is None else neighbors

        # Remove self-loops
        if key in self.neighbors:
            del self.neighbors[key]
            print("Ignoring self-loop neighbor")

    # Self-representation function    
    def __repr__(self):
        return f"Vertex({self.key}, {self.value}, Neighbors: {self.neighbors})"

    def add_neighbor(self, v, weight=0):
        self.neighbors[v] = weight  

    def get_key(self):
        return self.key 
    
    def get_value(self):
        return self.value 
    
    def get_neighbors(self):
        return list(self.neighbors.items())

################ Graph Class ################
# Initialize with nothing
# Attributes: vertices (empty dictionary where keys are vertex objects and values are list of neighbors which are key and weight)
# Represents an empty simple, undirected, weighted graph
class Graph:

    def __init__(self):
        # initialize set of vertices as a dictionary
        self.vertices = {}

    def get_weight(self,k_u,k_v):
        # check if edge exists
        if k_u in self.vertices and k_v in self.vertices[k_u].neighbors:
            # return weight
            return self.vertices[k_u].neighbors[k_v]
        return None

    def add_vertex(self,k,v,neighbors=[]):        
        # check that key not already used
        if k not in self.vertices:
            new_vertex = Vertex(k, v)
            self.vertices[k] = new_vertex

            for neighbor_key, weight in neighbors:
                if neighbor_key in self.vertices:
                    self.add_edge(k, neighbor_key, weight)
                else:
                    print(f"Neighbor {neighbor_key} does not exist. Ignoring the edge.")
        else: 
            print(str(k)+ " is already used as a key")


    def delete_vertex(self,k):
        if k in self.vertices:
            del self.vertices[k]
            for _, vertex in self.vertices.items():
                if k in vertex.neighbors:
                    del vertex.neighbors[k]

    def add_edge(self,k_u,k_v,w):     
        if k_u in self.vertices and k_v in self.vertices:
            # prevent self loops
            if k_u != k_v:
                self.vertices[k_u].add_neighbor(k_v, w)
                self.vertices[k_v].add_neighbor(k_u, w)
            else:
                print("Warning! You tried to add a self loop")

    def delete_edge(self,k_u,k_v):
        # check edge exists
        # remove edge from G.E
        if k_u in self.vertices and k_v in self.vertices[k_u].neighbors:
            del self.vertices[k_u].neighbors[k_v]
            del self.vertices[k_v].neighbors[k_u]
        
################ Map Class ################
# Initialize with dimension n and (optional) set of obstacles
# Attributes: n (size), obstacles (set of (x,y) coordinates)
# Represents an grid map where each point is a vertex
class Map(Graph):
    def __init__(self, n, obstacles=set()):
        super().__init__()
        self.n = n  # The size of the map (n x n)
        self.obstacles = obstacles # A set for faster lookup
        self.create_map()

    def create_map(self):
        # Create vertices for each cell in the grid
        for y in range(self.n):
            for x in range(self.n):
                if (x, y) not in self.obstacles:
                    # Create a vertex for each non-obstacle position
                    self.add_vertex((x, y), None, [])
        
        # Add edges between adjacent vertices
        for y in range(self.n):
            for x in range(self.n):
                if (x, y) not in self.obstacles:
                    # Possible neighbors are to the left, right, up, and down
                    neighbors = [
                        ((x - 1, y), 1),  # Left
                        ((x + 1, y), 1),  # Right
                        ((x, y - 1), 1),  # Down
                        ((x, y + 1), 1),  # Up
                    ]
                    # Filter out neighbors that are not valid (out of bounds or obstacles)
                    valid_neighbors = [
                        (nx, w) for (nx, w) in neighbors 
                        if 0 <= nx[0] < self.n and 0 <= nx[1] < self.n and nx not in self.obstacles
                    ]
                    # Add each valid neighbor as an edge
                    for neighbor, weight in valid_neighbors:
                        self.add_edge((x, y), neighbor, weight)
    def add_obstacle(self, x, y):
        # If the position is within the grid and not already an obstacle
        if 0 <= x < self.n and 0 <= y < self.n:
            # Add to the obstacles set
            self.obstacles.add((x, y))
            # Remove the vertex from the graph, which also removes its edges
            self.delete_vertex((x, y))

    def remove_obstacle(self, x, y):
        if (x, y) in self.obstacles:
            self.obstacles.remove((x, y))
            
            # Define potential neighbors based on map directions
            potential_neighbors = [
                ((x - 1, y), 1),  # Left
                ((x + 1, y), 1),  # Right
                ((x, y - 1), 1),  # Down
                ((x, y + 1), 1),  # Up
            ]
            
            # Filter out neighbors that are not valid (out of bounds or obstacles)
            valid_neighbors = [
                (nx, w) for (nx, w) in potential_neighbors 
                if 0 <= nx[0] < self.n and 0 <= nx[1] < self.n and nx not in self.obstacles
            ]
            
            # Add the vertex back with its valid neighbors
            self.add_vertex((x, y), None, valid_neighbors)

    def has_obstacle(self, x, y):
        # Check if the (x, y) is in the obstacles set
        return (x, y) in self.obstacles
        
    def display_map(self):
        for y in range(self.n):
            for x in range(self.n):
                if (x, y) in self.obstacles:
                    print('#', end=' ')
                else:
                    print('.', end=' ')
            print()  # New line after each row

    
########################################################################################
    ## PARTS B and C: Pathfinding algorithms: 
########################################################################################

    ################ bfs_spsp() ################
    # bfs_(spsp) takes as input a start point (a,b) and end point (y,z). Uses BFS to find a shortest path between those two points. 
    # Returns a data structure representing the shortest path.
    def bfs_spsp(self, start, end):
        # Check if start and end points are valid vertices in the graph
        if start not in self.vertices or end not in self.vertices:
            print("Start or end point is not in the map.")
            return None
        # Initialize data structures for BFS traversal
        visited = set()  # Set to keep track of visited vertices
        queue = Queue()  # Queue to store vertices for BFS traversal
        parent = {}      # Dictionary to store parent information for reconstructing the path
        # Start BFS traversal from the 'start' vertex
        queue.put(start)
        visited.add(start)
        while not queue.empty():
            # Get the current vertex from the queue
            current_vertex = queue.get()
            # Check if we have reached the destination 'end' vertex
            if current_vertex == end:
                # Reconstruct the path from 'start' to 'end'
                path = [end]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                path.reverse()
                return path
            # Explore neighbors of the current vertex
            for neighbor, _ in self.vertices[current_vertex].get_neighbors():
                if neighbor not in visited:
                    # Mark the neighbor as visited, enqueue it, and update its parent
                    visited.add(neighbor)
                    queue.put(neighbor)
                    parent[neighbor] = current_vertex
        # If no path is found between the given points
        print("No path found between the given points.")
        return None
        
    # print_shortest_path() method takes the output of the BFS-SPSP algorithm and prints a grid showing the distances only along the path.
    def print_shortest_path_spsp(self, path):
        if not path:
            print("No valid path to display.")
            return
        distances = {vertex: index for index, vertex in enumerate(path)}
        for y in range(self.n):
            for x in range(self.n):
                current = (x, y)
                if current in self.obstacles:
                    print('#', end=' ')
                elif current in distances:
                    print(distances[current], end=' ')
                else:
                    print('.', end=' ')
            print()  # New line after each row

    ################ bfs_sssp() ################
    # bfs_sssp takes as input a starting point (x,y) within the map.
    # Uses BFS to compute the shortest path distances from (x,y) to all points in the map. 
    # returns a data structure that maps each point in the map to its shortest-path distance 
    # and its parent point (i.e., its predecessor along some shortest path).
    def bfs_sssp(self, start_point):
        # Check if the start_point is a valid point in the map
        if start_point not in self.vertices:
            print("Invalid start point. Please provide a valid point within the map.")
            return None

        # Initialize data structures for distances and parents
        distances = {point: float('inf') for point in self.vertices}
        parents = {point: None for point in self.vertices}

        # Initialize the queue for BFS
        queue = Queue()
        start_key = start_point
        distances[start_key] = 0  # Distance from the start to itself is 0
        queue.put(start_key)

        # Perform BFS to compute shortest path distances and parents
        while not queue.empty():
            current_key = queue.get()

            # Iterate through neighbors of the current point
            for neighbor_key, weight in self.vertices[current_key].get_neighbors():
                if distances[neighbor_key] == float('inf'):
                    # If the neighbor has not been visited
                    distances[neighbor_key] = distances[current_key] + weight
                    parents[neighbor_key] = current_key
                    queue.put(neighbor_key)

        # Return a data structure mapping each point to its shortest-path distance and parent
        shortest_path_info = {point: {'distance': distances[point], 'parent': parents[point]} for point in distances}
        return shortest_path_info
    
    # print_shortest_path() method takes the output of an SSSP algorithm
    # returns a grid showing the distances between the start node and all other nodes.
    def print_shortest_path_sssp(self, sssp_result):
        if not sssp_result:
            print("No result to print. Run BFS SSSP first.")
            return

        # Extract relevant information from the SSSP result
        distances = {point: sssp_result[point]['distance'] for point in sssp_result}

        # Print the grid of shortest path distances
        for y in range(self.n):
            for x in range(self.n):
                current_point = (x, y)
                if current_point in distances:
                    distance = distances[current_point]
                    print(f'{distance}', end=' ')
                else:
                    print('#', end=' ')  # Use 'X' to denote the starting point
            print()  # New line after each row


    ################ dij_sssp() ################
    # requires non neg weights (check?)
    # start is a (x,y) coord tuples
    def dij_sssp(self, start):
        # Check if the start is a valid point in the map
        if start not in self.vertices:
            print("Invalid start point. Please provide a valid point within the map.")
            return None

        # Initialize data structures for distances and parents
        distances = {point: float('inf') for point in self.vertices}
        parents = {point: None for point in self.vertices}

        # Use PriorityQueue for efficient retrieval of the minimum distance
        queue = PriorityQueue()

        # Initialize distances and insert the start point into the priority queue
        distances[start] = 0
        queue.put((0, start))
        # Initialize the visited set
        visited = set()

        while not queue.empty():
            current_distance, current_point = queue.get()

            # If the current point has been visited, skip it
            if current_point in visited:
                continue

            visited.add(current_point)

            # Iterate through neighbors of the current point
            for neighbor_point, weight in self.vertices[current_point].get_neighbors():
                new_distance = distances[current_point] + weight

                if new_distance < distances[neighbor_point]:
                    distances[neighbor_point] = new_distance
                    parents[neighbor_point] = current_point

                    # Insert the updated distance into the priority queue
                    queue.put((new_distance, neighbor_point))

        # Return a data structure mapping each point to its shortest-path distance and parent
        shortest_path_info = {point: {'distance': distances[point], 'parent': parents[point]} for point in distances}
        return shortest_path_info
        
    # Print function takes the output of Dijkstra and prints a grid showing all the shortest paths distances.
    def print_dij_sssp(self, shortest_path_info):
        if not shortest_path_info:
            print("No valid shortest path information to display.")
            return

        # Initialize an empty print output
        output = ""
        for y in range(self.n):
            for x in range(self.n):
                current = (x, y)
                if current in self.obstacles:
                    output += "# "
                elif current in shortest_path_info:
                    distance = shortest_path_info[current]['distance']
                    output += str(distance) + ' ' if distance != float('inf') else 'Inf '
                else:
                    output += '. '
            output += '\n'
        print(output)
    

    ################ IMPROVEMENTS ################

    # Large Improvement: Modifed Dijkstra to exit early upon reaching a designated goal tile. 
    # (In other words, converts Dijkstra into a *single-pair* shortest paths algorithm.)
    def dij_spsp(self, start, goal):
        # Check if the start and goal are valid points in the map
        if start not in self.vertices or goal not in self.vertices:
            print("Invalid start or goal point. Please provide valid points within the map.")
            return None, None, 0  # Return None for path and distance, 0 for visited_count

        visited_count = 0  # Counter for visited vertices

        # Initialize data structures for distances and parents
        distances = {point: float('inf') for point in self.vertices}
        parents = {point: None for point in self.vertices}
        distances[start] = 0  # Distance from the start to itself is 0

        # Initialize a priority queue for vertices
        pq = PriorityQueue()
        pq.put((0, start))  # Push the start vertex with priority 0
        visit_order = []  # List to keep track of the visit order

        # Perform Dijkstra's algorithm
        while not pq.empty():
            current_distance, current_vertex = pq.get()
            visit_order.append(current_vertex)
            visited_count += 1  # Increment visited counter

            # Early exit if the goal is reached
            if current_vertex == goal:
                break

            # Iterate through neighbors of the current point
            for neighbor_key, weight in self.vertices[current_vertex].get_neighbors():
                distance_through_vertex = current_distance + weight

                # Relaxation step
                if distance_through_vertex < distances[neighbor_key]:
                    distances[neighbor_key] = distance_through_vertex
                    parents[neighbor_key] = current_vertex
                    pq.put((distance_through_vertex, neighbor_key))

        # Reconstruct the shortest path from start to goal if goal was reached
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parents[current]
        path.reverse()

        # Return the shortest path and distance if the goal was reached
        if path and path[0] == start:
            return path, distances[goal], visit_order
        else:
            print("No path found to the goal.")
            return None, None, visited_count

    # print_shortest_path() method takes the output of an SPSP algorithm and returns a grid showing the distances only along the path.
    def print_helper(self, path):
        if not path:
            print("No valid path to display.")
            return
        distances = {vertex: index for index, vertex in enumerate(path)}
        # Initialize an empty print output (this makes testing easier)
        output = ""
        for y in range(self.n):
            for x in range(self.n):
                current = (x, y)
                if current in self.obstacles:
                    output += "# "
                elif current in distances:
                    output += str(distances[current]) + ' '
                else:
                    output += '. '
            output += '\n'
        print(output)

    # Print the output of print_helper()
    def print_dij_spsp(self, path):
        self.print_helper(path)


    # Large and small improvements: WFS related methods
    def wfs_search(self, start, h, *h_args):
        """
        Perform Whatever-First Search.
        start: The starting vertex.
        h: Heuristic function that takes a map, a vertex, and additional arguments.
        h_args: Additional (optional) arguments for the heuristic function.
        Returns list of vertices in the order they were visited.
        """
        # Check if the start is a valid point in the map
        if start not in self.vertices:
            return []  # Return an empty list for an invalid start point

        # Initialize the priority queue with the start vertex
        pq = PriorityQueue()
        pq.put((h(self, start, *h_args), start))

        # Set to keep track of visited vertices
        visited = set()

        # List to store the order of visited vertices
        visit_order = []

        while not pq.empty():
            # Extract vertex with the lowest heuristic value
            _, current_vertex = pq.get()

            # Skip if already visited
            if current_vertex in visited:
                continue

            # Mark current vertex as visited
            visited.add(current_vertex)
            visit_order.append(current_vertex)

            # Add neighbors to the priority queue
            for neighbor_key, _ in self.vertices[current_vertex].get_neighbors():
                if neighbor_key not in visited:
                    pq.put((h(self, neighbor_key, *h_args), neighbor_key))

        return visit_order

    def print_wfs_path(self, visit_order):
        """
        Print the map showing the visit order from the WFS algorithm.
        visit_order: A list of vertices in the order they were visited by WFS.
        """
        if not visit_order:
            print("No visit order provided or the visit order is empty.")
            return

        # Create a grid initialized with '.', representing unvisited locations
        grid = [['.' for _ in range(self.n)] for _ in range(self.n)]

        # Mark obstacles with '#'
        for (x, y) in self.obstacles:
            grid[y][x] = '#'

        # Mark the visit order
        for index, (x, y) in enumerate(visit_order):
            if grid[y][x] != '#':  # Do not overwrite obstacles
                grid[y][x] = str(index % 10)  # Use modulo 10 to keep single digits

        # Print the grid
        for row in grid:
            print(' '.join(row))


    def wfs_heuristic(map_instance, current_vertex, goal, g_costs):
        # Calculate H (Manhattan Distance) to the goal
        h = abs(current_vertex[0] - goal[0]) + abs(current_vertex[1] - goal[1])
        # G cost from the g_costs dictionary
        g = g_costs[current_vertex]
        # Total cost
        f = g + h
        return f


    def print_algorithm_visit_order(self, algorithm, *args):
        """
        Execute the given algorithm and print the visit order on the map.
        algorithm: The name of the algorithm ('dij_spsp' or 'wfs_search').
        args: Arguments required by the algorithm.
        """
        # Execute the algorithm and get the visit order
        if algorithm == 'dij_spsp':
            _, _, visit_order = self.dij_spsp(*args)
        elif algorithm == 'wfs_search':
            visit_order = self.wfs_search(*args)
        else:
            print("Invalid algorithm name.")
            return

        # Print the visit order
        # Initialize a grid with '.' to represent unvisited vertices
        grid = [['.' for _ in range(self.n)] for _ in range(self.n)]

        # Mark obstacles with '#'
        for (x, y) in self.obstacles:
            grid[y][x] = '#'

        # Iterate through the visit_order and mark each vertex with its order
        for index, (x, y) in enumerate(visit_order):
            if grid[y][x] != '#':
                grid[y][x] = str(index % 10)  # Mod 10 for single-digit representation!!!

        # Print the grid
        for row in grid:
            print(' '.join(row))
