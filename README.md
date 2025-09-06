# Project 2

Group members: Aliya Ghassaei, Aidan Mokalla, Mei Tate 

## Part A - Graphs and Maps
<details>
<summary> We have implemented three classes in this code: Vertex, Graph, and Map. </summary>

### `Graph` and `Vertex` object
A simple, undirected, weighted graph is defined as $G = (E, V)$ where $V$ is the set of vertices and $E$ is the set of edges.
Formally, $E = \{(u,v,w) ~|~ u \neq v\}$ and $w$ represents the weight of this edge.
We have implemented vertices using the class `Vertex` which is initialized with a key, value, and 
optional list of neighbors. A `Vertex` object has the following attributes:

| Attribute | Description |
| --- | ----------- |
| `self.key`| the key of the vertex |
| `self.value` | the value of the vertex |
| `self.neighbors` | a list of vertices that have an edge connecting to this vertex |

We have impemented graphs using a `Graph` class. A `Graph` is implemented with an empty dictionary
of vertices `self.vertices`. Below is a table of the `Graph` methods and their descriptions:


| Method | Description |
| --- | ----------- |
| `get_weight(self, k_u, k_v)` | returns the weight of the edge $(u,v)$ if it exists |
| `add_vertex(self, k, v, neighbors)`| adds a vertex to the graph and updates its neighbors (if there are any) |
| `delete_verex(self, k)` | removes a vertex from the graph and any edges associated with it |
| `add_edge(self, k_u, k_v, w)` | adds the edge $(u,v,w)$|
| `delete_edge(self, k_u, k_v, w)` | remove an edge if it exists |



The graph class is implemented using an **adjacency list** representation. In an adjacency list, 
* each vertex in the graph is associated with a list of its neighbors. 
* The neighbors attribute in the Vertex class serves as an adjacency list for each vertex. 
* The add_edge method updates these lists to represent the connections between vertices. 

In comparison, an adjacency matrix representation is used to represent all possible edges between vertices. The matrix has dimensions V x V (where V is the number of vertices), and the presence or absence of an edge between vertices is indicated by the entries in the matrix. This approach is generally less space-efficient for sparse graphs (graphs with fewer edges), but it allows for constant-time lookups of edge existence.

In the provided code, the use of a dictionary (neighbors) within each Vertex instance allows for an efficient representation of the graph since the Map class, which will utilize the Graph class, it isn't going to need large complex graphs. Each vertex only stores information about its immediate neighbors, and the add_edge method efficiently updates these lists.

### `Map` object
A `Map` is specific type of `Graph`. In addition to all of the `Graph` methods, `Map` also has access to the following functions:


| Method | Description |
| --- | ----------- |
| `create_map(self)` | creates vertices between cells in the grid and connects adjacent vertices|
| `display_map(self)` | prints a map with the characters listed below |
| `add_obstacle(self,x,y)` | adds an obstacle to the map |
| `remove_obstacle(self,x,y)` | removes an from the map |
| `is_obstacle(self,x,y)` | returns `True` if a coordinate is an obstacle |

| Charater Key | Symbol |
| ------------ | ------ |
| Open Spaces  | .      |
| Obsticles    | #      | 

Here's an example of how you could use the Map class to create a map and display it:
```sh
m = Map(5)
m.display_map()
```
This will create the following 5x5 map that has no obsticles and display it as a grid of dots (represented by '.'). 
```sh
    . . . . .
    . . . . .
    . . . . .
    . . . . .
    . . . . .
```
We can add an obstacle (represented by the symbol '#') at position (2, 3) in our map via:
```sh
map.add_obstacle(2, 3)
```
Which when m.display_map() is called will print:
```sh
    . . . . .
    . . . . .
    . . . . .
    . . # . .
    . . . . .
```
</details>



## Part B - Breadth-First Search
<details>
<summary> In this part we implemented 4 new methods to the Map class: bfs_sssp(), print_shortest_path_sssp(), bfs_spsp() and print_shortest_path_spsp(). </summary>

A tutorial/demo that shows off the 4 functions:
- BFS Single Source Shortest Paths (BFS-SSSP): `bfs_sssp()` takes as input a starting point (x,y) within the map and uses BFS to compute the shortest path distances from (x,y) to all points in the map. The method returns a data structure that maps each point in the map to its shortest-path distance and its parent point (i.e., its predecessor along some shortest path).
- Display output of BFS-SSSP: `print_shortest_paths_sssp()` method takes the output of the SSSP algorithm and prints a grid showing all the shortest paths distances.
    Example Usage:
```sh
    # initialize our Map with obstacles 
    map = Map(6, [(1, 1), (2, 2), (3, 3)])
    # choose a starting point on our map 
    start_point = (0,1)
    # preform bfs_sssp() on our map using our chosen starting point 
    path = map.bfs_sssp(start_point)
    # call our print path function
    map.print_shortest_path_sssp(path)
```
This will print:
```sh
1 2 3 4 5 6
0 # 4 5 6 7
1 2 # 6 7 8 
2 3 4 # 8 9
3 4 5 6 7 8
4 5 6 7 8 9
```
 
- BFS Single Pair Shortest Paths (BFS-SPSP): `bfs_spsp()` takes as input a start point (a,b) and end point (y,z) and uses BFS to find a shortest path between those two points. It returns a data structure representing the shortest path. It's a method of the Map class. 
- Display output of BFS-SPSP: `print_shortest_path_spsp()` takes the output of the BFS-SPSP algorithm and prints a grid showing the distances only along the path. It's a method of the Map class. 
    Example usage:
```sh
    # initialize our Map with obstacles 
    my_map = Map(5, [(1, 1), (2, 2), (3, 3)])
    # choose a starting point and ending point on our map 
    start_point = (0, 0)
    end_point = (2, 3)
    # preform bfs_spsp() on our map using our chosen points
    shortest_path = my_map.bfs_spsp(start_point, end_point)
    # call our print path function and print the path
    my_map.print_shortest_path_spsp(shortest_path)
    print("Shortest Path:", shortest_path)
```
This will print:
```sh
0 . . . .
1 # . . .
2 3 # . .
. 4 5 # .
. . . . .
```
```sh
Shortest Path: [(0, 0), (0, 1), (0, 2), (1, 2), (1, 3), (2, 3)]
```
</details>


## Part C – Pathfinding algorithms
<details>
<summary> In this part we implemented 2 new methods to the Map class: Dijkstra SSSP and a Dijkstra print function. </summary>

- Dijkstra SSSP: `dij_sssp()` takes as input a map and a starting (“source”) tile. Uses an updateable priority queue instead of a (plain) queue to determine which vertex to explore next. The priority queue is weighted so that vertex v has weight equal to the current distance estimate d(s,v)
- Dijkstra print function: `print_shortest_path_dij()` is a print function that takes the output of Dijkstra and prints a grid showing all the shortest paths distances.
    Example usage:
```sh
map_instance = Map(5, [(1, 1), (2, 2), (3, 3)])
start_point = (0, 0)
shortest_path_info = map_instance.dij_sssp(start_point)
map_instance.print_dij_sssp(shortest_path_info)
```
This will print:
```sh
0 1 2 3 4 
1 # 3 4 5
2 3 # 5 6
3 4 5 # 7
4 5 6 7 8
```

</details>

## Part C – Improvements
<details>
<summary> This section contains explanations of the bonus improvements we implimented. </summary>

The code for the improvements is located in the `improvements.py` file.

- Small improvement `Difficult_Terrain_Map`: This class contains regular tiles, obstacles, and tiles that represent terrain that is more difficult to traverse. It's possible for the user to input variables reg_cost and high_cost to determine the cost of traversing regular and difficult terrain, respectively. In this type of map, traveling from tile A to tile B (where A and B are non-obstacles) costs high_cost if either tile is difficult terrain; otherwise (if both A and B are regular terrain) it costs reg_cost. This class also makes use of the Graph class from Part A.
Example useage:
``` sh
    # Create a map with regular cost as 1 and high cost as 3
    my_map = Difficult_Terrain_Map(reg_cost=1, high_cost=3)

    Add tiles to the map
    my_map.add_tile("A", "regular")
    my_map.add_tile("B", "difficult_terrain")
    my_map.add_tile("C", "regular")
    my_map.add_tile("D", "obstacle")
    my_map.add_tile("E", "difficult_terrain")

    # Connect tiles with appropriate costs
    my_map.connect_tiles("A", "B")
    my_map.connect_tiles("B", "C")
    my_map.connect_tiles("C", "D")
    my_map.connect_tiles("C", "E")

    # Get the cost of traveling between tiles
    cost_AB = my_map.get_cost("A", "B")
    cost_BC = my_map.get_cost("B", "C")
    cost_CD = my_map.get_cost("C", "D")
    cost_CE = my_map.get_cost("C", "E")

    print(f"Cost from A to B: {cost_AB}")
    print(f"Cost from B to C: {cost_BC}")
    print(f"Cost from C to D: {cost_CD}")
    print(f"Cost from C to E: {cost_CE}")
```
This will print:
``` sh
Cannot connect tiles with obstacles.
Cost from A to B: 3
Cost from B to C: 3
Cost from C to D: None
Cost from C to E: 3
```

- Small improvement `ElevatedMap`: We implemented a subclass of `Map` which encodes an elevation for each vertex, encoded in the dictionary `self.elevation` where keys are vertex keys and values are the corresponding elevation. We consider the traversal of an edge `(u, v)` to be "uphill" if the elevation of `u` is less than that of `v`, and "downhill" if the opposite is true. - `Elevated Map` takes an "uphill cost" and "downhill cost" which we add to the weight of edge `(u, v)` at the time that we call the parent method `self.get_weight(k_u, k_v)`. Below, we demonstrate how we initialize an `ElevatedMap`, add vertices and edges, and get the edge weights with and without accounting for elevation:

```
>>> # Initialize a 4x4 map with an uphill cost of 3 and a downhill cost of -4
>>> eMap = ElevatedMap(4, 3, -4)

>>> # Display map information
>>> eMap.display_map_info()
Map size: 4
Obstacles: set()
Vertices and elevations: {}

>>> # Add vertices with key, value, and elevation
>>> eMap.add_vertex_elevatedMap("u", 1, 5)
>>> eMap.add_vertex_elevatedMap("v", 2, 0)
>>> eMap.add_vertex_elevatedMap("w", 3, -1)

>>> # Display map updated map information
>>> eMap.display_map_info()
Map size: 4
Obstacles: set()
Vertices and elevations: {'u': 5, 'v': 0, 'w': -1}

>>> # Add an edge (u, v)
>>> eMap.add_edge("u", "v", 10)

>>> # Get edge weight with and without accounting for elevation:
>>> eMap.get_weight("u", "v")
10
>>> eMap.get_weight_elevation("u", "v")
6
```
When traversing the map, 

- (Large improvement) Implement “Whatever-First” Search (WFS), `wfs_search(self, start, h, *h_args)`: This algorithm takes as input a map, a starting source tile, and a heuristic function h. The heuristic function takes a map, a vertex, and a list of additional arguments (possibly empty) as inputs, and outputs an integer weight. The implementation of WFS uses an updateable min priority queue where the weight of vertex v in the priority queue is the output of the heuristic function on the map, the vertex v, and any additional arguments.
- (Small improvement) Implement a print function for WFS (similar to the one for BFS), `print_wfs_path(self, visit_order)`.
Example useage:
``` sh
# Example code to use wfs_search and print_wfs_path methods
map_instance = Map(5)

# Add obstacles to the map
map_instance.add_obstacle(1, 2)
map_instance.add_obstacle(2, 2)
map_instance.add_obstacle(3, 2)

# Define the start point and heuristic function
start_point = (0, 0)

# Example using wfs_search algorithm
visit_order = map_instance.wfs_search(start_point, lambda map, vertex, *args: 0)
print("WFS Visit Order:")
map_instance.print_wfs_path(visit_order)
```
This will print:
``` sh
WFS Visit Order:
0 5 9 3 7
1 6 0 4 8
2 # # # 9
3 7 1 5 0
4 8 2 6 1
```

- (Large improvement) Modify Dijkstra and/or Whatever-First Search to exit early upon reaching a designated goal tile. (In other words, convert Dijkstra and/or Whatever-First Search into a *single-pair* shortest paths algorithm.) Construct a few test maps and compare the number of vertices visited before reaching the goal for single-pair-BFS (from part B) vs single-pair Dijkstra/WFS.
    - Extra bonus `wfs_heuristic(map_instance, current_vertex, goal, g_costs)`: Our algorithm makes use of the Manhattan Distance heuristic. The Manhattan Distance between two points is the sum of the absolute differences of their coordinates. The idea is to estimate the minimum cost required to reach the goal by only moving along the grid lines. An improvement to this hueristic would be some sort of obstacle consideration, such as adding 3 to the Manhattan distance for each obstacle in each path (because it would usually take around O(3) extra steps to go around the obstacle). However, we didn't have time to implement obstacle considerations.
- (Small improvement) Print function for modified Dijkstra and/or WFS that shows the *order* in which tiles are visited, `print_algorithm_visit_order(self, algorithm, *args)`.
Example useage:
``` sh
# Example code to use dij_spsp and print_dij_spsp methods
map_instance = Map(5)

# Add obstacles to the map
map_instance.add_obstacle(1, 2)
map_instance.add_obstacle(2, 2)
map_instance.add_obstacle(3, 2)

# Display the initial map
print("Initial Map:")
map_instance.display_map()
print()

# Define the start and goal points
start_point = (0, 0)
goal_point = (4, 4)

# Find the single-pair shortest path using Dijkstra's algorithm
shortest_path, shortest_distance, visit_order = map_instance.dij_spsp(start_point, goal_point)

# Display the shortest path grid
print("Shortest Path:")
map_instance.print_dij_spsp(shortest_path)

# Print the shortest distance and visit order
print(f"Shortest Distance: {shortest_distance}")
print(f"Visit Order: {visit_order}")
```
This will print:
```sh
Initial Map:
. . . . . 
. . . . . 
. # # # .
. . . . .
. . . . .

Shortest Path:
0 . . . .
1 . . . .
2 # # # .
3 . . . .
4 5 6 7 8

Shortest Distance: 8
Visit Order: [(0, 0), (0, 1), (1, 0), (0, 2), (1, 1), (2, 0), (0, 3), (2, 1), (3, 0), (0, 4), (1, 3), (3, 1), (4, 0), (1, 4), (2, 3), (4, 1), (2, 4), (3, 3), (4, 2), (3, 4), (4, 3), (4, 4)]
```

</details>
  
## Testing

<details>
<summary> Tests for our classes and functions reside in `test.py`. We used the `unittest` package which allows you group tests into classes based on their purpose. To run all tests, run `python test.py` or `python -m unittest test.TestClass` where `TestClass` is one of the groups of tests. `unittest` will print `OK` to the terminal if all tests are passed. </summary>
