from main import *


#  (Small improvement Difficult_Terrain_Map): Contains regular tiles and obstacles, as well as tiles that represent terrain that is more difficult to traverse. 
# It's possible for the user to input variables reg_cost and high_cost to determine the cost of traversing regular and difficult terrain, respectively. 
# In this type of map, traveling from tile A to tile B (where A and B are non-obstacles) costs high_cost if either tile is difficult terrain; otherwise 
# (if both A and B are regular terrain) it costs reg_cost.
class Difficult_Terrain_Map:
    def __init__(self, reg_cost=1, high_cost=3):
        self.graph = Graph()
        self.regular_cost = reg_cost
        self.high_cost = high_cost

    def add_tile(self, tile_key, tile_type):
        if tile_key not in self.graph.vertices:
            self.graph.add_vertex(tile_key, tile_type)
        else:
            print(f"Tile with key {tile_key} already exists.")

    def add_obstacle(self, obstacle_key):
        self.add_tile(obstacle_key, "obstacle")

    def add_difficult_terrain(self, terrain_key):
        self.add_tile(terrain_key, "difficult_terrain")

    def connect_tiles(self, tile_key_1, tile_key_2):
        tile_1_type = self.graph.vertices[tile_key_1].value
        tile_2_type = self.graph.vertices[tile_key_2].value

        if tile_1_type == "obstacle" or tile_2_type == "obstacle":
            print("Cannot connect tiles with obstacles.")
        else:
            cost = self.high_cost if tile_1_type == "difficult_terrain" or tile_2_type == "difficult_terrain" else self.regular_cost
            self.graph.add_edge(tile_key_1, tile_key_2, cost)

    def get_cost(self, tile_key_1, tile_key_2):
        return self.graph.get_weight(tile_key_1, tile_key_2)

# Elevated map (small improvement): 
# downhill cost should be negative
class ElevatedMap(Map):
    def __init__(self, n, cost_uphill, cost_downhill, obstacles=set()):
        super().__init__(n, obstacles=obstacles)
        self.cost_uphill = cost_uphill
        self.cost_downhill = cost_downhill
        self.elevation = {}

    def add_vertex_elevatedMap(self, k, v, elevation, neighbors=[]):
        self.add_vertex(k,v,neighbors) # call the Graph function to add a vertex
        self.elevation[k] = elevation # update elevation dictionary

    def delete_vertex_elevatedMap(self, k, elevation):
        self.delete_vertex(k) # call parent class function
        del self.elevation[k] # remove from elevation dictionary

    # Get the ammount that should be added to the weight of the edge (u, v) to account for elevation differece
    # Although this is not a directed graph, assume that we are traversing the graph from u to v 
    def get_elevation_difference(self, k_u, k_v):

        # If going downhill
        if self.elevation[k_u] > self.elevation[k_v]:
            return self.cost_downhill

        # If going uphill
        if self.elevation[k_u] < self.elevation[k_v]:
            return self.cost_uphill
        
        # No change in elevation
        return 0

    # Not a directed graph, but assume that if traversing, you're going from u to v
    def get_weight_elevation(self, k_u, k_v):
        edge_weight = self.get_weight(k_u, k_v) # Original edge weight
        difference = self.get_elevation_difference(k_u, k_v) # Ammount to adjust by
        
        if edge_weight == None:
            return difference
        
        return edge_weight + difference
    
    def display_map_info(self):
        print(f"Map size: {self.n}")
        print(f"Obstacles: {self.obstacles}")
        print(f"Vertices and elevations: {self.elevation}")









