from main import *

import unittest

# Vertex Class tests
class TestVertex(unittest.TestCase):
    def test_add_neighbor(self):
        v = Vertex('v', 1)
        self.assertRaises(TypeError, v.add_neighbor("u"))
        u = Vertex('u', 1)
        v.add_neighbor(u)        
        self.assertTrue('u' in v.neighbors)
    
    # vertex can't have itself as a neighbor (not sure how to do try/except yet)
    def test_self_neighbor(self):
        v = Vertex('v', 1, neighbors={'v':2})
        self.assertFalse('v' in v.neighbors.keys())


# Graph Class tests
class TestGraph(unittest.TestCase):
    
    def test_add_vertex(self):
        g = Graph()
        g.add_vertex('A', 10) 
        self.assertTrue('A' in g.vertices.keys())
    
    def test_prevent_self_loop(self):
        g = Graph()
        g.add_vertex('A', 1, [('A', 2)])
        # vertex should still be added, but should not add the self-loop
        self.assertTrue('A' in g.vertices.keys())
        self.assertFalse('A' in g.vertices['A'].neighbors)
    
    # not finished yet
    # def test_neighbor_dne(self):
    #     g = Graph()
    #     g.add_vertex('A', 1, [('B', 2), ('C', 3)])
    #     self.assertTrue(print.called) # make sure the warning printed
    
    def test_valid_edge(self):
        self.assertFalse(valid_edge("A", "A"))



# Map Class tests
class TestMap(unittest.TestCase):
    def test_display_map(self):
        # Create a new map of size 2x2 with no obstacles initially
        map = Map(2)
        map.display_map()
        print()
        '''make sure it prints four dots'''
        # To be edited once display_maps saves map in variable (then we can do "assert True == whatever it is")
    
    def test_make_obstacle(self):
        # Create a new map of size 5x5 with no obstacles initially
        map = Map(5)
        # Add an obstacle at position (2, 3)
        map.add_obstacle(2, 3)
        # Check if an obstacle exists at position (2, 3)
        self.assertTrue(map.has_obstacle(2, 3))
        # Display the updated ASCII picture of the map
        map.display_map()
        print()
        # Remove the obstacle at position (2, 3)
        map.remove_obstacle(2, 3)
        # Check if an obstacle exists at position (2, 3)
        self.assertFalse(map.has_obstacle(2, 3))
        # Display the ASCII picture of the map
        map.display_map()

class TestBFSspsp(unittest.TestCase):
    
    def test_invalid_start_bfs_spsp(self):
        m = Map(2)
        self.assertTrue(m.bfs_spsp((-1,0), (2,2)) == (None,0)) #start is out of range

    def test_invalid_end_bfs_spsp(self):
        m = Map(2)
        self.assertTrue(m.bfs_spsp((0,0), (3,2)) == (None,0)) # end is out of range

    def test_bfs_spsp_setUp(self):
        map = Map(5, [(1, 1), (2, 2), (3, 3)])
        self.assertTrue(map.n == 5)

    def test_path_1(self):
        map = Map(4)
        start = (0, 0)
        end = (4, 4)
        path = map.bfs_spsp(start, end)
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4)])
        
    def test_invalid_points(self):
        m = Map(5)
        start = (0, 0)
        end = (10, 10)
        path = m.bfs_spsp(start, end)
        self.assertTrue(path == (None,0))

class TestBFSsssp(unittest.TestCase):
    def test_invalid_start_bfs_sssp(self):
        m = Map(2)
        self.assertEqual(m.bfs_sssp((-1,0)), None) #start is out of range

class TestEdges(unittest.TestCase):
    def test_valid_edge(self):
        self.assertFalse(valid_edge("u", "u"))
        self.assertTrue(valid_edge("u", "v"))




if __name__ == '__main__':
    unittest.main()










