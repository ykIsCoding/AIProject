# Required imports
import numpy as np
import networkx as nx
from Boundaries import Boundaries
from Map import EPSILON
from queue import PriorityQueue 

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0

def h1(current_node, objective_node) -> np.float32:
    """ First heuristic to implement """
    global NODES_EXPANDED
    h = 0
    x1, y1 = current_node
    x2, y2 = objective_node
    h = abs(x1 - x2) + abs(y1 - y2)
    NODES_EXPANDED += 1
    return h

def h2(current_node, objective_node) -> np.float32:
    """ Second heuristic to implement """
    global NODES_EXPANDED
    h = 0
    x1, y1 = current_node
    x2, y2 = objective_node
    h = max(abs(x1 - x2),abs(y1 - y2))
    NODES_EXPANDED += 1
    return h

def build_graph(detection_map: np.array, tolerance: np.float32) -> nx.DiGraph:
    """ Builds an adjacency graph (not an adjacency matrix) from the detection map """
    # The only possible connections from a point in space (now a node in the graph) are:
    #   -> Go up
    #   -> Go down
    #   -> Go left
    #   -> Go right
    # Not every point has always 4 possible neighbors
    graph = nx.DiGraph()
    def get_neighbours(detection_map:np.array, cur_x,cur_y):
        #get neighbours whose detection possibilities are under tolerance
        top_bottom = [-1,0,1,0]
        left_right = [0,1,0,-1]
        neighbours = []
        for i in range(0,4):
            neighbour_y = cur_y + top_bottom[i]
            neighbour_x = cur_x + left_right[i]
            if neighbour_y >=0 and neighbour_x>=0 and neighbour_y < len(detection_map) and neighbour_x < len(detection_map[0]):
                neighbours.append((neighbour_x,neighbour_y))
        return neighbours
    
    for y in range(0,len(detection_map)): #loop through all maptiles and calculate the cost (aka edge values) for every map tile
        for x in range(0,len(detection_map[0])):
            current_node = (x,y)
            temp_neighbours = get_neighbours(detection_map,x,y)
            graph.add_node(current_node)
            for n in temp_neighbours:
                diff = abs(detection_map[y][x] - detection_map[n[1]][n[0]])
                if diff <= tolerance:
                    graph.add_edge(current_node, n, weight=diff)
    print("all cost edges for all vertices have been set. Graph generated.")
    return graph


def discretize_coords(high_level_plan: np.array, boundaries: Boundaries, map_width: np.int32, map_height: np.int32) -> np.array:
    """ Converts coordiantes from (lat, lon) into (x, y) """
    width_res = boundaries.calculate_lat_diff()/map_width #calculate the difference in lat from 1 tile to another
    height_res = boundaries.calculate_lon_diff()/map_height #calculate the difference in long from 1 tile to another
    return [(int((v[0]-boundaries.min_lat)/width_res),int((v[1]-boundaries.min_lon)/height_res)) for v in high_level_plan]
        

def bfs(locations, initial_location_index, heuristic_function):
    print("POIS", locations, initial_location_index)
    q = list(locations) 
    cur = locations[initial_location_index]
    q.pop(initial_location_index)
    f = [cur]
    while q: # While q is not empty
        c = np.inf
        lcn = None
        lcn_index = -1 
        for i, g in enumerate(q):
            if heuristic_function(cur, g) < c:
                c = heuristic_function(cur, g)
                lcn = g
                lcn_index = i
        if lcn is not None:
            f.append(lcn)
            cur = lcn
            q.pop(lcn_index) 
        else:
            break
        print("q", q)
    print("POIS2", f, q)
    return f
def path_finding(G: nx.DiGraph,
                 heuristic_function,
                 locations: np.array, 
                 initial_location_index: np.int32, 
                 boundaries: Boundaries,
                 map_width: np.int32,
                 map_height: np.int32) -> tuple:
    """ Implementation of the main searching / path finding algorithm """
    #come up with a high level plan by planning order of visiting POI using heuristic to estimate cost, take the least cost
    #high level plan will be an array of POIs in order of visit
    #use bfs to determine order
    high_level_plan = bfs(locations, initial_location_index, heuristic_function)
    #discretize coordinates
    high_level_plan = discretize_coords(high_level_plan,boundaries,map_width,map_height)
    #use a star to compute least cost path
    print("high level plan",high_level_plan)
    full_path = []
    total_cost = 0
    for i in range(len(high_level_plan) - 1):
        start_node = high_level_plan[i]
        goal_node = high_level_plan[i + 1]
        try:
            shortest_path = nx.astar_path(G, start_node, goal_node, heuristic=heuristic_function)
            print("shortest path", shortest_path, "from: ",start_node," to ", goal_node)
            full_path.append(shortest_path) 
            print("**********************************")
            total_cost += compute_path_cost(G, [shortest_path])
            
        except nx.NetworkXNoPath:
            print(f"No path found between discretized locations {start_node} and {goal_node}")
            return [], NODES_EXPANDED
    return full_path, NODES_EXPANDED
                

def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """ Computes the total cost of the whole planning solution """
    print("solution plan", solution_plan)
    total_cost = 0.0
    if len(solution_plan) < 2:
        return total_cost
    for h in range(len(solution_plan)):
        for i in range(len(solution_plan[h]) - 1):
            u = solution_plan[h][i]
            v = solution_plan[h][i + 1]
            if G.has_edge(u, v):
                total_cost += G[u][v]['weight']
            else: return -1.0
    return np.float32(total_cost)
