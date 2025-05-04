# Required imports
import numpy as np
from Location import Location
from Boundaries import Boundaries
from Radar import Radar
from tqdm import tqdm
from MapTile import MapTile
import networkx as nx

# Constant that avoids setting cells to have an associated cost of zero
EPSILON = 1e-4

class Map:
    """ Class that models the map for the simulation """
    def __init__(self, 
                 boundaries: Boundaries,
                 height:     np.int32, 
                 width:      np.int32, 
                 radars:     np.array=None):
        self.boundaries = boundaries        # Boundaries of the map
        self.height     = height            # Number of coordinates in the y-axis
        self.width      = width             # Number of coordinates int the x-axis
        self.radars     = radars            # List containing the radars (objects)
        self.map = None

    def generate_radars(self, n_radars: np.int32) -> None:
        """ Generates n-radars randomly and inserts them into the radars list """
        # Select random coordinates inside the boundaries of the map
        lat_range = np.linspace(start=self.boundaries.min_lat, stop=self.boundaries.max_lat, num=self.height)
        lon_range = np.linspace(start=self.boundaries.min_lon, stop=self.boundaries.max_lon, num=self.width)
        rand_lats = np.random.choice(a=lat_range, size=n_radars, replace=False)
        rand_lons = np.random.choice(a=lon_range, size=n_radars, replace=False)
        self.radars = []        # Initialize 'radars' as an empty list

        # Loop for each radar that must be generated
        for i in range(n_radars):
            # Create a new radar
            new_radar = Radar(location=Location(latitude=rand_lats[i], longitude=rand_lons[i]),
                              transmission_power=np.random.uniform(low=1, high=1000000),
                              antenna_gain=np.random.uniform(low=10, high=50),
                              wavelength=np.random.uniform(low=0.001, high=10.0),
                              cross_section=np.random.uniform(low=0.1, high=10.0),
                              minimum_signal=np.random.uniform(low=1e-10, high=1e-15),
                              total_loss=np.random.randint(low=1, high=10),
                              covariance=None)

            # Insert the new radar
            self.radars.append(new_radar)
        return
    
    def get_radars_locations_numpy(self) -> np.array:
        """ Returns an array with the coordiantes (lat, lon) of each radar registered in the map """
        locations = np.zeros(shape=(len(self.radars), 2), dtype=np.float32)
        for i in range(len(self.radars)):
            locations[i] = self.radars[i].location.to_numpy()
        return locations
    
    def compute_detection_map(self) -> np.array:
        """ Computes the detection map for each coordinate in the map (with all the radars) """
        #array of array of MapTiles
        temp_map = []
        lat_division = self.boundaries.calculate_lat_diff()/self.height
        lon_division = self.boundaries.calculate_lon_diff()/self.width
        radar_locs = self.get_radars_locations_numpy()
        for y in range(0,self.height):
            temp_row = []
            for x in range(0,self.width):
                temp_map_tile = MapTile(
                    lat = y * lat_division,
                    long =  x * lon_division,
                    x = x,
                    y = y,
                    detection_prob = 0 #to do 
                )
                if any(coord[0] == temp_map_tile.lat and coord[1] == temp_map_tile.long for coord in radar_locs):
                    rdr = filter(lambda xr: xr.location.latitude == temp_map_tile.lat and xr.location.longitude == temp_map_tile.long, self.radars)
                    temp_map_tile.set_radar(rdr) #add radar to maptile
                temp_row.append(temp_map_tile)
                
            temp_map.append(temp_row)
        self.map = temp_map
        print("Map has been set!")
        print([[y.get_detection_prob() for y in x] for x in temp_map])
        return [[y.get_detection_prob() for y in x] for x in temp_map]
    
    def generate_graph(self):
        def get_surrounding_maptiles(map_tile): #get the top left right bottom maptiles of the current maptiles (aka cells)
            if type(map_tile) is not MapTile: return
            top_bottom = [1,0,-1,0]
            left_right = [0,-1,0,1]
            neighbours = []
            #top, then left, then bottom, then right 
            for i in range(0,4):
                neighbour_y = map_tile.y+top_bottom[i]
                neighbour_x = map_tile.x+left_right[i]
                if neighbour_y >=0 and neighbour_x>=0 and neighbour_y < self.height and neighbour_x < self.width:
                    neighbours.append(self.map[neighbour_y][neighbour_x])
                else:
                    neighbours.append(None)
            return neighbours
        
        for y in range(0,self.height): #loop through all maptiles and calculate the cost (aka edge values) for every map tile
            for x in range(0,self.width):
                current_maptile = self.map[y][x]
                if type(current_maptile) is not MapTile: return
                temp_neighbours = get_surrounding_maptiles(current_maptile)
                temp_edge_values = []
                for n in range(0,4):
                    if temp_neighbours[n] is not None and type(temp_neighbours[n]) is MapTile:
                        temp_edge_values.append(temp_neighbours[n].get_detection_prob()) #set the cost to the detection probability of the map tile being transitioned into
                    else:
                        temp_edge_values.append(np.inf) # if the maptile is out of bounds, set cost to infinity
                current_maptile.set_edges(temp_edge_values[0],temp_edge_values[1],temp_edge_values[2],temp_edge_values[3]) # the costs are then set into the maptile
        print("all cost edges for all vertices have been set. Graph generated.")

    def generate_astar_path(graph,start, target, heuristic):
        astarpath = nx.astar_path(graph,start,target,heuristic)
        print(astarpath)
        return astarpath

    def calculate_detection_prob(self):
        if self.map is None: return
        def get_detection_prob_list_of_maptile(current_maptile, residual_cost = 0.01):
            if type(current_maptile) is not current_maptile: return
            if current_maptile.get_radar() is None: return
            list_of_detection_prob = []
            for rdr in self.radars:
                if type(rdr) is not Radar: return
                list_of_detection_prob.append(rdr.compute_detection_level(current_maptile.lat, current_maptile.long))
            p_max = max(list_of_detection_prob)
            p_min = min(list_of_detection_prob)
            for itm in list_of_detection_prob:
                itm = ((itm - p_min)/(p_max-p_min))
                itm = (itm*(1-residual_cost)) + residual_cost
            return max(list_of_detection_prob) 
        for y in range(0,self.height): #loop through all maptiles and calculate the cost (aka edge values) for every map tile
            for x in range(0,self.width):
                current_maptile = self.map[y][x]
                if type(current_maptile) is not MapTile: return
                #set the maximum probability to current maptile
                probability_for_maptile = get_detection_prob_list_of_maptile(current_maptile)
                current_maptile.set_detection_prob(probability_for_maptile)





