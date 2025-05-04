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
        self.probabilities = []
        self.min_prob = 100
        self.max_prob = 0

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
        lat_division = self.boundaries.calculate_lat_diff()/self.width
        lon_division = self.boundaries.calculate_lon_diff()/self.height
        
        radar_locs = self.get_radars_locations_numpy()

        #calculate detection probability of 1 tile
        def compute_detection_probabilities(lat,long):
            #lat long of current tile
           # print("lat: ", lat, "long: ",long, "height: ", self.height,"width: ",self.width)
            probabilities = []
            for rdr in self.radars:
                probabilities.append(rdr.compute_detection_level(lat,long))
            if(min(probabilities)<self.min_prob):
                self.min_prob = min(probabilities)
            if(max(probabilities)>self.max_prob):
                self.max_prob = max(probabilities)
            return max(probabilities)
        
        for y in range(0,self.height):
            temp_row = []
            temp_prob = []
            for x in range(0,self.width):
                temp_map_tile = MapTile(
                    lat = min(self.boundaries.min_lat + x * lat_division,self.boundaries.max_lat),
                    long = min(self.boundaries.min_lon + y * lon_division,self.boundaries.max_lon),
                    x = x,
                    y = y,
                    detection_prob=0
                )
                l = compute_detection_probabilities(temp_map_tile.lat,temp_map_tile.long)
                temp_row.append(temp_map_tile)
                temp_prob.append(l)
            temp_map.append(temp_row)
            self.probabilities.append(temp_prob)
        for y in range(0,self.height):
            for x in range(0,self.width):
                #min-max technique
                cur_prob = self.probabilities[y][x]
                residual_cost = 0.01
                possibility = (((cur_prob-self.min_prob)/(self.max_prob-self.min_prob))*(1-residual_cost)) + residual_cost
                temp_map[y][x].set_detection_prob(possibility)
        self.map = temp_map
        print("Map has been set!")
        return [[y.get_detection_prob() for y in x] for x in temp_map]


    def generate_astar_path(graph,start, target, heuristic):
        astarpath = nx.astar_path(graph,start,target,heuristic)
        print(astarpath)
        return astarpath

    





