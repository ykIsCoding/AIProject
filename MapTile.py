
class MapTile:
    def __init__(self,lat,long,x,y,detection_prob):
        self.x = x # index in row of grid
        self.y = y # index in column of grid
        self.lat = lat # latitude of current maptile
        self.long = long #longitude of current maptile
        self.detection_prob=detection_prob
    def set_detection_prob(self,val):
        self.detection_prob = val
    def set_edges(self,top,left,bottom,right):
        self.edges = [top,left,bottom,right]
    def get_detection_prob(self):
        return self.detection_prob

        