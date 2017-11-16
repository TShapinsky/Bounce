import math
from scipy.optimize import minimize
import numpy as np
class Ray:
    def __init__(self, point, vector):
        self.point = np.array(point)
        self.vector = np.array(vector)

    def get_point(self, scalar):
        return self.point + self.vector*scalar

    def get_distance(self,x,args):
        ray2 = args
        p2 = ray2.get_point(x[1])
        p1 = self.get_point(x[0])
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def find_closest_point(self, ray2):
        res = minimize(self.get_distance,[0,0],(ray2))
        points = [self.get_point(res.x[0]),ray2.get_point(res.x[1])]
        #print(points)
        return np.mean(points,axis=0)
