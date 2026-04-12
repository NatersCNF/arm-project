import sympy
import numpy as np
import math
from arm import *

class point:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.pos = [x, y, z]
        self.rot = [roll, pitch, yaw]
        self.full = self.pos + self.rot


class path:
    def __init__(self, arm, type = "p2p"):
        self.keyPoints = []

        if all(isinstance(item, links) for item in arm):
            self.arm = arm
        else:
            raise ValueError("arm must be an array of links from the links class")

        if type == "smooth":
            self.resolution = 1

        elif type != "p2p":
            raise ValueError("invalid type")
        
        self.type = type
        
    def addP(self, point):
        self.keyPoints.append(point.full)
    
    def removeP(self, index):
        self.keyPoints.pop(index - 1)

    def getKey(self):
        return self.keyPoints
    
    def getStart(self):
        return self.keyPoints[-1]
    
    def getEnd(self):
        return self.keyPoints[0]
    
    def setResolution(self, resolution=1):
        if resolution < 0:
            self.resolution = resolution
        else:
            raise ValueError("resolution must be greater than 0 and less than the 2 closest points to work properly")

class pointSet:
    def __init__(self, keyPoints):
        if len(keyPoints) > 1:
            self.keyPoints = keyPoints        
        else:
            raise ValueError("not enough points in the path")
        
        self.type = keyPoints.type