import sympy
import numpy as np
import math
from arm import *

class point:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.pos = [x, y, z]
        self.rot = [roll, pitch, yaw]
        self.full = self.pos + self.rot


class path:
    def __init__(self, type="p2p"):
        self.keyPos = []

        if type == "smooth":
            self.resolution = 1

        elif type != "p2p":
            raise ValueError("invalid type")
        
        self.type = type
        
    def addP(self, point):
        self.keyPos.append(point.full)
    
    def removeP(self, index):
        self.keyPos.pop(index - 1)

    def getKey(self):
        return self.keyPos
    
    def getStart(self):
        return self.keyPos[-1]
    
    def getEnd(self):
        return self.keyPos[0]
    
    def setResolution(self, resolution=1):
        if resolution < 0:
            self.resolution = resolution
        else:
            raise ValueError("resolution must be greater than 0 and less than the 2 closest points to work properly")

class pointSet:
    def __init__(self, path, arm, PPu=10):
        
        if all(isinstance(item, links) for item in arm):
            self.arm = arm
        else:
            raise ValueError("arm must be an array of links from the links class")
        
        if PPu > 1:
            self.PPu = PPu

        self.uvec = []

        self.points = []
        if len(path.keyPos) > 1:
            self.path = path
        else:
            raise ValueError("not enough points in the path")
        
        self.type = path.type
        self.updateCheck()

    def getNp(self):
        return len(self.path.keyPos)

    def updateUVec(self):
        self.uvec.clear()

        nP = self.getNp()
        for i in range(nP - 1):
            v = []
            length = 0
            for j in range (3):
                v.append(self.path.keyPos[i + 1][j] - self.path.keyPos[i][j])
                length += v[j] ** 2

            length = math.sqrt(length)
            v.append(length)

            if length > 0:
                for j in range (3):
                    v[j] = v[j] / length
            self.uvec.append(v)
    
    def updateCheck(self):
        print("POINTS:")
        print(str(self.path.keyPos))

        print("UVECTORS:")
        self.updateUVec()
        print(str(self.uvec))

    def generateP2P(self):
        print("PLACE HOLDER SO I SEE WHAT HAPPENS ASDLKJASDLKSJDKASJDKLJDKLASDJSLAKJLKJDLKASJDKLSJADLAS")
        self.points.clear()
        nP = self.getNp()

        self.points.append(self.path.keyPos[0])

        for i in range(nP-1):
            initial = self.path.keyPos[i]
            final = self.path.keyPos[i + 1]
            currentLength = self.uvec[i][3]
            direction = self.uvec[i][:3]
            n = int(currentLength * self.PPu)

            interval = currentLength / n

            for j in range(1, n+1):
                step = j / n
                x = initial[0] + (direction[0] * j * interval)
                y = initial[1] + (direction[1] * j * interval)
                z = initial[2] + (direction[2] * j * interval)

                roll = initial[3] + ((final[3] - initial[3]) * step)
                pitch = initial[4] + ((final[4] - initial[4]) * step)
                yaw = initial[5] + ((final[5] - initial[5]) * step)

                x = round(x, 10)
                y = round(y, 10)
                z = round(z, 10)

                roll = round(roll, 10)
                pitch = round(pitch, 10)
                yaw = round(yaw, 10)

                self.points.append([x, y, z, roll, pitch, yaw])
    
    def getPathAngles(self):
        angles = []
        angles.append(getPosAngle(self.points[0],self.arm))
        
        Np = len(self.points)

        for i in range(1, Np):
            print("NEW POINT "+str(i))
            P = self.points[i]
            angles.append(getPosAngle(P,self.arm,angles[i-1]))
        
        return angles



    
    #def printPoints(self):
    #    print("Coordinates:")
    #    print("")
    #    i = 0
    #    for row in self.points:
    #        print("P_"+str(i)+str(tuple(row[:3])))
    #        i = i + 1
    #        
    #    print("Orientation:")
    #    print("")
    #    i = 0
    #    for row in self.points:
    #        print("A_"+str(i)+str(tuple(row[3:])))
    #        print(r"\operatorname{vector}\left(P_{"+str(i)+r"},\left(\right)\right)")
    #        i = i + 1

    def printPoints(self):
        for i, row in enumerate(self.points):
            x, y, z, roll, pitch, yaw = row

            print(f"P_{{{i}}} = ({x}, {y}, {z})")

            dx = math.cos(pitch) * math.cos(yaw)
            dy = math.cos(pitch) * math.sin(yaw)
            dz = math.sin(pitch)

            dx = round(dx, 6) + x
            dy = round(dy, 6) + y
            dz = round(dz, 6) + z

            print(f"\\operatorname{{vector}}(P_{{{i}}}, ({dx}, {dy}, {dz}))\n")