import sympy
import numpy
import math
from arm import *

class point:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.pos = [x, y, z]
        self.rot = [roll, pitch, yaw]
        self.full = self.pos + self.rot


class path:
    def __init__(self):
        self.keyPos = []

    def addP(self, point):
        self.keyPos.append(point.full)
    
    def removeP(self, index):
        self.keyPos.pop(index - 1)

    def getKey(self):
        return self.keyPos
    
    def getStart(self):
        return self.keyPos[0]
    
    def getEnd(self):
        return self.keyPos[-1]

class pointSet:
    def __init__(self, path, arm, PPs=10, type="p2p"):
        self.nP = 0
        
        if not(type == "p2p" or type == "smooth"):
            raise ValueError("invalid type")
        self.type = type
        
        if all(isinstance(item, links) for item in arm):
            self.arm = arm
        else:
            raise ValueError("arm must be an array of links from the links class")
        
        if PPs > 1:
            self.PPs = PPs

        self.uvec = []

        self.points = []
        if len(path.keyPos) > 1:
            self.path = path
        else:
            raise ValueError("not enough points in the path")
        
        self.updateCheck()

    def updatenP(self):
        self.nP = len(self.path.keyPos)
    
    def generatePoints(self):
        if self.type == "smooth":
            self.generateSpline()

        elif self.type == "p2p":
            self.generateP2P()

    def updateUVec(self):
        if self.type == "p2p":
            self.uvec.clear()

            for i in range(self.nP - 1):
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
        #elif self.type == "smooth":
    
    def updateCheck(self):
        print("nP value:")
        self.updatenP()
        print(str(self.nP))

        print("POINTS:")
        print(str(self.path.keyPos))

        print("UVECTORS:")
        self.updateUVec()
        print(str(self.uvec))
        

    def generateP2P(self, speed=1): #speed = u/s, and specified earlier, PPs = points per second 
        self.points.clear()

        self.points.append(self.path.keyPos[0])

        for i in range(self.nP-1):
            initial = self.path.keyPos[i]
            final = self.path.keyPos[i + 1]
            currentLength = self.uvec[i][3]
            direction = self.uvec[i][:3]
            n = int(currentLength * (self.PPs / speed))

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
    
    def generateSpline(self):
        m = []                  # assuming start & stop at first and last point
        m.append([0, 0, 0])

        for i in range(0, 1, (self.nP - 2)):
            mx = (self.path.keyPos[i + 1][0] - self.path.keyPos[i - 1][0]) / 2
            my = (self.path.keyPos[i + 1][1] - self.path.keyPos[i - 1][1]) / 2
            mz = (self.path.keyPos[i + 1][2] - self.path.keyPos[i - 1][2]) / 2
            
            slope = [mx, my, mz]
            m.append(slope)
        
        m.append([0, 0, 0])
            

















    def getPathAngles(self):
        angles = []
        angles.append(getPosAngle(self.points[0],self.arm))

        for i in range(1, len(self.points)):
            print("NEW POINT "+str(i))
            P = self.points[i]
            angles.append(getPosAngle(P,self.arm,angles[i-1]))
        return angles

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