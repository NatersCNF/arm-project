import numpy as np
import math
from arm import *
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation


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
    
    def setStart(self, P = None):
        if P is None:
            straightAngles = [math.pi] * len(self.Arm)

            startPoints = getAllPos(self.Arm, straightAngles)
            P = point(startPoints[-1][0], startPoints[-1][1], startPoints[-1][2], 0, 0, 0)
        
        self.addP(P)
            
class pointSet:
    def __init__(self, path, Arm, type="p2p", PPs=30, speed=1,live=False):
        self.nP = 0
        self.speed = speed
        
        if not(type == "p2p" or type == "smooth"):
            raise ValueError("invalid type")
        self.type = type
        
        if all(isinstance(item, link) for item in Arm):
            self.Arm = Arm
        else:
            raise ValueError("arm must be an array of links from the links class")
        
        if PPs > 1:
            self.PPs = PPs

        self.uvec = []

        self.points = []

        if len(path.keyPos) > 1 or live == True:
            self.path = path
        else:
            raise ValueError("not enough points in the path")
        
        self.updateCheck()
    
    def getPathData(self, printToggle=False): # used for plotting commands
        angles = self.getPathAngles(printToggle)
        return self.Arm, angles, self.points, self.path.keyPos

    def applyTransform(self, meshItem, P1, P2):
        vec = P2 - P1
        normLength = np.linalg.norm(vec)
        meshItem.resetTransform()

        if normLength > 0:
            zAxis = np.array([0, 0, 1])
            target = vec / normLength
            dot = np.dot(zAxis, target)
            
            if dot < -0.999999:
                meshItem.rotate(180, 0, 1, 0)
            else:
                cross = np.cross(zAxis, target)
                angle = np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))

                if np.linalg.norm(cross) > 1e-6:
                    meshItem.rotate(angle, *cross)
            meshItem.translate(*P1)

    def updatenP(self):
        self.nP = len(self.path.keyPos)
    
    def generatePoints(self):
        if self.type == "smooth":
            self.generateSpline()

        elif self.type == "p2p":
            self.generateP2P()
        
        print("Number of points: " + str(len(self.points)))

    def updateUVec(self):
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

    def updateCheck(self):
        #print("nP value:")
        self.updatenP()
        #print(str(self.nP))

        #print("POINTS:")
        #print(str(self.path.keyPos))

        #print("UVECTORS:")
        self.updateUVec()
        #print(str(self.uvec))

    """
    def addLivePoint(self):
        print("Enter a point, or type 'help' for info")
        usr = input()

        if usr.lower() == "help":
            print("Type either the x y and z coordinates (comma and space seperated) to stay the same orientation, or the full x, y, z, roll, pitch, yaw")
            print("If you want to use the angle pi, just type 'pi' or '-pi'")
            print("Proper syntax example (just xyz): '5, 3, 2', or full: '2, 1, 3, pi, 0, 0")
        
        else:
            Point = usr.split(", ")
            for i in range(len(Point)):
                if Point[i].lower() == "pi":
                    Point[i] = math.pi
                
                elif Point[i].lower() == "-pi":
                    Point[i] = -math.pi
            
            Point = [int(n) for n in Point]
            prevPoint = self.path.keyPos[-1]
            if len(Point) == 3:
                Point = point(Point[0], Point[1], Point[2], prevPoint[3], prevPoint[4], prevPoint[5])
            
            else:
                Point = point(Point[0], Point[1], Point[2], Point[3], Point[4], Point[5])
            
            self.path.addP(Point)
            
    def startLivePointer(self):
        speed = self.speed
        PPs = self.PPs

        
        while True:
            self.updateCheck()
            self.addLivePoint()
            print("CURRENT POINTS:")
            for i, point in enumerate(self.path.keyPos):
                print(str(i + 1) + ". " + str(point))
            
            m = []
            m.append([0, 0, 0])
    """

    def generateP2P(self): #speed = u/s, and specified earlier, PPs = points per second 
        self.points.clear()

        self.points.append(self.path.keyPos[0])

        for i in range(self.nP-1):
            initial = self.path.keyPos[i]
            final = self.path.keyPos[i + 1]
            currentLength = self.uvec[i][3]
            direction = self.uvec[i][:3]            

            pos_diff = np.linalg.norm(np.array(final[:3]) - np.array(initial[:3]))
            rot_diff = np.linalg.norm(np.array(final[3:]) - np.array(initial[3:]))

            metric = max(pos_diff, rot_diff)

            n = int(metric * (self.PPs / self.speed))
            n = max(1, n)

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
        self.points.clear()
        m = []                  # assuming start & stop at first and last point
        m.append([0, 0, 0])

        for i in range(1, (self.nP - 1)):
            PA = self.path.keyPos[i + 1]
            PB = self.path.keyPos[i - 1]
            

            mx = (PA[0] - PB[0]) / 2
            my = (PA[1] - PB[1]) / 2
            mz = (PA[2] - PB[2]) / 2
            
            slope = [mx, my, mz]
            m.append(slope)
        m.append([0, 0, 0])
        
        for i in range(0, self.nP - 1):
            initial = self.path.keyPos[i]
            final = self.path.keyPos[i + 1]

            posx = final[0] - initial[0]
            posy = final[1] - initial[1]
            posz = final[2] - initial[2]

            currentLength = math.sqrt((posx ** 2) + (posy ** 2) + (posz ** 2))

            PPL = int(currentLength * (self.PPs / self.speed))
            PPL = max(1, PPL)

            Pi = self.path.keyPos[i][:3]
            Pi1 = self.path.keyPos[i + 1][:3]
            mi = m[i]
            mi1 = m[i + 1]

            

            for j in range(1, PPL+1):
                t = (1 / PPL) * j

                roll = initial[3] + ((final[3] - initial[3]) * t)
                pitch = initial[4] + ((final[4] - initial[4]) * t)
                yaw = initial[5] + ((final[5] - initial[5]) * t)

                pos = getPatT(t, Pi, Pi1, mi, mi1)
                pos = pos + [roll, pitch, yaw]

                self.points.append(pos)
 
    def getPathAngles(self,printToggle=False):
        
        printInterval = int(3000 / len(self.Arm))
        angles = []
        initial_angle = getAngleOG(self.Arm, self.points[0],printOut=printToggle)
        cleaned_angle = solution_cleanup(self.Arm, initial_angle)
        angles.append(cleaned_angle)

        for i in range(1, len(self.points)):
            if printToggle:
                print("NEW POINT "+str(i))
            P = self.points[i]
            angles.append(getAngleOG(self.Arm, P,angles[i-1], printToggle))
            if i % printInterval == 0:
                print("Current at " + str(i))

        return angles

    def printPoints(self, offset=0):
        for i, row in enumerate(self.points):
            x, y, z, roll, pitch, yaw = row

            print(f"P_{{{i+offset}}} = ({x}, {y}, {z})")

            dx = math.cos(pitch) * math.cos(yaw)
            dy = math.cos(pitch) * math.sin(yaw)
            dz = math.sin(pitch)

            dx = round(dx, 6) + x
            dy = round(dy, 6) + y
            dz = round(dz, 6) + z

            print(f"\\operatorname{{vector}}(P_{{{i+offset}}}, ({dx}, {dy}, {dz}))\n")
    
    def printAllPoints(self):
        print("p2p points:")
        print("")
        self.generateP2P()
        self.printPoints()
        n = int(len(self.points))
        
        print("spline points:")
        print("")
        self.generateSpline()
        self.printPoints(n)

def getH(h, t):
    if h == "00":
        h00 = (2 * t ** 3) - (3 * t ** 2) + 1
        return h00
    elif h == "10":
        h10 = (t ** 3) - (2 * t ** 2) + t
        return h10
    elif h == "01":
        h01 = (-2 * t ** 3) + (3 * t ** 2)
        return h01
    elif h == "11":
        h11 = (t ** 3) - (t ** 2)
        return h11
    else:
        raise ValueError("not a valid h function")
    
def getPatT(t, Pi, Pi1, mi, mi1):    
    Pi = np.array(Pi)
    Pi1 = np.array(Pi1)
    mi = np.array(mi)
    mi1 = np.array(mi1)

    P00 = getH("00",t) * Pi
    
    P10 = getH("10",t) * mi
    P01 = getH("01",t) * Pi1
    P11 = getH("11",t) * mi1
    P = P00 + P10 + P01 + P11
    
    return P.tolist()

def getRotAxisPoints(Arm,valueSet,armPositions,cLength=1):
    halfLen = cLength / 2
    allRotAxes = getAllRotPos(Arm,valueSet)
    jointPointSet = []

    for frameAxes, framePos in zip(allRotAxes, armPositions):
        frame = []

        for i in range(len(frameAxes)):
            pos = np.array(framePos[i + 1])
            rotAxis = np.array(frameAxes[i])

            modified = rotAxis * halfLen

            start = pos + modified
            end = pos - modified

            frame.append([start, end])
        
        jointPointSet.append(frame)
    
    return jointPointSet

    helper = []
    for i in range(len(P1set)):
            P1 = P1set[i]
            P2 = P2set[i]
            linkMesh = getCylinderMesh(P1, P2, type)
            if linkMesh:
                view.addItem(linkMesh)
                helper.append(linkMesh)
    return np.array(helper)