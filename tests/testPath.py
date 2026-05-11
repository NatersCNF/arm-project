import sys
import os

# Adds the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import arm
import path
import plot

import math

Arm = []
Arm.append(arm.link(2, 0, 0, math.pi/2))
Arm.append(arm.link(0, 0, 3, 0))
Arm.append(arm.link(0, 0, 3, 0))
Arm.append(arm.link(3, 0, 0, math.pi/2))
Arm.append(arm.link(1, 0, 0, -math.pi/2))
Arm.append(arm.link(3, 0, 0, 0))



anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = arm.getAllPos(Arm, angles)

path1 = path.path()


a = 3
heightDiff = 1

path1.addP(path.point(a, a, 2-(heightDiff), math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 2-(heightDiff / 2), math.pi, 0, math.pi))
path1.addP(path.point(-a, -a, 2, math.pi, 0, math.pi))
path1.addP(path.point(-a, a, 2+(heightDiff / 2), math.pi, 0, math.pi))
path1.addP(path.point(a, a, 2+(heightDiff), math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 2-(heightDiff / 2), math.pi, 0, math.pi))





PointsPerSecond = 100       # the number of points along a line per second of movement (based on the speed, determines animation framerate)
UnitsPerSecond = 5         # the speed at which the endeffector will move along the path in units per second

#ps1 = path.pointSet(path1, Arm, "smooth",PointsPerSecond,UnitsPerSecond)
ps1 = path.pointSet(path1, Arm, "smooth",PointsPerSecond,UnitsPerSecond)
pathPoints = ps1.updateCheck()
ps1.generatePoints()

print("Calculating...")
Parm, Pangles, Ppoints, Pkey = ps1.getPathData()

plot.dispPath(Arm=Parm,Angles=Pangles,Points=Ppoints,PPs=PointsPerSecond,keyPos=Pkey)   