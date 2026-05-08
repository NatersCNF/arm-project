import sys
import os


# Adds the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import arm
import path
import plot

import numpy as np
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

b = 3

path1.addP(path.point(b, b, -1, math.pi, 0, math.pi))
path1.addP(path.point(b, b, -1, math.pi/2, 0, math.pi))
path1.addP(path.point(b, b, -1, math.pi, 0, math.pi))

path1.addP(path.point(b, b, -1, math.pi, math.pi/2, math.pi))
path1.addP(path.point(b, b, -1, math.pi, 0, math.pi))

path1.addP(path.point(b, b+1, -1, math.pi, 0, math.pi))
path1.addP(path.point(b, b, -1, math.pi, 0, math.pi))

path1.addP(path.point(b+1, b, -1, math.pi, 0, math.pi))
path1.addP(path.point(b, b, -1, math.pi, 0, math.pi))

path1.addP(path.point(b, b, 0, math.pi, 0, math.pi))
path1.addP(path.point(b, b, -1, math.pi, 0, math.pi))



a = 3
heightDiff = 1

path1.addP(path.point(a, a, 2-heightDiff, math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 2, math.pi, 0, math.pi))
path1.addP(path.point(-a, -a, 2, math.pi, 0, math.pi))
path1.addP(path.point(-a, a, 2, math.pi, 0, math.pi))
path1.addP(path.point(a, a, 2+heightDiff, math.pi, 0, math.pi))

path1.addP(path.point(a, a, 5, math.pi, math.pi/2, math.pi))
path1.addP(path.point(0, a, 5, math.pi, math.pi/2, math.pi))
path1.addP(path.point(0, a, -1, math.pi, -math.pi/2, math.pi))
path1.addP(path.point(a, a, 1, math.pi, 0, math.pi))


PointsPerSecond = 20       # the number of points along a line per second of movement (based on the speed, determines animation framerate)
UnitsPerSecond = 5         # the speed at which the endeffector will move along the path in units per second

ps1 = path.pointSet(path1, Arm, "p2p",PointsPerSecond,UnitsPerSecond)
pathPoints = ps1.updateCheck()
ps1.generatePoints()

print("Calculating....")
Parm, Pangles, Ppoints, Pkey = ps1.getPathData()

plot.dispPath(Arm=Parm,Angles=Pangles,Points=Ppoints,PPs=PointsPerSecond,keyPos=Pkey)