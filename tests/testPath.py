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
Arm.append(arm.link(0.5, 0, 0,  math.pi/2))
Arm.append(arm.link(0, 0, 3, 0))
Arm.append(arm.link(0, 0, 3,  0))
Arm.append(arm.link(3, 0, 0,  math.pi/2))
Arm.append(arm.link(1, 0, 0, -math.pi/2))
Arm.append(arm.link(3, 0, 0,  0))


anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = arm.getAllPos(Arm, angles)

path1 = path.path()
"""
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


"""
a = 3
heightDiff = 1

path1.addP(path.point(a, a, -heightDiff, math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 0, math.pi, 0, math.pi))
path1.addP(path.point(-a, -a, 0, math.pi, 0, math.pi))
path1.addP(path.point(-a, a, 0, math.pi, 0, math.pi))
path1.addP(path.point(a, a, heightDiff, math.pi, 0, math.pi))

path1.addP(path.point(a, a, 3, math.pi, math.pi/2, math.pi))
path1.addP(path.point(0, a, 3, math.pi, math.pi/2, math.pi))
path1.addP(path.point(a, a, -1, math.pi, 0, math.pi))


PointsPerSecond = 100       # the number of points along a line per second of movement (based on the speed)
UnitsPerSecond = 20         # the speed at which the endeffector will move along the path in units per second

ps1 = path.pointSet(path1, Arm, "smooth",PointsPerSecond,UnitsPerSecond)
pathPoints = ps1.updateCheck()
ps1.generatePoints()

print("Calculating....")
Parm, Pangles, Ppoints, Pkey = ps1.getPathData()

print("Done calculating!")
print("")
print("Which display method would you like to use?")
print("1. PyQTgraph")
print("2. MatPlotLib")
usr = input()

if usr == "1":
    plot.dispPyqt(Arm=Parm, PSangles=Pangles, Points=Ppoints,keyPos=Pkey)

elif usr == "2":
    plot.dispMatplot(Arm=Parm, PSangles=Pangles, Points=Ppoints,keyPos=Pkey) 