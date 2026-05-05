import sys
import os

# Adds the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import math
import arm
import path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation

Arm = []
Arm.append(arm.link(0.5, 0, 0,  math.pi/2))
Arm.append(arm.link(0, 0, 3, 0))
Arm.append(arm.link(0, 0, 3,  0))
Arm.append(arm.link(3, 0, 0,  math.pi/2))
Arm.append(arm.link(0, 0, 0, -math.pi/2))
Arm.append(arm.link(3, 0, 0,  0))


anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = arm.getAllPos(Arm, angles)

path1 = path.path()
"""
path1.addP(path.point(3, 3, -1, math.pi, 0, math.pi))
path1.addP(path.point(3, 3, -1, math.pi/2, 0, math.pi))
path1.addP(path.point(3, 3, -1, math.pi, 0, math.pi))

path1.addP(path.point(3, 3, -1, math.pi, math.pi/2, math.pi))
path1.addP(path.point(3, 3, -1, math.pi, 0, math.pi))

path1.addP(path.point(3, 4, -1, math.pi, 0, math.pi))
path1.addP(path.point(3, 3, -1, math.pi, 0, math.pi))

path1.addP(path.point(4, 3, -1, math.pi, 0, math.pi))
path1.addP(path.point(3, 3, -1, math.pi, 0, math.pi))

path1.addP(path.point(3, 3, 0, math.pi, 0, math.pi))
path1.addP(path.point(3, 3, -1, math.pi, 0, math.pi))
"""


a = 3

path1.addP(path.point(a, a, -1, math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 0, math.pi, 0, math.pi))
path1.addP(path.point(-a, -a, 0, math.pi, 0, math.pi))
path1.addP(path.point(-a, a, 0, math.pi, 0, math.pi))
path1.addP(path.point(a, a, 1, math.pi, 0, math.pi))

path1.addP(path.point(a, a, 3, math.pi, math.pi/2, math.pi))
path1.addP(path.point(0, a, 3, math.pi, math.pi/2, math.pi))



ps1 = path.pointSet(path1, Arm, "p2p",100,10)
pathPoints = ps1.updateCheck()

print("Generate P2P")

ps1.generatePoints()

ps1.dispPath()






"""
print("Printing ALL points")
print("")
ps1.printPoints()
ps1.printAllPoints()

ps1Angles = ps1.getPathAngles()
print("Print Angles")
print("")
for row in ps1Angles:
    print(str(row))
"""