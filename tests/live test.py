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

path1 = path.path()
path1.setStart(path.point(3, 3, -1, math.pi, 0, math.pi))



PointsPerSecond = 100       # the number of points along a line per second of movement (based on the speed)
UnitsPerSecond = 5         # the speed at which the endeffector will move along the path in units per second

ps1 = path.pointSet(path1, Arm, "p2p",PointsPerSecond,UnitsPerSecond,live=True)
pathPoints = ps1.updateCheck()

ps1.startLivePointer()