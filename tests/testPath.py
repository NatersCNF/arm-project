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

Scara = []
Scara.append(arm.link(r=2, theta=0, a=0, alpha=0, joint='r'))
Scara.append(arm.link(r=0, theta=0, a=5, alpha=0, joint='r'))
Scara.append(arm.link(r=0, theta=0, a=5, alpha=0, joint='r'))
Scara.append(arm.link(r=2, theta=0, a=0, alpha=0, joint='p'))
Scara.append(arm.link(r=0, theta=0, a=1, alpha=0, joint='r'))

anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = arm.getAllPos(Arm, angles)


PointsPerSecond = 100       # the number of points along a line per second of movement (based on the speed, determines animation framerate)
UnitsPerSecond = 5         # the speed at which the endeffector will move along the path in units per second

rotational_units = "deg"   # can be "deg", "rev" or"rad", just changes the text
distance_units = "m"       # doesn't actually mean anything, just changes the text


path1 = path.path()
a = 3
heightDiff = 4

path1.addP(path.point(a, a, 2-(heightDiff), math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 2-(heightDiff / 2), math.pi, 0, math.pi))
path1.addP(path.point(-a, -a, 2, math.pi, 0, math.pi))
path1.addP(path.point(-a, a, 2+(heightDiff / 2), math.pi, 0, math.pi))
path1.addP(path.point(a, a, 2+(heightDiff), math.pi, 0, math.pi))
path1.addP(path.point(a, -a, 2-(heightDiff / 2), math.pi, 0, math.pi))

ps1 = path.pointSet(path1, Scara, "smooth",PointsPerSecond,UnitsPerSecond)
ps1.updateCheck()
ps1.generatePoints()
print("Calculating...")
Parm, Pangles, Ppoints, Pkey = ps1.getPathData()



"""

heart1 = path.path()
heart2 = path.path()

h_level = 1


heart1.addP(path.point(3, 0, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(0, -3, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(-4, -3, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(-4, -1.5, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(-2.5, 0, h_level, math.pi, 0, math.pi))

heart1.addP(path.point(-2.5, 0, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(-4, 1.5, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(-4, 3, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(0, 3, h_level, math.pi, 0, math.pi))
heart1.addP(path.point(3, 0, h_level, math.pi, 0, math.pi))

hear1_path = path.pointSet(heart1, Scara, "smooth",PointsPerSecond,UnitsPerSecond)
hear1_path.updateCheck()
hear1_path.generatePoints()

print("Calculating...")
Parm, Pangles, Ppoints, Pkey = hear1_path.getPathData()
"""


plot.dispPath(Arm=Parm,Angles=Pangles,Points=Ppoints,PPs=PointsPerSecond,keyPos=Pkey,L_unit=distance_units,rot_unit=rotational_units)