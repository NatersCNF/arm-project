import sys
import os

# Adds the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import arm
from path import point, path, pointSet
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

Scara = []
Scara.append(arm.link(r=2, theta=0, a=0, alpha=0, joint='r'))
Scara.append(arm.link(r=0, theta=0, a=5, alpha=0, joint='r'))
Scara.append(arm.link(r=0, theta=0, a=5, alpha=0, joint='r'))
Scara.append(arm.link(r=2, theta=0, a=0, alpha=0, joint='p'))
Scara.append(arm.link(r=0, theta=0, a=1, alpha=0, joint='r'))

anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = arm.getAllPos(Arm, angles)


PointsPerSecond = 75       # the number of points along a line per second of movement (based on the speed, determines animation framerate)
speed = 4         # the speed at which the endeffector will move along the path in units per second
angular_speed = 0.5

linear_acceleration = 4
angular_acceleration = 0.5

rotational_units = "deg"   # can be "deg", "rev" or"rad", just changes the text
distance_units = "m"       # doesn't actually mean anything, just changes the text



a = 4
heightDiff = 1
z_level = 1
rotate = 0
"""
spiral = path()
default_orientation = [math.pi, rotate, math.pi] #down
start_point = [3, 3, z_level]
point_per_mz = 20
radius = 0.5
multiplier = 10
height = 2
start_z = 0
end_z = height

lean = math.pi / 10

point_num = int(point_per_mz * (end_z - start_z))
height_interval = height / point_num

xi = start_point[0]
yi = start_point[1]
zi = start_point[2]

spiral.addP(point(a, a, z_level-(heightDiff), math.pi, rotate, math.pi))
for i in range(point_num + 1):
    current_L = start_z + (height_interval * i)
    x = radius * np.sin(current_L * multiplier)
    y = radius * np.cos(current_L * multiplier)
    z = current_L

    roll = default_orientation[0] + (lean * np.sin(current_L * multiplier))
    pitch = default_orientation[1] + (lean * np.cos(current_L * multiplier))
    yaw = default_orientation[2]

    new_point = point(x + xi, y + yi, z + zi, roll, pitch, yaw)
    spiral.addP(new_point)

for i in range(point_num + 1):
    index = point_num - i
    new_point = spiral.keyPos[index]
    spiral.keyPos.append(new_point)


spiral_path = pointSet(spiral, Arm, "p2p_trap", PPs=PointsPerSecond, v=speed, a=linear_acceleration, angular_v=angular_speed, angular_a=angular_acceleration)
spiral_path.updateCheck()
spiral_path.generatePoints()

Parm, Pangles, Ppoints, Pkey = spiral_path.getPathData()
plot.dispPath(Arm=Parm,Angles=Pangles,Points=Ppoints,PPs=PointsPerSecond,keyPos=Pkey,L_unit=distance_units,rot_unit=rotational_units)
"""

path1 = path()
path1.addP(point(a, a, z_level-(heightDiff), math.pi, rotate, math.pi))
path1.addP(point(a, -a, z_level-(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(-a, -a, z_level, math.pi, rotate, math.pi))
path1.addP(point(-a, a, z_level+(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(a, a, z_level+(heightDiff), math.pi, rotate, math.pi))

rotate = math.pi / 10

path1.addP(point(a, a, z_level-(heightDiff), math.pi, rotate, math.pi))
path1.addP(point(a, -a, z_level-(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(-a, -a, z_level, math.pi, rotate, math.pi))
path1.addP(point(-a, a, z_level+(heightDiff / 2), math.pi, rotate, math.pi))


path1.addP(point(a, a, z_level+(heightDiff), math.pi, rotate, math.pi))
path1.addP(point(-a, a, z_level+(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(-a, -a, z_level, math.pi, rotate, math.pi))
path1.addP(point(a, -a, z_level-(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(a, a, z_level-(heightDiff), math.pi, rotate, math.pi))

rotate = 0

path1.addP(point(a, a, z_level+(heightDiff), math.pi, rotate, math.pi))
path1.addP(point(-a, a, z_level+(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(-a, -a, z_level, math.pi, rotate, math.pi))
path1.addP(point(a, -a, z_level-(heightDiff / 2), math.pi, rotate, math.pi))
path1.addP(point(a, a, z_level-(heightDiff), math.pi, rotate, math.pi))

path_types = ["p2p_trap", "p2p", "smooth"]
current_arm = Arm
current_path_type = path_types[0]



ps1 = pointSet(path1, current_arm, current_path_type, PPs=PointsPerSecond, v=speed, a=linear_acceleration, angular_v=angular_speed, angular_a=angular_acceleration)
ps1.updateCheck()
ps1.generatePoints()

print("Calculating...")
#print("POINTS: " + str(ps1.points))
Parm, Pangles, Ppoints, Pkey = ps1.getPathData()


"""

heart1 = path()
heart2 = path()

h_level = 1


heart1.addP(point(3, 0, h_level, math.pi, 0, math.pi))
heart1.addP(point(0, -3, h_level, math.pi, 0, math.pi))
heart1.addP(point(-4, -3, h_level, math.pi, 0, math.pi))
heart1.addP(point(-4, -1.5, h_level, math.pi, 0, math.pi))
heart1.addP(point(-2.5, 0, h_level, math.pi, 0, math.pi))

heart1.addP(point(-2.5, 0, h_level, math.pi, 0, math.pi))
heart1.addP(point(-4, 1.5, h_level, math.pi, 0, math.pi))
heart1.addP(point(-4, 3, h_level, math.pi, 0, math.pi))
heart1.addP(point(0, 3, h_level, math.pi, 0, math.pi))
heart1.addP(point(3, 0, h_level, math.pi, 0, math.pi))

hear1_path = pointSet(heart1, Scara, "smooth",PointsPerSecond,UnitsPerSecond)
hear1_path.updateCheck()
hear1_path.generatePoints()

print("Calculating...")
Parm, Pangles, Ppoints, Pkey = hear1_path.getPathData()
"""


plot.dispPath(Arm=Parm,Angles=Pangles,Points=Ppoints,PPs=PointsPerSecond,keyPos=Pkey,L_unit=distance_units,rot_unit=rotational_units)