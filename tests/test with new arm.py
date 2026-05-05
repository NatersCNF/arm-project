import sys
import os

# Adds the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import math
from newArm import *
from path import *


link1 = link(0, 0, 4, math.radians(90))
link2 = link(0, 0, 4, 0)
link3 = link(0, 0, 4, -math.radians(90))
link4 = link(0, 0, 4, 0,)
link5 = link(0, 0, 4, math.radians(90))
link6 = link(0, 0, 4, math.radians(45))

arm = [link1, link2, link3, link4, link5, link6]

anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = getAllPos(arm, angles)

print("FIRST POINT")
DESIREDPOS1 = [-6, 2, 3, 0, -math.pi/2, 0]
angles2 = getAngle(DESIREDPOS1, arm)
pos2 = getAllPos(arm, angles2)



i = 1
for row in pos2:
    print("P_"+str(i)+"="+str(row))
    i = i + 1


print("")
print("SECOND POINT")

DESIREDPOS2 = [2, -3, 3, 0, -math.pi/2, 0]
angles2 = getAngle(DESIREDPOS2, arm, angles2)
pos2 = getAllPos(arm, angles2)

i = 1
for row in pos2:
    print("P_"+str(i)+"="+str(row))
    i = i + 1