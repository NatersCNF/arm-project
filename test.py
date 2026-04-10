import math
from arm import *


link1 = links(0, 5, math.radians(90), [-math.pi, math.pi])
link2 = links(0, 3, 0, [-math.pi, math.pi])
link3 = links(0, 4, -math.radians(90), [-math.pi, math.pi])
link4 = links(0, 1, 0, [-math.pi, math.pi])
link5 = links(0, 6, math.radians(90), [-math.pi, math.pi])
link6 = links(0, 3, math.radians(45), [-math.pi, math.pi])

arm = [link1, link2, link3, link4, link5, link6]

anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = getAllPos(arm, angles)

for row in pos1:
    print(row)

# DESIREDPOS = [3, 3, 3]

# angles2 = getPosAngle(DESIREDPOS[0], DESIREDPOS[1], DESIREDPOS[2], 0, 0, 0, arm)

# print(angles2)

# print("TESTING VALUES")

# pos2 = getAllPos(arm, angles2)
# for row in pos2:
#     print(row)