import math
from arm import *


link1 = links(0, 3, math.radians(90), [-math.pi, math.pi])
link2 = links(0, 2, 0, [-math.pi, math.pi])
link3 = links(0, 2, -math.radians(90), [-math.pi, math.pi])
link4 = links(0, 1, 0, [-math.pi, math.pi])
link5 = links(0, 3, math.radians(90), [-math.pi, math.pi])


arm = [link1, link2, link3, link4, link5]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45)]
pos1 = getAllPos(arm, angles)



for row in pos1:
    print(row)

angles2 = getPosAngle(5, 0, 3, 0, 0, 0, arm)
print(angles2)