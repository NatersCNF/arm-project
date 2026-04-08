import math
from arm import *


link1 = links(0, 3, math.radians(90), [-math.pi, math.pi])
link2 = links(0, 2, 0, [-math.pi, math.pi])
link3 = links(0, 2, -math.radians(90), [-math.pi, math.pi])
link4 = links(0, 1, 0, [-math.pi, math.pi])

arm = [link1, link2, link3, link4]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60)]
pos1 = getAllPos(arm, angles)

for row in pos1:
    print(row)