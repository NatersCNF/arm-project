import math
from arm import *
from path import *


link1 = links(0, 4, math.radians(90), [-math.pi, math.pi])
link2 = links(0, 4, 0, [-math.pi, math.pi])
link3 = links(0, 4, -math.radians(90), [-math.pi, math.pi])
link4 = links(0, 4, 0, [-math.pi, math.pi])
link5 = links(0, 4, math.radians(90), [-math.pi, math.pi])
link6 = links(0, 4, math.radians(45), [-math.pi, math.pi])

arm = [link1, link2, link3, link4, link5, link6]

anglesPI = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

angles = [math.radians(30), math.radians(45), -math.radians(20), math.radians(60), math.radians(45), -math.radians(20)]
pos1 = getAllPos(arm, angles)

path1 = path()


path1.addP(point(2, 2, 2, math.pi, 1, 0))
path1.addP(point(5, -2, 3, -math.pi, math.pi, 0))
path1.addP(point(-5, -5, 0, math.pi, 0, -1))
path1.addP(point(2, 2, 2, math.pi, 1, 0))

#path1.addP(point(3,3,3,0,0,0))
#path1.addP(point(2, 2, 2, math.pi, 1, 0))


ps1 = pointSet(path1, arm, "smooth",4)
#pathPoints = ps1.updateCheck()

#print("Generate P2P")

#ps1.generatePoints()

print("Printing ALL points")
#print("")
#ps1.printPoints()

ps1.printAllPoints()

#ps1Angles = ps1.getPathAngles()
#print("Print Angles")
#print("")
#for row in ps1Angles:
#    print(str(row))




#DESIREDPOS = [-6, 2, 3]
#angles2 = getPosAngle(DESIREDPOS[0], DESIREDPOS[1], DESIREDPOS[2], 0, -math.pi/2, 0, arm)
#pos2 = getAllPos(arm, angles2)

#i = 1
#for row in pos2:
#    print("P_"+str(i)+"="+str(row))
#    i = i + 1

    

# DESIREDPOS = [3, 3, 3]

# angles2 = getPosAngle(DESIREDPOS[0], DESIREDPOS[1], DESIREDPOS[2], 0, 0, 0, arm)

# print(angles2)

# print("TESTING VALUES")

# pos2 = getAllPos(arm, angles2)
# for row in pos2:
#     print(row)