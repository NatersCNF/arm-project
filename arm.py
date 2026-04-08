import sympy
import numpy as np

class links:
    def __init__(self, dORtheta, a, alpha, Jlim, joint='r'):
        self.a = a
        self.alpha = alpha

        if joint == 'r':
            self.theta = sympy.Symbol(f'theta_{id(self)}')
            self.d = dORtheta
        
        elif joint == 'p':
            self.d = sympy.Symbol(f'd_{id(self)}')
            self.theta = dORtheta

        if Jlim[0] != Jlim[1] and len(Jlim) == 2:
            self.Jlim = sorted(Jlim)

        self.joint = joint
        
        self.DH = self.getDH()

    def getDH(self):
        ctheta = sympy.cos(self.theta)
        stheta = sympy.sin(self.theta)
        calpha = sympy.cos(self.alpha)
        salpha = sympy.sin(self.alpha)

        a = self.a
        d = self.d

        DH = sympy.Matrix([
            [ctheta, (-stheta * calpha), (stheta * salpha), (a * ctheta)],
            [stheta, (ctheta * calpha), (-ctheta * salpha), (a * stheta)],
            [0, salpha, calpha, d],
            [0, 0, 0, 1]
        ])
        return DH
    
def getSetDH(arm, n):
    if n <= len(arm):
        DH = arm[0].DH
        for i in range(1, n):
            DH = DH * arm[i].DH
        return DH
    else:
        raise ValueError("not enough links, or invalid link index")


def getAllPos(arm, angles):
    allPos = []

    for i in range(0, len(arm)):
        DH = getSetDH(arm, (i + 1))
        sub = []
        for j in range(0, i + 1):
            if arm[j].joint == 'r':
                sub.append([arm[j].theta,angles[j]])
            else:
                sub.append([arm[j].d,angles[j]])
        
        pos = DH.subs(sub)
        x = float(pos[0, 3])
        y = float(pos[1, 3])
        z = float(pos[2, 3])

        allPos.append([x, y, z])
    return allPos