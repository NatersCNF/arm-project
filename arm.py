import sympy
import numpy as np
import math

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

def getFullDH(arm):
    return getSetDH(arm, len(arm))

def checkPos(arm, pos):
    posMagnitude = math.sqrt(math.pow(pos[0],2) + math.pow(pos[1],2) + math.pow(pos[2],2))

    maxLen = 0
    for link in arm:
        maxLen = maxLen + link.a
    
    if maxLen < posMagnitude:
        raise ValueError("probably too long")

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

def getX(subX, Xold, J, F):
    Fval = np.array(F.subs(subX)).astype(float).flatten()
    Jval = np.linalg.pinv(np.array(J.subs(subX)).astype(float))

    delta = Jval @ Fval

    X = Xold - delta
    return X

def loopCheck(F, subX, iteration, tol=0.0001):
    if iteration > 400:
        raise ValueError("no solution was found")
    print(F.subs(subX))
    
    for expr in F:
        output = abs(expr.subs(subX))
        if output > tol:
            return True
    return False

def buildFunction(arm):
    fullDH = getFullDH(arm)
    rotation = fullDH[:3, :3]
    return F, J, variables


def getPosAngle(P, arm, Xold=None):
    x, y, z, roll, pitch, yaw = P    
    if Xold is None:
        Xold = [0] * len(arm)

    pos = [x, y, z]
    checkPos(arm, pos)
    
    fullDH = getFullDH(arm)
    
    rotation = fullDH[:3, :3]

    Rx = sympy.Matrix([
        [1, 0, 0,],
        [0, sympy.cos(roll), -sympy.sin(roll)],
        [0, sympy.sin(roll), sympy.cos(roll)]
    ])

    Ry = sympy.Matrix([
        [sympy.cos(pitch), 0, sympy.sin(pitch)],
        [0, 1, 0],
        [-sympy.sin(pitch), 0, sympy.cos(pitch)]
    ])

    Rz = sympy.Matrix([
        [sympy.cos(yaw), -sympy.sin(yaw), 0],
        [sympy.sin(yaw), sympy.cos(yaw), 0],
        [0, 0, 1],
    ])

    R = Rz * Ry * Rx

    x_expr = fullDH[0, 3] - x
    y_expr = fullDH[1, 3] - y
    z_expr = fullDH[2, 3] - z
    rotation1_expr = rotation[0, 0] - R[0, 0]
    rotation2_expr = rotation[1, 0] - R[1, 0]
    rotation3_expr = rotation[2, 0] - R[2, 0]

    F = sympy.Matrix([x_expr,y_expr,z_expr, rotation1_expr, rotation2_expr, rotation3_expr])

    variables = []

    for link in arm:        
        if link.joint == 'r':
            variables.append(link.theta)
        else:
            variables.append(link.d)
    
    X = Xold.copy()
    J = F.jacobian(variables)

    subX = [(variables[i], Xold[i]) for i in range(len(variables))]
    i = 0

    while loopCheck(F, subX, i):
        Xold = X
        subX = [(variables[i], Xold[i]) for i in range(len(variables))]
        X = getX(subX, Xold, J, F)
        i += 1
    return X