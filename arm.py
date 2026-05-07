import numpy as np

class link:
    def __init__(self, r, theta, a, alpha, joint='r'):
        self.r = r
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.joint = joint

    def getTransform(self, x):

        if self.joint == 'r':
            theta = x
            r = self.r
        
        else:
            theta = self.theta
            r = x
        
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        calpha = np.cos(self.alpha)
        salpha = np.sin(self.alpha)

        a = self.a

        DH = np.array([
            [ctheta, -stheta * calpha, stheta * salpha, a * ctheta],
            [stheta, ctheta * calpha, -ctheta * salpha, a * stheta],
            [0, salpha, calpha, r],
            [0, 0, 0, 1]
        ])
        return DH

def getForward(arm, values):
    transform = np.eye(4)

    for link, q in zip(arm, values):
        transform = transform @ link.getTransform(q)
    
    return transform

def getArmTransform(arm,values):
    linkNum = len(arm)
    numVal = len(values)
    if linkNum != numVal:
        raise ValueError("You need to present the same number of joint values as there are joints")

    transform = []

    for link, value in zip(arm, values):
        curT = link.getTransform(value)
        transform.append(curT)
    
    for i in range(linkNum - 1): # i is the previous link, so the 2nd joint is 1, or i = 0
        transform[i + 1] = transform[i] @ transform[i + 1]
    
    return transform

def getAllTransform(arm,valueSet):
    allTransforms = []

    for value in valueSet:
        allTransforms.append(getArmTransform(arm,value))
    
    return np.array(allTransforms)

def getAllPos(arm, values):
    positions = []
    transform = np.eye(4)

    positions.append([0.0, 0.0, 0.0])

    for link, q in zip(arm, values):
        transform = transform @ link.getTransform(q)
        positions.append(transform[:3, 3].tolist())
    
    return positions

def getJacobian(arm, values, epsilon=0.00001):
    n = len(arm)
    J = np.zeros((6, n))
    curTransform = getForward(arm, values)
    curPos = curTransform[:3, 3]
    curRot = curTransform[:3, :3]

    for i in range(n):
        valuesP = values.copy()
        valuesM = values.copy()
        valuesP[i] += epsilon
        valuesM[i] -= epsilon

        transformP = getForward(arm, valuesP)
        transformM = getForward(arm, valuesM)

        newPosP = transformP[:3, 3]
        newPosM = transformM[:3, 3]

        J[:3, i] = (newPosP - newPosM) / (epsilon * 2)

        newRP = transformP[:3, :3]
        newRM = transformM[:3, :3]

        R0 = curRot
        R1 = newRP

        diffR = R1 @ R0.T

        R_plus = transformP[:3, :3]
        R_minus = transformM[:3, :3]

        R_err = R_plus @ R_minus.T

        J[3:6, i] = 0.5 * np.array([
            R_err[2,1] - R_err[1,2],
            R_err[0,2] - R_err[2,0],
            R_err[1,0] - R_err[0,1]
        ]) / (2 * epsilon)

    return J

def getAngleOG(arm, P, guess=None,printOut=False,tol=0.001, maxIterations=300):
    x, y, z, roll, pitch, yaw = P

    maxIterations = int(len(arm) * 50)

    if maxIterations < 200:
        maxIterations = 200
    
    targetPos = np.array([x, y, z])

    Rx = np.array([
        [1, 0, 0,],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],
    ])

    targetR = Rz @ Ry @ Rx

    if guess is None:
        guess = np.zeros(len(arm))
    
    else:
        guess = np.array(guess, dtype=float)

    for i in range(maxIterations):
        curTransform = getForward(arm, guess)
        curRot = curTransform[:3, :3]
        curPos = curTransform[:3, 3]

        errorPos = targetPos - curPos

        R_err = targetR @ curRot.T

        errorRot = 0.5 * np.array([
            R_err[2,1] - R_err[1,2],
            R_err[0,2] - R_err[2,0],
            R_err[1,0] - R_err[0,1]
        ])
        
        w_pos = 1.0
        w_rot = 0.5

        error = np.concatenate([
            w_pos * errorPos,
            w_rot * errorRot
        ])

        if np.linalg.norm(error) < tol:
            return guess
        
        jacobian = getJacobian(arm, guess)
        lam = 1e-2
        J = jacobian
        invJ = J.T @ np.linalg.inv(J @ J.T + (lam**2) * np.eye(6))

        alpha = 0.1
        guess += alpha * (invJ @ error)

        if printOut:
            print("Guess " + str(i + 1) + ": " + str(guess))
    
    raise ValueError("Could not find a solution within the maximum number of iterations")

def getRotPos(transforms):
    rotationAxes = []
    for transform in transforms:
        #x = [transform[0][0], transform[1][0], transform[2][0]]
        #y = [transform[0][1], transform[1][1], transform[2][1]]
        #z = [transform[0][2], transform[1][2], transform[2][2]]

        
        x = transform[0][2]
        y = transform[1][2] # attempt to make the returned rotational axis perpendicular to the link
        z = transform[2][2]
        rotationAxes.append([x, y, z])
    
    return rotationAxes

def getAllRotPos(arm,valueSet):
    allTransforms = getAllTransform(arm,valueSet)
    allRotationAxes = []
    for transform in allTransforms:
        allRotationAxes.append(getRotPos(transform))
    
    return allRotationAxes