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

def getAllPos(arm, values=None):
    if values is None:
        values = []
        for link in arm:
            if link.joint == 'r':
                values.append(link.theta)
            
            elif link.joint == 'p':
                values.append(link.r)

    positions = []
    transform = np.eye(4)

    positions.append([0.0, 0.0, 0.0])

    for link, q in zip(arm, values):
        transform = transform @ link.getTransform(q)
        positions.append(transform[:3, 3].tolist())
    
    return positions

def getJacobian_old(arm, values, epsilon=0.00001):
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

def getJacobian(arm, values):
    num_joints = len(arm)
    transforms = np.array(getArmTransform(arm,values))
    arm_pos = np.array(getAllPos(arm, values))


    rotation_matrices = transforms[:,:3,:3]

    end_effector_pos = arm_pos[-1, :]

    jacobian = np.zeros((6, num_joints))

    for i in range(num_joints):
        if i == 0:
            prev_joint_z = np.array([0, 0, 1])
            prev_joint_pos = np.array([0, 0, 0])

        else:
            prev_joint_z = transforms[i-1][:3, 2]
            prev_joint_pos = arm_pos[i, :]

        current_link = arm[i]
        
        if current_link.joint == 'r':
            pos_diff = end_effector_pos - prev_joint_pos

            jacobian[:3, i] = np.cross(prev_joint_z, pos_diff)
            jacobian[3:6, i] = prev_joint_z
        
        elif current_link.joint == 'p':
            jacobian[:3, i] = prev_joint_z
            jacobian[3:6, i] = np.zeros(3)

        else:
            raise ValueError("invalid joint type")
        
    return jacobian
       
def getAngleOG(arm, P, guess=None,printOut=False,tol=0.001, maxIterations=300):
    x, y, z, roll, pitch, yaw = P

    maxIterations = int(len(arm) * 50)

    if maxIterations < 200:
        maxIterations = 200
    
    targetPos = np.array([x, y, z])

    Salpha = np.sin(yaw)
    Calpha = np.cos(yaw)
    
    Sbeta = np.sin(roll)
    Cbeta = np.cos(roll)
    
    Sgamma = np.sin(pitch)
    Cgamma = np.cos(pitch)

    targetR = np.array([
        [(Calpha * Cbeta), ((Calpha * Sbeta * Sgamma) - (Salpha * Cgamma)), ((Calpha * Sbeta * Cgamma) + (Salpha * Sgamma))],
        [(Salpha * Cbeta), ((Salpha * Sbeta * Sgamma) + (Calpha * Cgamma)), ((Salpha * Sbeta * Cgamma) - (Calpha * Sgamma))],
        [(-Sgamma), (Cbeta * Sgamma), (Cbeta * Cgamma)]
    ])

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
        
    raise ValueError("Could not find a solution within the maximum number of iterations for pos " + str(targetPos))

def getAngle_LM(arm, P, guess=None,printOut=False,tol=0.001, maxIterations=300):
    x, y, z, roll, pitch, yaw = P

    maxIterations = int(len(arm) * 50)

    if maxIterations < 200:
        maxIterations = 200
    
    targetPos = np.array([x, y, z])

    Salpha = np.sin(yaw)
    Calpha = np.cos(yaw)
    
    Sbeta = np.sin(roll)
    Cbeta = np.cos(roll)
    
    Sgamma = np.sin(pitch)
    Cgamma = np.cos(pitch)

    targetR = np.array([
        [(Calpha * Cbeta), ((Calpha * Sbeta * Sgamma) - (Salpha * Cgamma)), ((Calpha * Sbeta * Cgamma) + (Salpha * Sgamma))],
        [(Salpha * Cbeta), ((Salpha * Sbeta * Sgamma) + (Calpha * Cgamma)), ((Salpha * Sbeta * Cgamma) - (Calpha * Sgamma))],
        [(-Sgamma), (Cbeta * Sgamma), (Cbeta * Cgamma)]
    ])

    if guess is None:
        guess = np.zeros(len(arm))
    
    else:
        guess = np.array(guess, dtype=float)

    
    lam = 0.01
    alpha = 1.0
    v = 2
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

        current_error_norm = np.linalg.norm(error)

        if current_error_norm < tol:
            return guess
        
        jacobian = getJacobian(arm, guess)
        J = jacobian
        invJ = J.T @ np.linalg.inv(J @ J.T + (lam**2) * np.eye(6))
        
        test_guess = guess + alpha * (invJ @ error)
        test_transform = getForward(arm, test_guess)
        
        test_rot = test_transform[:3, :3]
        test_pos = test_transform[:3, 3]

        new_error_pos = targetPos - test_pos

        rot_error = targetR @ test_rot.T

        new_error_rot = 0.5 * np.array([
            rot_error[2,1] - rot_error[1,2],
            rot_error[0,2] - rot_error[2,0],
            rot_error[1,0] - rot_error[0,1]
        ])
        
        w_pos = 1.0
        w_rot = 0.5

        new_error = np.concatenate([
            w_pos * new_error_pos,
            w_rot * new_error_rot
        ])

        if printOut:
            print("Guess " + str(i + 1) + ": " + str(guess))

        new_error_norm = np.linalg.norm(new_error)

        if new_error_norm < current_error_norm:
            guess = test_guess
            good_guess = False
            lam /= v
        
        else:
            lam *= v
            continue

        
        
    raise ValueError("Could not find a solution within the maximum number of iterations for pos " + str(targetPos))



def solution_cleanup(Arm, values):
    primsatic_index = prismatic_indices(Arm)
    rev = 2 * np.pi
    cleaned_values = []
    for i, value in enumerate(values):
        if i in primsatic_index:
            cleaned_values.append(value)
        
        else:
            angle_abs = np.abs(value)
            sign = np.sign(value)

            if angle_abs > rev:
                count = np.floor(angle_abs / rev)
                change = count * rev
            else:
                change = 0
            new_angle = angle_abs - change

            cleaned_values.append(new_angle * sign)
    return cleaned_values
        
def getRotPos(transforms):
    rotationAxes = []
    for transform in transforms:
        x = transform[0][2]
        y = transform[1][2]
        z = transform[2][2]
        rotationAxes.append([x, y, z])
    
    return rotationAxes

def getAllRotPos(arm,valueSet):
    allTransforms = getAllTransform(arm,valueSet)
    allRotationAxes = []
    for transform in allTransforms:
        allRotationAxes.append(getRotPos(transform))
    
    return allRotationAxes

def getRotationMatrices(arm,valueSet):
    allTransforms = np.array(getAllTransform(arm,valueSet))
    rotationMatrices = allTransforms[:, :, :3, :3]
    return rotationMatrices

def prismatic_indices(Arm):
    prismatic_index = []
    for i, link in enumerate(Arm):
        if link.joint == 'p':
            prismatic_index.append(i)
    return prismatic_index

def get_all_joint_pos(Arm,valueSet):
    arm_positions = []
    for values in valueSet:
        pos = getAllPos(Arm,values)
        arm_positions.append(pos)
    return arm_positions