import numpy as np

class link:
    def __init__(self, r, theta, a, alpha, joint='r',upper_limit=None,lower_limit=None):
        self.r = r
        self.theta = theta
        self.a = a
        self.alpha = alpha
        self.joint = joint

        if upper_limit is not None and lower_limit is not None:
            if upper_limit != lower_limit:
                self.lower_limit = min(upper_limit,lower_limit)
                self.upper_limit = max(upper_limit,lower_limit)
            
            else:
                raise ValueError("Limits must not be equal")
        
        else:
            self.lower_limit = None
            self.upper_limit = None

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

def getAngle_LM(arm, P, guess=None,tol=0.001, maxIterations=300):
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

    curTransform = getForward(arm, guess)
    current_error = get_solution_error(curTransform, targetPos, targetR)
    current_error_norm = np.linalg.norm(current_error)
    jacobian = getJacobian(arm, guess)

    if current_error_norm < tol:
        return guess

    for i in range(maxIterations):
        cur_invJ = get_invJ(jacobian,lam)
        test_guess = guess + alpha * (cur_invJ @ current_error)

        test_transform = getForward(arm, test_guess)
        new_error = get_solution_error(test_transform, targetPos, targetR)
        new_error_norm = np.linalg.norm(new_error)


        if new_error_norm < current_error_norm:
            guess = test_guess
            current_error = new_error
            current_error_norm = new_error_norm

            if current_error_norm < tol:
                return guess
            jacobian = getJacobian(arm, guess)
            lam /= v 

        else:
            lam *= v
        
        if lam > 1e10:
            raise ValueError("Too high")

        
        
    raise ValueError("Could not find a solution within the maximum number of iterations for pos " + str(targetPos))

def get_better_angles(arm, target_P, previous_solution=None, tol=0.001):
    random_solution_max = 5

    if previous_solution is None:
        previous_solution = []
        for link in arm:
            if link.joint == 'r':
                previous_solution.append(link.theta)
            
            elif link.joint == 'p':
                previous_solution.append(link.r)

    try:
        return getAngle_LM(arm, target_P, guess=previous_solution,tol=tol)
    except ValueError:
        for i in range(random_solution_max):
            random_values = []
            for link in arm:
                if link.joint == 'r':
                    u_limit = link.upper_limit
                    l_limit = link.lower_limit
                    if u_limit is None and l_limit is None:
                        u_limit = np.pi
                        l_limit = -np.pi
                
                elif link.joint == 'p':
                    u_limit = link.upper_limit
                    l_limit = link.lower_limit
                    if u_limit is None and l_limit is None:
                        u_limit = 10
                        l_limit = -10
                
                random_guess = np.random.uniform(l_limit,u_limit)
                random_values.append(random_guess)
            
            try:
                return getAngle_LM(arm, target_P, guess=random_values)
            except ValueError:
                continue
        
        raise ValueError("Unreachable point.")

def get_pointset_anglesOLD(arm, point_set, start_guess=None):
    printInterval = int(3000 / len(arm))
    angles = []
    initial_angle = getAngleOG(arm, point_set[0], start_guess)
    cleaned_angle = solution_cleanup(arm, initial_angle)
    angles.append(cleaned_angle)

    for i in range(len(point_set) - 1):
        current_point = point_set[i + 1]

        angles.append(getAngleOG(arm, current_point,angles[i]))
        if i % printInterval == 0:
            print("Current at " + str(i))

    return angles

def get_pointset_angles(arm, point_set, start_guess=None):
    printInterval = int(3000 / len(arm))
    angles = []
    initial_angle = getAngle_LM(arm, point_set[0], start_guess)
    cleaned_angle = solution_cleanup(arm, initial_angle)
    angles.append(cleaned_angle)

    for i in range(len(point_set) - 1):
        current_point = point_set[i + 1]

        angles.append(getAngle_LM(arm, current_point,angles[i]))
        if i % printInterval == 0:
            print("Current at " + str(i))

    return angles

def get_solution_error(curTransform, targetPos, targetR):
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

    return error

def get_invJ(jacobian, lam):
    return jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + (lam**2) * np.eye(6))

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