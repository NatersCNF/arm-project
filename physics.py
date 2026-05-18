import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6 import QtCore

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from arm import getAllTransform, getAllPos, get_all_joint_pos, getJacobian, get_invJ, getRotationMatrices


# gravity
g_vec = np.array([0, 0, -9.81]) # m/s^2, down in the z
g = 9.81


def get_kinematic(Arm,valueSet,PPs, v_i=None,w_i=None,v_f=None,w_f=None,loop=True):
    transforms = getAllTransform(Arm,valueSet)
    interval = 1 / PPs

    # initial velocity and angular at all joints in frame 0 (i) and frame -1 (f)
    if v_i is None or len(v_i) != 3:
        v_i = np.zeros(3)
    else:
        v_i = np.array(v_i)

    if w_i is None or len(w_i) != 3:
        w_i = np.zeros(3)
    else:
        w_i = np.array(w_i)

    # assume default value (0) or the specified initial conditions to the final frame
    if v_f is None or len(v_f) != 3 or loop is True:
        v_f = v_i
    else:
        v_f = np.array(v_f)

    if w_f is None or len(w_f) != 3 or loop is True:
        w_f = w_i
    else:
        w_f = np.array(w_f)
    
    W_i = np.array([
                [0, -w_i[2], w_i[1], v_i[0]],
                [w_i[2], 0, -w_i[0], v_i[1]],
                [-w_i[1], w_i[0], 0, v_i[2]],
                [0, 0, 0, 0]
            ])
    W_f = np.array([
                [0, -w_f[2], w_f[1], v_f[0]],
                [w_f[2], 0, -w_f[0], v_f[1]],
                [-w_f[1], w_f[0], 0, v_f[2]],
                [0, 0, 0, 0]
            ])
    
    joint_num = len(Arm)
    frame_num = len(transforms)

    full_frame_i = np.tile(W_i, (joint_num, 1, 1))
    full_frame_f = np.tile(W_f, (joint_num, 1, 1))

    full_frame_i = full_frame_i.tolist()
    full_frame_f = full_frame_f.tolist()


    all_W = []
    all_W.append(full_frame_i)

    for frame in range(frame_num - 1):
        frame_transforms = transforms[frame]
        next_frame_transforms = transforms[frame + 1]

        frame_W = []
        for joint in range(joint_num):
            joint_transform = frame_transforms[joint]
            joint_next_transform = next_frame_transforms[joint]

            relative_transform = joint_next_transform @ np.linalg.inv(joint_transform)

            translation = relative_transform[:3, 3]
            rotation = relative_transform[:3, :3]

            velocity = translation / interval
            rotation_dt = (rotation - np.eye(3)) / interval

            wx, wy, wz = rotation_dt[2, 1], rotation_dt[0, 2], rotation_dt[1, 0]     # w is meant to represent omgega
            vx, vy, vz = velocity

            W = np.array([
                [0, -wz, wy, vx],
                [wz, 0, -wx, vy],
                [-wy, wx, 0, vz],
                [0, 0, 0, 0]
            ])

            frame_W.append(W)
        all_W.append(frame_W)
    all_W.append(full_frame_f)
    return all_W

def get_acceleration(W,PPs):
    interval = 1 / PPs
    W = np.array(W)
    cur_W = W[:-1]
    next_W = W[1:]

    diff = next_W - cur_W
    H = diff / interval
    return H.tolist()

def get_motion_components(WorH): #  reutrns as vx, vy, yz, omegax, omegay, omegaz (or linear/angular acceleration)
    WorH = np.array(WorH)
    v = WorH[:, :, 0:3,3]

    wx = WorH[:, :, 2, 1]
    wy = WorH[:, :, 0, 2]
    wz = WorH[:, :, 1, 0]
    omega = np.stack([wx, wy, wz], axis=-1)

    return v, omega

class link_physics:
    def __init__(self, length, kg_m=None, m=None, radius=None, type="slender"):
        self.length = length
        
        if kg_m is None:
            kg_m = 5

        if m is None:
            m = kg_m * length
    
        self.mass = m

        if type == "slender":
            self.I_Cm = slenderbar_Iyy(self.mass,self.length)
            #self.I_joint = slenderbar_Iy1(self.mass,self.length)
            self.I_Cm_tensor = slenderbar_I_tensor(self.mass, self.length)
            self.Cm_joint = self.length / 2                         # distance of center of mass from previous joint

        elif type == "cylinder":
            self.I_Cm = cylinder_Iyy(self.mass,self.length, radius)
            self.I_joint = self.I_Cm + self.mass * ((self.length / 2) ** 2)
            self.I_Cm_tensor = cylinder_I_tensor(self.mass, self.length, radius)
            self.Cm_joint = self.length / 2
        
        else:
            raise ValueError("currently can't do " + type)
        
    def get_link_kinetic_energy(self,angular,velocity):
        angular = np.array(angular)
        velocity = np.array(velocity)

        T_linear = (1 / 2) * (self.mass * np.sum(velocity ** 2))
        T_rotational = (1 / 2) * (self.I_Cm * np.sum(angular ** 2))

        T = T_linear + T_rotational
        return T
    
    def get_gravitational_potential_energy(self, height, gravity=None, reference=None):
        if gravity is None:
            gravity = g

        if reference is None:
            reference = 0

        PE = self.mass * gravity * (height - reference)
        return PE
    
    def link_energy(self, height, angular, velocity,reference=None):
        T = self.get_link_kinetic_energy(angular,velocity)
        PE = self.get_gravitational_potential_energy(height,reference=reference)

        energy = T + PE

        return energy

    def link_force(self,acceleration):
        F = self.mass * acceleration
        return F
    
    def link_torque(self,angular_acceleration):
        torque = self.I_joint * angular_acceleration
        return torque
    
    def link_torque3D(self, angular_velocity, angular_acceleration, rotation_matrix):
        I_tensor_rotated = rotate_I_tensor(self.I_Cm_tensor,rotation_matrix)

        angular_velocity = np.array(angular_velocity)
        angular_acceleration = np.array(angular_acceleration)

        velocity_product = I_tensor_rotated @ angular_velocity
        acceleration_product = I_tensor_rotated @ angular_acceleration

        torque = acceleration_product + np.cross(angular_velocity, velocity_product)
        return torque

def get_arm_link_properties(points,type="normal"):
    cur_joint = np.array(points[:-1])
    next_joint = np.array(points[1:])
    vectors = next_joint - cur_joint

    if type == "normal":
        return vectors
    
    elif type == "unit":
        unit_vectors = []
        for vec in vectors:
            L = np.linalg.norm(vec)
            if round(L, 4) == 0:
                u_vec = np.zeros_like(vec)
            else:
                u_vec = vec / L
            unit_vectors.append(u_vec)
        return unit_vectors
    
    elif type == "mag" or type == "length":
        magnitudes = []
        for vec in vectors:
            L = np.linalg.norm(vec)
            magnitudes.append(L)
        return magnitudes
    
    else:
        raise ValueError("Unsupported property type")

def get_all_arm_properties(type, arm_positions=None, valueSet=None, Arm=None):
    if arm_positions is None:
        arm_positions = get_all_joint_pos(Arm,valueSet)

    properties = []
    for points in arm_positions:
        item = get_arm_link_properties(points,type)
        properties.append(item)
    
    return properties

def get_cylinder_physics_arm(Arm,radius,kg_m=None):
    points = getAllPos(Arm)
    lengths = get_arm_link_properties(points,type="length")

    physics_arm = []
    for length in lengths:
        link = link_physics(length=length,kg_m=kg_m,radius=radius,type="cylinder")
        physics_arm.append(link)
    return physics_arm

def get_simple_physics_arm(Arm,kg_m=None):
    points = getAllPos(Arm)
    lengths = get_arm_link_properties(points,type="length")

    physics_arm = []
    for length in lengths:
        link = link_physics(length=length,kg_m=kg_m,type="slender")
        physics_arm.append(link)
    return physics_arm

def slenderbar_Iyy(m,L):
    Iyy = (1/12) * m * (L ** 2)
    return Iyy

def cylinder_Iyy(m, L, r):
    return (m / 12) * (L ** 2) + (m / 4) * (r ** 2)

def slenderbar_Iy1(m,L):
    Iy1 = (1/3) * m * (L ** 2)
    return Iy1

def slenderbar_I_tensor(m, L):
    I_xx = 0
    I_yy = (m / 12) * (L ** 2)
    I_zz = (m / 12) * (L ** 2)
    
    I_tensor = np.zeros((3,3))
    I_tensor[0][0] = I_xx
    I_tensor[1][1] = I_yy
    I_tensor[2][2] = I_zz

    return I_tensor.tolist()

def cylinder_I_tensor(m, L, r): 
    I_xx = (m / 2) * (r ** 2)
    I_yy = (m / 12) * (L ** 2) + (m / 4) * (r ** 2)
    I_zz = (m / 12) * (L ** 2) + (m / 4) * (r ** 2)
    
    I_tensor = np.zeros((3, 3))
    
    I_tensor[0][0] = I_xx
    I_tensor[1][1] = I_yy
    I_tensor[2][2] = I_zz

    return I_tensor.tolist()

def rotate_I_tensor(I_tensor, rotation_matrix):
    I_tensor = np.array(I_tensor)
    rotation_matrix = np.array(rotation_matrix)
    rotation_transpose = rotation_matrix.T

    rotated_I = rotation_matrix @ I_tensor @ rotation_transpose
    return rotated_I

def get_all_CoM(Arm, scientific_Arm, valueSet):

    frame_num = len(valueSet)
    directions = get_all_arm_properties(Arm,valueSet,"unit")

    arm_positions = []
    for values in valueSet:
        pos = getAllPos(Arm,values)
        arm_positions.append(pos)

    CM_magnitude = []
    for physics_link in scientific_Arm:
        CM = physics_link.Cm_joint
        CM_magnitude.append(CM)
    
    CM_all = np.tile(CM_magnitude,(frame_num, 1))

    COM_pos = [] # frame, joint, coordinate of CoM for that joint
    for frame_CM, frame_pos, frame_directons in zip(CM_all, arm_positions, directions):
        frame_COM_pos = []
        for joint_CM, joint_pos, joint_direction in zip(frame_CM, frame_pos, frame_directons):
            vec = np.array(joint_direction) * joint_CM
            pos = joint_pos + vec
            frame_COM_pos.append(pos)
        COM_pos.append(frame_COM_pos)
    
    return COM_pos

def get_all_end_velocities(Arm, valueSet, PPs):
    dt = 1 / PPs
    all_velocities = []
    for i in range(1, len(valueSet)):
        q_current = np.array(valueSet[i])
        q_prev = np.array(valueSet[i - 1])
        q_dot = (q_current - q_prev) / dt

        jacobian = getJacobian(Arm, q_current)
        end_velocity = jacobian @ q_dot
        # linear is 0:3, angular is 3:6
        all_velocities.append(end_velocity.tolist())
    return all_velocities

def get_all_CoM_pos(Arm, scientific_Arm, valueSet, armPositions=None):
    if armPositions is None:
        joint_pos = np.array(get_all_joint_pos(Arm,valueSet))
    else:
        joint_pos = np.array(armPositions)

    joint_directions = np.array(get_all_arm_properties(type="unit",arm_positions=joint_pos,valueSet=valueSet))

    for i, scientific_link in enumerate(scientific_Arm):
        CoM_pos = scientific_link.Cm_joint
        joint_directions[:, i, :] *= CoM_pos
    
    CoM_positions = joint_pos[:, :-1, :] + joint_directions
    return CoM_positions

def get_CoM_kin(Arm, scientific_Arm, valueSet, PPs):
    joint_pos = np.array(get_all_joint_pos(Arm,valueSet))
    CoM_pos = get_all_CoM_pos(Arm, scientific_Arm, valueSet, joint_pos)

    dt = 1 / PPs

    CoM_pos_diff = np.diff(CoM_pos, axis=0)
    CoM_velocity = CoM_pos_diff / dt

    CoM_velocity_diff = np.diff(CoM_velocity, axis=0)
    CoM_acceleration = CoM_velocity_diff / dt

    CoM_velocity = np.concatenate([np.zeros((1, *CoM_velocity.shape[1:])), CoM_velocity], axis=0)
    CoM_acceleration = np.concatenate([np.zeros((2, *CoM_acceleration.shape[1:])), CoM_acceleration], axis=0)

    return CoM_velocity, CoM_acceleration

def get_all_end_pos(Arm, valueSet):
    pos = get_all_joint_pos(Arm, valueSet)
    pos = np.array(pos)
    end = pos[:, -1, :]
    return end.tolist()

def get_all_end_velocity(end_points, PPs):
    end_points = np.array(end_points)
    dt = 1 / PPs
    velocity = np.insert(np.diff(end_points, axis=0) / dt, 0, np.zeros(3), axis=0)
    return velocity.tolist()

def get_all_end_acceleration(velocity, PPs):
    dt = 1 / PPs
    velocity = np.array(velocity)
    zero = np.zeros((1, 3))
    acceleration = np.concatenate([zero, (np.diff(velocity, axis=0) / dt), zero], axis=0)
    return acceleration.tolist()

def get_all_torque(Arm, scientific_Arm, valueSet, PPs, end_effector_mass=0):
    end_gravity_force = g_vec * end_effector_mass
    end_pos = get_all_end_pos(Arm, valueSet)
    end_acceleration = np.array(get_all_end_acceleration(get_all_end_velocity(end_pos, PPs), PPs))
        
    all_W = get_kinematic(Arm,valueSet,PPs)
    all_H = get_acceleration(all_W,PPs)

    joint_angular_velocity = get_motion_components(all_W)[1]
    joint_angular_acceleration = get_motion_components(all_H)[1]

    frame_num = len(joint_angular_acceleration)
    joint_num = len(scientific_Arm)

    joint_pos = np.array(get_all_joint_pos(Arm,valueSet))
    CoM_pos = get_all_CoM_pos(Arm, scientific_Arm, valueSet, joint_pos)
    CoM_acceleration = (get_CoM_kin(Arm, scientific_Arm, valueSet, PPs))[1]
    rotation_matrices = getRotationMatrices(Arm,valueSet)

    total_torque = []
    for i in range(frame_num):
        frame_torque = []
        for j in range(joint_num):
            joint_torque_sum = np.zeros(3)
            current_joint_pos = joint_pos[i][j]

            for k in range(joint_num - 1, j - 1, -1):
                scientific_link = scientific_Arm[k]
                link_mass = scientific_link.mass

                joint_omega = joint_angular_velocity[i][k]
                joint_alpha = joint_angular_acceleration[i][k]

                moment_arm = CoM_pos[i][k] - current_joint_pos

                # inertial torque
                rotation_matrix = rotation_matrices[i][k]
                rotational_torque = scientific_link.link_torque3D(joint_omega, joint_alpha, rotation_matrix)

                # gravity moment torque
                gravity_force = g_vec * link_mass
                gravity_torque = np.cross(moment_arm, gravity_force)

                # Linear inertia
                linear_torque = np.cross(moment_arm, CoM_acceleration[i][k] * link_mass)
                
                #velocity_cross = np.cross(joint_omega, scientific_link.I_Cm_tensor @ joint_omega)
                #joint_torque_sum += velocity_cross

                joint_torque_sum += (rotational_torque + gravity_torque + linear_torque)
            
            if end_effector_mass != 0:
                end_moment_arm = end_pos[i] - current_joint_pos
                end_weight_torque = np.cross(end_moment_arm, end_gravity_force)
                
                end_force = end_acceleration[i] * end_effector_mass
                end_linear_torque = np.cross(end_moment_arm, end_force)

                joint_torque_sum += end_weight_torque + end_linear_torque
            
            frame_torque.append(joint_torque_sum.tolist())

        total_torque.append(frame_torque)

    return total_torque