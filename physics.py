import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6 import QtCore

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from arm import getAllTransform, getAllPos

# gravity
g = [0, 0, -9.81] # m/s^2, down in the z


def get_kinematic(Arm,valueSet,PPs, v_i=None,w_i=None,v_f=None,w_f=None,loop=True):
    transforms = getAllTransform(Arm,valueSet)
    interval = 1 / PPs

    # initial velocity and angular at all joints in frame 0 (i) and frame -1 (f)
    if v_i is None or len(v_i) != 3:
        v_i = np.zeros((1,3))
    
    if w_i is None or len(w_i) != 3:
        w_i = np.zeros((1,3))

    
    # assume default value (0) or the specified initial conditions to the final frame
    if v_f is None or len(v_f) != 3 or loop is True:
        v_f = v_i

    if w_f is None or len(w_f) != 3 or loop is True:
        w_f = w_i
    
    v_i = np.array(v_i)
    w_i = np.array(w_i)

    v_f = np.array(v_f)
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

def get_acceleration(W,interval):
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
    def __init__(self, length, direction, kg_m=None, m=None, type="slender"):
        self.length = length
        self.direction = direction
        
        if kg_m is None:
            kg_m = 5

        if m is None:
            m = kg_m * length
    
        self.mass = m

        if type == "slender":
            self.I_Cm = slenderbar_Iyy(self.mass,self.length)
            self.I_joint = slenderbar_Iy1(self.mass,self.length)
            self.Cm_joint = self.length / 2                         # distance of center of mass from previous joint
        
        else:
            raise ValueError("currently can't do anything but a slender bar")
        
    def get_link_kinetic_energy(self,angular,velocity):
        angular = np.array(angular)
        velocity = np.array(velocity)

        T_linear = (1 / 2) * (self.mass * velocity ** 2)
        T_rotational = (1 / 2) * (self.I_Cm * angular ** 2)

        T = T_linear + T_rotational
        return T
    
    def get_gravitational_potential_energy(self, height, gravity=np.linalg.norm(g)):
        PE = self.mass * gravity * height
        return PE
    
    def link_energy(self, height, angular, velocity):
        T = self.get_link_kinetic_energy(angular,velocity)
        PE = self.get_gravitational_potential_energy(height)

        energy = T + PE

        return energy


def get_arm_link_properties(Arm,values=None,type="normal"):
    points = np.array(getAllPos(arm=Arm,values=values))
    cur_joint = points[:-1]
    next_joint = points[1:]
    vectors = next_joint - cur_joint

    if type == "normal":
        return vectors
    
    elif type == "unit":
        unit_vectors = []
        for vec in vectors:
            L = np.linalg.norm(vec)
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

def get_all_arm_properties(Arm, valueSet,type):
    properties = []
    for values in valueSet:
        item = get_arm_link_properties(Arm,values,type)
        properties.append(item)
    
    return properties

def get_simple_physics_arm(Arm,kg_m=None):
    lengths = get_arm_link_properties(Arm=Arm,type="mag")
    directions = get_arm_link_properties(Arm=Arm,type="unit")

    physics_arm = []
    for length, direction in zip(lengths, directions):
        link = link_physics(length=length,direction=direction,kg_m=kg_m,)
        physics_arm.append(link)
    return physics_arm
           
def slenderbar_Iyy(m,L):
    Iyy = (1/12) * m * (L ** 2)
    return Iyy

def slenderbar_Iy1(m,L):
    Iy1 = (1/3) * m * (L ** 2)
    return Iy1

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

def get_arm_energy(Arm, valueSet,scientific_Arm, angular=None, velocity=None): # frame, joint, value (valueset)
    W = get_kinematic(Arm,valueSet=valueSet)
    motion_components = get_motion_components(W)

    if velocity is None:
        velocity = motion_components[0]

    if angular is None:
        angular = motion_components[1]

    arm_positions = []
    for values in valueSet:
        pos = getAllPos(Arm,values)
        arm_positions.append(pos)

    CoM_pos = np.array(get_all_CoM(Arm, scientific_Arm, valueSet))

    heights = CoM_pos[:, :, 2] #only z

    all_energy = []
    for frame_velocity, frame_angular, frame_height in zip(velocity, angular, heights):
        frame_energy = []
        for joint_velocity, joint_angular, joint_height, scientificlink in zip(frame_velocity, frame_angular, frame_height, scientific_Arm):
            link_energy = scientificlink.link_energy(joint_height, joint_angular, joint_velocity)
            frame_energy.append(link_energy)
        all_energy.append(frame_energy)

    return all_energy
            
    













