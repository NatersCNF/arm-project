import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6 import QtCore

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from arm import getAllTransform

def get_kinematic(Arm,valueSet,PPs):
    transforms = getAllTransform(Arm,valueSet)
    interval = 1 / PPs

    frameNum = len(transforms)

    all_W = []
    for frame in range(frameNum - 1):
        frame_transforms = transforms[frame]
        next_frame_transforms = transforms[frame + 1]

        frame_W = []
        for joint in range(len(frame_transforms)):
            joint_transform = frame_transforms[joint]
            joint_next_transform = next_frame_transforms[joint]

            diff = joint_next_transform - joint_transform
            diff = np.array(diff)
            W = diff / interval
            frame_W.append(W)
        all_W.append(frame_W)
    
    return all_W

def get_velocity(W,type="linear"):
    omega = zip(W[:,:,2,1], W[:,:,0,2], W[:,:,1,0])
    v = zip(W[:,:,3,0], W[:,:,3,1], W[:,:,3,2])

    if type == "linear":
        return v
    
    elif type == "angular":
        return omega