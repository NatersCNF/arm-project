
# redoing physics, ignore the methods that are commented out
def find_arm_forces(scientific_Arm, accelerations):
    forces = []
    for scientific_link, acceleration in zip(scientific_Arm, accelerations):
        forces.append(scientific_link.mass * acceleration)
    return forces

def get_all_forces(scientific_Arm, acceleration_set):
    forces = []
    for accelerations in acceleration_set:
        frame_accelerations = find_arm_forces(scientific_Arm, accelerations)
        forces.append(frame_accelerations)
    
    return forces

def get_joint_velocities(Arm, valueSet, PPs, v_i=None,v_f=None):
    arm_positions = get_all_joint_pos(Arm,valueSet)
    interval = 1 / PPs
    if v_i is None:
        v_i = np.array([0, 0, 0])
    
    if v_f is None:
        v_f = v_i

    joint_num = len(Arm)
    
    velocities = []
    v_i = np.tile(v_i, (joint_num, 1, 1))
    v_f = np.tile(v_f, (joint_num, 1, 1))
    velocities.append(v_i)

    prev_pos = arm_positions[:-1, :, :, :]
    pos = arm_positions[1:, :, :, :]

    displacement = pos - prev_pos
    rate = displacement / interval

    velocities.extend(rate)
    velocities.append(v_f)

    return velocities

def get_joint_accelerations(velocities,PPs):
    interval = 1 / PPs
    prev_velocity = velocities[:-1, :, :, :]
    velocity = velocities[1:, :, :, :]

    diff = velocities - prev_velocity
    acceleration = diff / interval
    return acceleration

def get_arm_energy(Arm, valueSet, PPs, scientific_Arm, angular=None, velocity=None,height_reference="zero"): # frame, joint, value (valueset)
    if height_reference == "zero":
        reference = 0
    
    W = get_kinematic(Arm,valueSet=valueSet, PPs=PPs)
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
            link_energy = scientificlink.link_energy(joint_height, joint_angular, joint_velocity, reference)
            frame_energy.append(link_energy)
        all_energy.append(frame_energy)

    return all_energy