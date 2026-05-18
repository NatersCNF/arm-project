import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6 import QtCore

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from arm import getAllPos, getAllRotPos, getRotationMatrices, prismatic_indices
from physics import *

def dispPath(Arm, Angles, Points, PPs, keyPos,L_unit=None,rot_unit=None):
    print("Done calculating!")
    print("")
    print("Which display method would you like to use?")
    print("1. PyQTgraph")
    print("2. MatPlotLib")


    while True:
        #usr = input()
        usr = "1"

        if usr == "":
            break

        elif usr == "1":
            dispPyqt(Arm=Arm, PSangles=Angles, Points=Points,keyPos=keyPos,PPs=PPs,L_unit=L_unit,rot_unit=rot_unit)
            break

        elif usr == "2":
            dispMatplot(Arm=Arm, PSangles=Angles, Points=Points,keyPos=keyPos,PPs=PPs)
            break


# PyQTgraph main function & sub-functions
jointR = 0.3

def dispPyqt(Arm, PSangles, Points, keyPos, margin=2,PPs=30,L_unit=None,rot_unit=None):   
    axisStroke = 3
    
    show_markers = False
    show_pie = False
    show_text = True
    color_joints = True

    keyPos = np.array(keyPos)[:, 0:3]
    pathPoints = np.array(Points)[:, :3]
    armPositions = np.array([getAllPos(Arm, angle) for angle in PSangles])
    endPos = armPositions[:, -1, :]

    prismatic_index = prismatic_indices(Arm)

    link_directions = np.array(get_all_arm_properties(arm_positions=armPositions,type="unit"))

    prismatic_signs = []
    for p_index in prismatic_index:
        z_direction = link_directions[:, p_index, :]

        vertical_sign = np.sign(z_direction[:, 2])
        prismatic_signs.append(vertical_sign)
    

    app = pg.mkQApp("Robot Arm")
    view = gl.GLViewWidget()
    view.show()

    clip_offset = 0.01 # offset to avoid clipping between the pies and the end circles
    text_offset = 0.5 # distance from the side of the joint to the text

    jointPoints = np.array(getRotAxisPoints(Arm=Arm,valueSet=PSangles,armPositions=armPositions))
    jointPoints_out = np.array(getRotAxisPoints(Arm=Arm,valueSet=PSangles,armPositions=armPositions,offset=clip_offset))

    jointP1set, jointP2set = jointPoints[:, :, 0, :], jointPoints[:, :, 1, :]
    linkP1set, linkP2set = armPositions[:, :-1, :], armPositions[:, 1:, :]

    #    path
    pathLine = gl.GLLinePlotItem(pos=pathPoints,color=(1.0, 0.65, 0.0, 1.0),width=1)
    armLine = gl.GLLinePlotItem(pos=armPositions[0],color=(0.5, 0.8, 1.0, 1.0),width=1)
    view.addItem(pathLine)
    view.addItem(armLine)

    #    arm links
    linkCylinders = makeMeshHelperSet(P1set=linkP1set[0], P2set=linkP2set[0], view=view, type="link")
    
    #  joints
    # cylinders
    pie_res = 100
    
    scientific_arm = get_cylinder_physics_arm(Arm=Arm,radius=jointR, kg_m=1)
    #scientific_arm = get_simple_physics_arm(Arm, 1)
    torque = get_all_torque(Arm,scientific_arm,PSangles,PPs,end_effector_mass=10)
    torque = np.linalg.norm(torque, axis=2)

    if color_joints:
        color_property = torque
        joint_colors = get_color_intensity(values=color_property,absolute_mode=True,mode="local",prismatic_index=prismatic_index)
        jointCylinders = makeMeshHelperSet(P1set=jointP1set[0], P2set=jointP2set[0], view=view, type="joint", colors=joint_colors[0])
    
    else:
        jointCylinders = makeMeshHelperSet(P1set=jointP1set[0], P2set=jointP2set[0], view=view, type="joint")

    # markers
    if show_markers:
        markers = jointLines(Arm=Arm, valueSet=PSangles, jointPoints=jointPoints_out, armPositions=armPositions)
        markerLine = gl.GLLinePlotItem(pos=markers[0], color=(1, 1, 0, 1), width=2, mode='lines')
        view.addItem(markerLine)
    
    # pies
    if show_pie:
        arcSet = getArcSet(Arm=Arm,valueSet=PSangles,jointPoints=jointPoints_out,armPositions=armPositions,resolution=pie_res)
        face_colors = get_colors(valueSet=PSangles,arc_set=arcSet,Arm=Arm,resolution=pie_res)
        arcFaces = getArcFaceSet(arcSet)
        arc1_meshes, arc2_meshes = pieMeshMaker(arcSet, arcFaces, view, face_colors)

    # text
    if show_text:
        text_value = torque
        text_points = np.array(adjusted_text_points(Arm=Arm,valueSet=PSangles,armPositions=armPositions,offset=text_offset))
        #joint_text = make_joint_text(Arm=Arm,valueSet=text_value,L_unit=L_unit,rot_unit=rot_unit)
        joint_text = make_joint_text(Arm=Arm,valueSet=text_value,other_unit="Nm")
        joint_text_items = make_text_item_set(text_points=text_points,text_valueSet=joint_text, view=view)

        

    #    endeffector
    sphereData = gl.MeshData.sphere(rows=20,cols=20,radius=(jointR * 1.1))
    sphereMesh = gl.GLMeshItem(meshdata=sphereData,smooth=True,color=(1, 0.2, 0.2, 1),shader='shaded')
    sphereMesh.translate(*endPos[0])
    view.addItem(sphereMesh)

    origins = getJointOrigin(Arm=Arm,valueSet=PSangles,length=2)
    xOriginLine = gl.GLLinePlotItem(pos=origins[0][0], color=(1, 0, 0, 1), width=axisStroke, mode='lines')
    yOriginLine = gl.GLLinePlotItem(pos=origins[0][1], color=(0, 1, 0, 1), width=axisStroke, mode='lines')
    zOriginLine = gl.GLLinePlotItem(pos=origins[0][2], color=(0, 0, 1, 1), width=axisStroke, mode='lines')
    
    view.addItem(xOriginLine)
    view.addItem(yOriginLine)
    view.addItem(zOriginLine)

    state = {'curF': 0}

    def update():
        f = state['curF']
        if show_markers:
            markerLine.setData(pos=markers[f])
        
        xOriginLine.setData(pos=origins[f][0])
        xOriginLine.setData(pos=origins[f][0])
        yOriginLine.setData(pos=origins[f][1])
        zOriginLine.setData(pos=origins[f][2])

        armLine.setData(pos=armPositions[f])

        sphereMesh.resetTransform()
        sphereMesh.translate(*endPos[f])

        for i in range(len(linkCylinders)):
            current_cylinder = linkCylinders[i]
            P1, P2 = linkP1set[f][i], linkP2set[f][i]
            

            if i in prismatic_index:
                index = prismatic_index.index(i)
                view.removeItem(current_cylinder)
                
                if prismatic_signs[index][f] < 0:
                    new_cylinder = getCylinderMesh(P2, P1, "link")
                else:
                    new_cylinder = getCylinderMesh(P1, P2, "link")

                view.addItem(new_cylinder)
                linkCylinders[i] = new_cylinder
            
            else:
                applyTransform(current_cylinder, P1, P2)

        
        if show_pie:
            for i in range(len(arcSet[0][f])):
                new_arc1_verts = arcSet[0][f][i]
                new_arc2_verts = arcSet[1][f][i]
                new_faces = arcFaces[f][i]
                new_colors = face_colors[f][i] if face_colors else None

                arc1_meshes[i].setMeshData(vertexes=new_arc1_verts, faces=new_faces, faceColors=new_colors)
                arc2_meshes[i].setMeshData(vertexes=new_arc2_verts, faces=new_faces, faceColors=new_colors)

        if show_text:
            for i in range(len(joint_text_items)):
                new_text = joint_text[f][i]
                
                new_text = joint_text[f][i]
                new_points = text_points[f][i]
                
                joint_text_items[i].setData(pos=new_points, text=new_text)
            
        for i in range(len(jointCylinders)):
            P1, P2 = jointP1set[f][i], jointP2set[f][i]

            applyTransform(jointCylinders[i][0], P1, P2)
            applyTransform(jointCylinders[i][1], P1, P2)
            applyTransform(jointCylinders[i][2], P2, P1)
            
            if color_joints:
                color = joint_colors[f][i]
                jointCylinders[i][0].setColor(color)
                jointCylinders[i][1].setColor(color)
                jointCylinders[i][2].setColor(color)

        state['curF'] = (f + 1) % len(armPositions)
    
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(int(1000/PPs))

    zgrid = gl.GLGridItem()
    view.addItem(zgrid)

    xMax = max(np.max(np.abs(armPositions[:, :, 0])), np.max(np.abs(pathPoints[:, 0]))) + margin
    yMax = max(np.max(np.abs(armPositions[:, :, 1])), np.max(np.abs(pathPoints[:, 1]))) + margin
    zMax = max(np.max(np.abs(armPositions[:, :, 2])), np.max(np.abs(pathPoints[:, 2]))) + margin

    x_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0], [xMax,0,0]]), color=(1,0,0,1), width=axisStroke)
    y_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,yMax,0]]), color=(0,1,0,1), width=axisStroke)
    z_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,zMax]]), color=(0,0,1,1), width=axisStroke)

    view.addItem(x_axis)
    view.addItem(y_axis)
    view.addItem(z_axis)

    app.exec()

def getCylinderMesh(P1, P2, type, color=None, resolution=20):
    vec = P2 - P1
    normLength = np.linalg.norm(vec)
    short_link = 0.0000001
    if normLength < short_link:
        normLength = short_link

    link_length = normLength

    linkageR = 0.2

    if type == "link":
        if color is None:
            color=(0.5, 0.8, 1.0, 1.0)
        radiusC = linkageR
        Data = gl.MeshData.cylinder(rows=1, cols=20, radius=[radiusC, radiusC], length=link_length)
    
    elif type == "joint":
        if color is None:
            color=(0.6, 0.98, 0.6, 1.0)
        radiusC = jointR
        Data = gl.MeshData.cylinder(rows=1, cols=resolution, radius=[radiusC, radiusC], length=link_length)
    
    elif type == "circle":
        if color is None:
            color=(0.6, 0.98, 0.6, 1.0)
        radiusC = jointR
        Data = gl.MeshData.cylinder(rows=1, cols=resolution, radius=[radiusC, 0], length=0.01)
    
    
    if normLength == 0 and type != "circle":
        return None
    
    mesh = gl.GLMeshItem(meshdata=Data, smooth=True, color=color,shader='shaded')

    zAxis = np.array([0, 0, 1])
    target = vec / normLength

    cross = np.cross(zAxis, target)
    dot = np.dot(zAxis, target)
    angle = np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))

    if np.linalg.norm(cross) > 1e-6:
        mesh.rotate(angle, *cross)
    
    mesh.translate(*P1)
    
    return mesh

def makeMeshHelperSet(P1set, P2set, view, colors=None, type="link",resolution=20):
    helper = []

    numCylinder = len(P1set)
    if type == "joint":
        numCylinder -= 1

    for i in range(numCylinder):
            P1 = P1set[i]
            P2 = P2set[i]
            if colors is None:
                color = None
            
            else:
                color = colors[i]
            linkMesh = getCylinderMesh(P1, P2, type, resolution=resolution, color=color)
            
            if linkMesh:
                view.addItem(linkMesh)
                if type == "joint":
                    circle1 = getCylinderMesh(P1, P2, "circle", color)
                    circle2 = getCylinderMesh(P2, P1, "circle", color)

                    view.addItem(circle1)
                    view.addItem(circle2)

                    helper.append([linkMesh, circle1, circle2])

                else:
                    helper.append(linkMesh)
                    
    return np.array(helper,dtype=object)

def getRotAxisPoints(Arm,valueSet,armPositions=None,cLength=0.5,offset=None):
    if armPositions is None:
        armPositions = np.array([getAllPos(Arm, angle) for angle in valueSet])

    if offset is not None:
        cLength += offset
        
    halfLen = cLength / 2
    allRotAxes = getAllRotPos(Arm,valueSet)
    jointPointSet = []
    
    for frameAxes, framePos in zip(allRotAxes, armPositions):
        frame = []
        ground = np.array(framePos[0])
        groundAxis = np.array([0, 0, 1])
        groundAxis = groundAxis * halfLen
        start = ground + groundAxis
        end = ground + groundAxis
        frame.append([ground + groundAxis, ground - groundAxis])

        for i in range(len(frameAxes)):
            pos = np.array(framePos[i + 1])
            rotAxis = np.array(frameAxes[i])

            modified = rotAxis * halfLen

            start = pos + modified
            end = pos - modified

            frame.append([start, end])
        
        jointPointSet.append(frame)
    
    return jointPointSet

def applyTransform(meshItem, P1, P2, color=None):
    vec = P2 - P1
    normLength = np.linalg.norm(vec)
    meshItem.resetTransform()

    if normLength > 0:
        zAxis = np.array([0, 0, 1])
        target = vec / normLength
        dot = np.dot(zAxis, target)
        
        if dot < -0.999999:
            meshItem.rotate(180, 0, 1, 0)
        else:
            cross = np.cross(zAxis, target)
            angle = np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))

            if np.linalg.norm(cross) > 1e-6:
                meshItem.rotate(angle, *cross)
        meshItem.translate(*P1)

def jointLines(Arm,valueSet,jointPoints, armPositions,vecLength=None,mode="moved"):
    if vecLength is None:
        vecLength = jointR

    frame_with_base = [[1.0, 0.0, 0.0]]
    uVec = get_all_arm_properties(arm_positions=armPositions,type="unit")

    armDirections = []
    for frame_uvecs in uVec:
        armDirections.append(frame_with_base + frame_uvecs)

    armDirections = np.array(armDirections) * vecLength
    rotationMatrices = getRotationMatrices(arm=Arm,valueSet=valueSet)

    lines = []
    for frame in range(len(valueSet)):
        frameLines = []
        numJoints = rotationMatrices.shape[1]
        
        for i in range(numJoints):
            P1 = jointPoints[frame][i][0]
            P2 = jointPoints[frame][i][1]

            

            vecIn = armDirections[frame][i]
            vecOut = rotationMatrices[frame][i][:3, 0] * vecLength

            angle = valueSet[frame][i]
            joint_axis = P2 - P1
            u = joint_axis / np.linalg.norm(joint_axis)
            v = vecOut
            uv_cross = np.cross(u, v)
            uv_dot = np.dot(u, v)
            new_vec = (v * np.cos(angle)) + (uv_cross * np.sin(angle)) + (u * uv_dot * (1 - np.cos(angle)))

            vecIn = new_vec


            if mode == "moved":
                frameLines.extend([P1, P1 + vecIn, P2, P2 + vecIn])
                frameLines.extend([P1, P1 + vecOut, P2, P2 + vecOut])

            elif mode == "out":
                frameLines.append([vecIn, vecOut])

        lines.append(np.array(frameLines))
    return lines

def getArcSet(Arm,valueSet,jointPoints=None, armPositions=None,radius=None,resolution=50): # resolution is points per circle
    if radius is None:
        radius = jointR
    frameNum = len(valueSet)
    if armPositions is None:
        armPositions = np.array([getAllPos(Arm, angle) for angle in valueSet])
    
    if jointPoints is None:
        jointPoints = np.array(getRotAxisPoints(Arm=Arm,valueSet=valueSet,armPositions=armPositions))
    
    prismatic_index = prismatic_indices(Arm)
    jointNum = len(jointPoints[0])

    lines = np.array(jointLines(Arm,valueSet,jointPoints, armPositions, vecLength=1, mode="out"))
    
    jointAxes = getAllRotPos(Arm,valueSet)
    P1set, P2set = jointPoints[:, :, 0, :], jointPoints[:, :, 1, :]
    static_vec = lines[:, :, 0, :]

    arcPoints = []
    for i in range(frameNum):
        frameArc = []
        frameAxes = jointAxes[i]
        frameAxes.insert(0,[0,0,1])
        frameStatic = static_vec[i]

        for j in range(jointNum - 1):
            if j in prismatic_index:
                continue

            joint_dir = frameAxes[j]
            joint_static = frameStatic[j]
            joint_static = joint_static / np.linalg.norm(joint_static)

            extraDir = np.linalg.cross(joint_dir, joint_static)
            extra_length = np.linalg.norm(extraDir)
            
            if extra_length != 0:
                extraDir = extraDir / np.linalg.norm(extraDir)

            joint_angle = valueSet[i][j]
            if abs(joint_angle) > (2 * np.pi):
                angle = np.sign(joint_angle) * 2 * np.pi
                angle_offset = joint_angle - angle
            
            else:
                angle = joint_angle
                angle_offset = 0

            rotations = abs(angle) / (2 * np.pi)

            numPoints = int(np.ceil(resolution * rotations))

            if numPoints < 2:
                numPoints = 2

            interval = angle / (numPoints - 1)
            

            arc = [np.array([0, 0, 0])]
            for k in range(numPoints):
                pointAngle = (interval * k) + angle_offset
                P = (joint_static * radius * np.cos(pointAngle)) + (extraDir * radius * np.sin(pointAngle))
                arc.append(P)
            frameArc.append(arc)
        arcPoints.append(frameArc)
    
    arc1_set = []
    arc2_set = []
    for frameP1, frameP2, frameArcs in zip(P1set,P2set,arcPoints):
        frame1 = []
        frame2 = []
        for P1, P2, jointArc in zip(frameP1, frameP2, frameArcs):
            arcs = np.array(jointArc)

            arc1 = arcs + P1
            arc2 = arcs + P2

            frame1.append(arc1)
            frame2.append(arc2)
        arc1_set.append(frame1)
        arc2_set.append(frame2)
    
    return [arc1_set, arc2_set]

def getArcFaceSet(arcSet):
    arc1_set, arc2_set = arcSet[0], arcSet[1]

    faces = []
    for frame in range(len(arc1_set)):
        frame_faces = []
        frame_arcs = arc1_set[frame]

        for i in range(len(frame_arcs)):
            joint_faces = []
            joint_arc = frame_arcs[i]
            num_vertices = len(joint_arc)
            num_arcP = num_vertices - 1
            num_tris = num_arcP - 1

            for k in range(num_tris):
                joint_faces.append([0, (k + 1), (k + 2)])

            #joint_faces.append([0, num_arcP, 1])
            frame_faces.append(joint_faces)
            
        faces.append(frame_faces)

    return faces

def pieMeshMaker(arc_set,arc_faces,view, colors=None):
    startFrame1_arcs, startFrame2_arcs = arc_set[0][0], arc_set[1][0]
    start_faces = arc_faces[0]

    if colors is not None:
        initial_color = colors[0]
        #initial_color = [1, 0.8, 0, 0.5]

    arc1_meshes = []
    arc2_meshes = []

    for joint_arc1, joint_arc2, joint_faces, joint_colors in zip(startFrame1_arcs, startFrame2_arcs, start_faces, initial_color):
        arc1_meshdata = gl.MeshData(vertexes=joint_arc1, faces=joint_faces)
        arc2_meshdata = gl.MeshData(vertexes=joint_arc2, faces=joint_faces)

        if colors is None:
            arc1_meshdata = gl.MeshData(vertexes=joint_arc1, faces=joint_faces)
            arc2_meshdata = gl.MeshData(vertexes=joint_arc2, faces=joint_faces)

            arc1_mesh = gl.GLMeshItem(meshdata=arc1_meshdata, color=(1, 0.8, 0, 0.5), smooth=False)
            arc2_mesh = gl.GLMeshItem(meshdata=arc2_meshdata, color=(1, 0.8, 0, 0.5), smooth=False)
        
        else:
            arc1_meshdata = gl.MeshData(vertexes=joint_arc1, faces=joint_faces, faceColors=joint_colors)
            arc2_meshdata = gl.MeshData(vertexes=joint_arc2, faces=joint_faces, faceColors=joint_colors)

            arc1_mesh = gl.GLMeshItem(meshdata=arc1_meshdata, smooth=False)
            arc2_mesh = gl.GLMeshItem(meshdata=arc2_meshdata, smooth=False)

        view.addItem(arc1_mesh)
        view.addItem(arc2_mesh)

        arc1_meshes.append(arc1_mesh)
        arc2_meshes.append(arc2_mesh)
    
    return arc1_meshes, arc2_meshes

def get_colors(valueSet, arc_set, Arm, min_color=None, max_color=None, resolution=50):
    valueSet = np.array(valueSet)
    prismatic_index = prismatic_indices(Arm)
    face_per_circle = resolution - 1
    if min_color is None:
        min_color = [1, 0.8, 0, 0.5]
    min_color = np.array(min_color)

    if max_color is None:
        max_color = [1, 0, 0, 0.5]
    max_color = np.array(max_color)

    color_diff = max_color - min_color

    value_abs = np.abs(valueSet)

    max_angle = np.max(value_abs)
    if max_angle == 0:
        max_angle = 2 * np.pi

    arc_set = arc_set[0]

    colors = []
    for frame in range(len(arc_set)):
        frame_color = []
        frame_values = valueSet[frame]

        for joint in range(len(frame_values)):
            if joint in prismatic_index:
                continue

            

            joint_angle = np.abs(frame_values[joint])
            full_rotation_check = joint_angle > (2 * np.pi)
            fraction = joint_angle / max_angle
            color_gradient = color_diff * fraction

            rotations = joint_angle / (2 * np.pi)
            full_rotations = np.floor(rotations)
            full_rotation_faces = full_rotations * face_per_circle

            total_face_num = max(1, int(np.ceil(face_per_circle * rotations)))
            arc_faces = min(face_per_circle, total_face_num)
            
            color_step = color_gradient / total_face_num

            joint_colors = []
            for i in range(arc_faces):
                face_color = min_color + color_step * i
                joint_colors.append(face_color)
        
            if full_rotation_check:
                joint_colors = np.array(joint_colors)
                roll_over = total_face_num - full_rotation_faces
                color_offset = np.array(color_step * roll_over)
                joint_colors = [color + color_offset for color in joint_colors]

            frame_color.append(joint_colors)
        colors.append(frame_color)
    
    return colors

def pathLength(armPositions):
    endPos = armPositions[:,-1:,:]
    distanceTravelled = 0
    for i in range(1, len(armPositions)):
        oldPos = endPos[i - 1]
        pos = endPos[i]
        vec = pos - oldPos
        displacement = np.linalg.norm(vec)
        distance += displacement
    return distanceTravelled

def getJointOrigin(Arm, valueSet, armPositions=None,length=2,joint=-1):
    if armPositions is None:
        armPositions = np.array([getAllPos(Arm, angle) for angle in valueSet])
    
    rotationMatrices = getRotationMatrices(arm=Arm,valueSet=valueSet)
    
    jointRotation = rotationMatrices[:, joint, :, :] * length
    jointPos = armPositions[:, joint, :]

    origins = []
    for pos, dir in zip(jointPos,jointRotation):
        xOrigin = pos + dir[:, 0]
        yOrigin = pos + dir[:, 1]
        zOrigin = pos + dir[:, 2]

        xVec = np.array([pos, xOrigin])
        yVec = np.array([pos, yOrigin])
        zVec = np.array([pos, zOrigin])


        origins.append([xVec, yVec, zVec])

    return origins

def adjusted_text_points(Arm, valueSet, armPositions=None, offset=0.5, default_direction="out"):
    # default point choice for any non-prismatic joints. "out" picks the one further from the origin, and "in" picks the one closest to the origin
    # offset is the distance of the text from the joint circle
    frame_num = len(valueSet)
    text_points = np.array(getRotAxisPoints(Arm=Arm,valueSet=valueSet,armPositions=armPositions,offset=offset))
    prismatic_index = prismatic_indices(Arm)

    single_point_set = []
    for frame in range(frame_num):
        adjusted_frame_points = []
        frame_text_points = text_points[frame]
        frame_positions = armPositions[frame]

        joint_num = len(frame_text_points)

        for joint in range(joint_num):
            P1 = frame_text_points[joint][0]
            P2 = frame_text_points[joint][1]
            prev_joint = joint - 1

            if joint in prismatic_index or prev_joint in prismatic_index:

                if joint in prismatic_index:
                    pos = frame_positions[joint + 1]
                
                else:
                    pos = frame_positions[joint - 1]
                
                P1_diff = pos - P1
                P2_diff = pos - P2

                P1_distance = np.linalg.norm(P1_diff)
                P2_distance = np.linalg.norm(P2_diff)
                
                if P1_distance > P2_distance:
                    point = P1
                
                else:
                    point = P2

            else:
                if joint + 1 != joint_num:
                    next_joint_pos = frame_positions[joint + 1]
                    P1_distance = next_joint_pos - P1
                    P2_distance = next_joint_pos - P2

                    P1_distance = np.linalg.norm(P1_distance)
                    P2_distance = np.linalg.norm(P2_distance)

                    if (P1_distance - P2_distance) > 0.5:
                        adjusted_frame_points.append(P1)
                        continue
                    
                    elif (P2_distance - P1_distance) > 0.5:
                        adjusted_frame_points.append(P2)
                        continue

                P1_d2ground = np.linalg.norm(P1)
                P2_d2ground = np.linalg.norm(P2)

                difference = np.abs(P2_d2ground) - np.abs(P1_d2ground)

                if difference > 0.0001:
                    if P1_d2ground > P2_d2ground:
                        in_point = P2
                        out_point = P1
                    
                    else:
                        in_point = P1
                        out_point = P2
                    
                else:
                    in_point = P1
                    out_point = P2

                if default_direction == "out":
                    point = out_point
                elif default_direction == "in":
                    point = in_point
                else:
                    raise ValueError("dunno what happened")
                
            adjusted_frame_points.append(point)
        
        single_point_set.append(adjusted_frame_points)

    return single_point_set

def make_joint_text(Arm, valueSet, L_unit=None, rot_unit=None, other_unit=None):
    local_max = np.max(valueSet, axis=0)
    if L_unit is None:
        L_unit="u"
    
    if rot_unit is None:
        rot_unit="rad"

    if other_unit is not None:
        other_text = " " + other_unit

    valueSet = np.array(valueSet)
    prismatic_index = prismatic_indices(Arm)

    d_unit_text = " " + L_unit

    if rot_unit == "rad":
        rot_unit_text = " rads"
        modifier = 1
    
    elif rot_unit == "deg":
        rot_unit_text = " \u00b0"
        modifier = 180 / np.pi
    
    elif rot_unit == "rev":
        rot_unit_text = " revs"
        modifier = 1 / (2 * np.pi)

    text = []
    for frame_value in valueSet:
        frame_text = []

        for i, joint_value in enumerate(frame_value):
            joint_max = round(local_max[i], 2)
            if other_unit is None:
                if i in prismatic_index:
                    joint_value = round(joint_value, 2)
                    joint_text = str(joint_value) + d_unit_text

                else:
                    num_value = joint_value * modifier
                    num_value = round(num_value, 2)
                    joint_text = str(num_value) + rot_unit_text
            
            else:
                if i in prismatic_index:
                    joint_text = ""

                else:
                    num_value = round(joint_value, 2)
                    joint_text = str(num_value) + other_text + ", max = " + str(joint_max) + other_text

            frame_text.append(joint_text)
        text.append(frame_text)
    
    return text

def make_text_item_set(text_points, text_valueSet, view):
    text_points = np.array(text_points)
    initial_values = text_valueSet[0]
    initial_pos = text_points[0, :, :]

    text_items = []
    for joint_initial_pos, joint_initial_text in zip(initial_pos, initial_values):
        point_text = gl.GLTextItem(pos=joint_initial_pos,text=joint_initial_text)
        view.addItem(point_text)
        text_items.append(point_text)
    return text_items

def get_color_intensity(values, neutral_color=None, min_color=None, max_color=None, absolute_mode=True, mode="global", prismatic_index=None): # neutral color is 0
    # mode="local" means per joint LOCAL min max, mode="global" means global min/max
    # absolute_mode True uses absolute values, 0 as the lowest, and False means there could be a negative minimum, using neutral_color as 0
    values = np.array(values)

    if prismatic_index is not None:
        for index in prismatic_index:
            values = np.delete(values, index, axis=1)

    frame_num = len(values)

    abs_values = np.abs(values)

    local_min = np.min(values, axis=0)
    local_max = np.max(values, axis=0)
    local_abs_max = np.max(abs_values, axis=0)
    joint_num = len(local_abs_max)           

    global_min = np.min(local_min)
    global_max = np.max(local_max)
    global_abs_max = np.max(local_abs_max)

    if neutral_color is None:
        neutral_color = [0.6, 0.98, 0.6, 1.0]
    neutral_color = np.array(neutral_color)

    if max_color is None:
        max_color = [1.0, 0, 0, 1.0]
    max_color = np.array(max_color)

    color_diff_up = max_color - neutral_color

    if not absolute_mode:
        if min_color is None:
            min_color = [0, 0, 1.0, 1.0]
        min_color = np.array(min_color)
        color_diff_down = min_color - neutral_color
    
    if mode == "global":
        local_min = np.tile(global_min, joint_num)
        local_max = np.tile(global_max, joint_num)
        local_abs_max = np.tile(global_abs_max, joint_num)
    
    colors = []
    for i in range(frame_num):
        frame_colors = []
        for j in range(joint_num):
            if absolute_mode:
                current_value = abs_values[i][j]
                joint_max = local_abs_max[j]

                if joint_max == 0:
                    color = neutral_color.copy()
                
                else:
                    value_fraction = current_value / joint_max
                    color = neutral_color + (color_diff_up * value_fraction)

            else:
                current_value = values[i][j]

                color = neutral_color.copy()

                if current_value > 0:
                    joint_max = local_max[j]

                    if joint_max == 0:
                        color = neutral_color.copy()
                    else:
                        value_fraction = abs(current_value / joint_max)
                        color += color_diff_up * value_fraction
                
                elif current_value < 0:
                    joint_min = local_min[j]
                    if joint_min == 0:
                        color = neutral_color.copy()
                    else:
                        value_fraction = abs(current_value / joint_min)
                        color += color_diff_down * value_fraction

            frame_colors.append(np.clip(color, 0.0, 1.0).tolist())
        colors.append(frame_colors)
    return colors


# MatPlotLib main function
def dispMatplot(Arm, PSangles, Points, keyPos, PPs=30,margin=2,labelToggle=True,keyToggle=False):
    Points = np.array(Points)
    keyPoints = np.array(keyPos)
    keyPoints = keyPoints[:, 0:3]

    armPositions = []
    for angle in PSangles:
        armPositions.append(getAllPos(Arm,angle))
    
    midPoint = []
    armPointNum = len(armPositions[0])

    for position in armPositions:
        mid = []
        for i in range(armPointNum - 1):
            midCur = [0, 0, 0]
            midCur[0] = (position[i][0] + position[i + 1][0]) / 2
            midCur[1] = (position[i][1] + position[i + 1][1]) / 2
            midCur[2] = (position[i][2] + position[i + 1][2]) / 2
            mid.append(midCur)
        midPoint.append(mid)
    
    labels = []
    

    def updateRoboLine(frame, armPositions, line, labels, midPoint):
        pos = armPositions[frame]
        
        cleanPos = [list(p[:3]) for p in pos]
        cleanPos.insert(0, [0, 0, 0])
        pos = np.array(cleanPos)
        
        x = pos[:, 0]
        y = pos[:, 1]
        z = pos[:, 2]
        
        line.set_data(x, y)
        line.set_3d_properties(z)
        
        if labelToggle and labels:
            for i, label in enumerate(labels):
                frameMid = midPoint[frame][i]
                label.set_position((frameMid[0], frameMid[1]))
                label.set_3d_properties(frameMid[2], zdir=(1, 1, 0))
                
            return [line] + labels
        
        
        return [line]


    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    PathLim = np.array([0, 0, 0, 0, 0, 0]) #xmin, xmax, ymin, ymax, zmin, zmax

    armLim = np.array([0, 0, 0, 0, 0, 0])  # same as pathlim
    armPosNP = np.array(armPositions)      
    
    for i in range(3):
        PathLim[i * 2] = Points[:, i].min()
        PathLim[(i * 2) + 1] = Points[:, i].max()

        armLim[i * 2] = armPosNP[:, :, i].min()
        armLim[(i * 2) + 1] = armPosNP[:, :, i].max()
    
    xMin, xMax = min(PathLim[0], armLim[0]), max(PathLim[1], armLim[1])
    yMin, yMax = min(PathLim[2], armLim[2]), max(PathLim[3], armLim[3])
    zMin, zMax = min(PathLim[4], armLim[4]), max(PathLim[5], armLim[5])


    ax.set_xlim(xMin - margin, xMax + margin)
    ax.set_ylim(yMin - margin, yMax + margin)
    ax.set_zlim(zMin - margin, zMax + margin)

    ax.set_box_aspect([1,1,1])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    line, = ax.plot([], [], [], marker='o')

    xVec = Points[:, 0]
    yVec = Points[:, 1]
    zVec = Points[:, 2]

    xKey = keyPoints[:, 0]
    yKey = keyPoints[:, 1]
    zKey = keyPoints[:, 2]


    limit = 1000
    n = 10
    Points = np.arange(-limit, limit + n, n)
    
    if keyToggle:
        ax.scatter(xKey, yKey, zKey, color='orchid', s=50,edgecolors='black', linewidth=2, label='Keypoints', zorder=2)
    

    if labelToggle:
        for i in range(armPointNum - 1):
            mid = midPoint[0][i]
            startLabel = ax.text(mid[0], mid[1], mid[2], str(i+1), color='red')
            labels.append(startLabel)

    ax.plot(Points, 0, 0, color='r', linewidth=1)
    ax.plot(0, Points, 0, color='g', linewidth=1)
    ax.plot(0, 0, Points, color='b', linewidth=1)

    ax.plot(xVec, yVec, zVec)
    ax.set_title("Robot Arm & Path")

    

    intervalFix = 1000 / PPs

    ani = animation.FuncAnimation(
        fig,
        updateRoboLine,
        frames=len(armPositions),
        fargs=(armPositions, line, labels, midPoint),
        interval=intervalFix,
        blit=True
    )

    plt.show()