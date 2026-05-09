import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6 import QtCore

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from arm import getAllPos, getAllRotPos, getRotationMatrices

def dispPath(Arm, Angles, Points, PPs, keyPos):
    print("Done calculating!")
    print("")
    print("Which display method would you like to use?")
    print("1. PyQTgraph")
    print("2. MatPlotLib")


    while True:
        usr = input()

        if usr == "":
            break

        elif usr == "1":
            dispPyqt(Arm=Arm, PSangles=Angles, Points=Points,keyPos=keyPos,PPs=PPs)

        elif usr == "2":
            dispMatplot(Arm=Arm, PSangles=Angles, Points=Points,keyPos=keyPos,PPs=PPs)


# PyQTgraph main function & sub-functions
jointR = 0.3


def dispPyqt(Arm, PSangles, Points, keyPos, margin=2,PPs=30):
    axisStroke = 3
    keyPos = np.array(keyPos)[:, 0:3]
    pathPoints = np.array(Points)[:, :3]
    armPositions = np.array([getAllPos(Arm, angle) for angle in PSangles])
    endPos = armPositions[:, -1, :]

    app = pg.mkQApp("Robot Arm")
    view = gl.GLViewWidget()
    view.show()

    jointPoints = np.array(getRotAxisPoints(Arm=Arm,valueSet=PSangles,armPositions=armPositions))

    jointP1set, jointP2set = jointPoints[:, :, 0, :], jointPoints[:, :, 1, :]
    linkP1set, linkP2set = armPositions[:, :-1, :], armPositions[:, 1:, :]

    # path
    pathLine = gl.GLLinePlotItem(pos=pathPoints,color=(1.0, 0.65, 0.0, 1.0),width=1)
    armLine = gl.GLLinePlotItem(pos=armPositions[0],color=(0.5, 0.8, 1.0, 1.0),width=1)
    view.addItem(pathLine)
    view.addItem(armLine)

    # arm links
    linkCylinders = makeMeshHelperSet(P1set=linkP1set[0], P2set=linkP2set[0], view=view, type="link")
    
    # joints
    jointCylinders = makeMeshHelperSet(P1set=jointP1set[0], P2set=jointP2set[0], view=view, type="joint") 
    markers = jointLines(Arm=Arm,valueSet=PSangles,jointPoints=jointPoints,armPositions=armPositions)
    markerLine = gl.GLLinePlotItem(pos=markers[0], color=(1, 1, 0, 1), width=2, mode='lines')
    view.addItem(markerLine)

    # endeffector
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
        markerLine.setData(pos=markers[f])
        
        xOriginLine.setData(pos=origins[f][0])
        yOriginLine.setData(pos=origins[f][1])
        zOriginLine.setData(pos=origins[f][2])

        armLine.setData(pos=armPositions[f])

        sphereMesh.resetTransform()
        sphereMesh.translate(*endPos[f])

        for i in range(len(linkCylinders)):
            P1, P2 = linkP1set[f][i], linkP2set[f][i]
            applyTransform(linkCylinders[i], P1, P2)
        
        for i in range(len(jointCylinders)):
            P1, P2 = jointP1set[f][i], jointP2set[f][i]
            applyTransform(jointCylinders[i][0], P1, P2)
            applyTransform(jointCylinders[i][1], P1, P2)
            applyTransform(jointCylinders[i][2], P2, P1)

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
 
def getCylinderMesh(P1, P2, type):
    vec = P2 - P1
    normLength = np.linalg.norm(vec)

    linkageR = 0.2

    if type == "link":
        color=(0.5, 0.8, 1.0, 1.0)
        radiusC = linkageR
        Data = gl.MeshData.cylinder(rows=1, cols=20, radius=[radiusC, radiusC], length=normLength)
    
    elif type == "joint":
        color=(0.6, 0.98, 0.6, 1.0)
        radiusC = jointR
        Data = gl.MeshData.cylinder(rows=1, cols=20, radius=[radiusC, radiusC], length=normLength)
    
    elif type == "circle":
        color=(0.6, 0.98, 0.6, 1.0)
        radiusC = jointR
        Data = gl.MeshData.cylinder(rows=1, cols=20, radius=[radiusC, 0], length=0.01)
    
    
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

def makeMeshHelperSet(P1set, P2set, view, type="link"):
    helper = []

    numCylinder = len(P1set)
    if type == "joint":
        numCylinder -= 1

    for i in range(numCylinder):
            P1 = P1set[i]
            P2 = P2set[i]
            linkMesh = getCylinderMesh(P1, P2, type)
            
            if linkMesh:
                view.addItem(linkMesh)
                if type == "joint":
                    circle1 = getCylinderMesh(P1, P2, "circle")
                    circle2 = getCylinderMesh(P2, P1, "circle")

                    view.addItem(circle1)
                    view.addItem(circle2)

                    helper.append([linkMesh, circle1, circle2])

                else:
                    helper.append(linkMesh)
                    
    return np.array(helper,dtype=object)

def getRotAxisPoints(Arm,valueSet,armPositions,cLength=0.5):
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

def applyTransform(meshItem, P1, P2):
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

def jointLines(Arm,valueSet,jointPoints=None, armPositions=None):
    if armPositions is None:
        armPositions = np.array([getAllPos(Arm, angle) for angle in valueSet])
    
    if jointPoints is None:
        jointPoints = np.array(getRotAxisPoints(Arm=Arm,valueSet=valueSet,armPositions=armPositions))

    armDirections = np.array(getArmDirections(armPositions)) * jointR
    rotationMatrices = getRotationMatrices(arm=Arm,valueSet=valueSet)

    lines = []
    for frame in range(len(valueSet)):
        frameLines = []
        numJoints = rotationMatrices.shape[1]
        
        for i in range(numJoints):
            P1 = jointPoints[frame][i][0]
            P2 = jointPoints[frame][i][1]

            vecIn = armDirections[frame][i]
            vecOut = rotationMatrices[frame][i][:3, 0] * jointR

            frameLines.extend([P1, P1 + vecIn, P2, P2 + vecIn])
            frameLines.extend([P1, P1 + vecOut, P2, P2 + vecOut])
        lines.append(np.array(frameLines))
    return lines

def getArmDirections(armPositions):
    uVec = []

    for frame in armPositions:
        numJoints = len(frame) - 1
        uFrame = []
        uFrame.append([1,0,0])
        for j in range(numJoints):
            vec = frame[j + 1] - frame[j]
            uvec = vec / np.linalg.norm(vec)
            uFrame.append(uvec)
        
        uVec.append(uFrame)
    
    return uVec

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