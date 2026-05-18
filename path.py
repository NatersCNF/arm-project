import numpy as np
import math
from arm import *
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation


class point:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.pos = [x, y, z]
        self.rot = [roll, pitch, yaw]
        self.full = self.pos + self.rot

class path:
    def __init__(self):
        self.keyPos = []

    def addP(self, point):
        self.keyPos.append(point.full)
    
    def removeP(self, index):
        self.keyPos.pop(index - 1)

    def getKey(self):
        return self.keyPos
    
    def getStart(self):
        return self.keyPos[0]
    
    def getEnd(self):
        return self.keyPos[-1]
    
    def setStart(self, P = None):
        if P is None:
            straightAngles = [math.pi] * len(self.Arm)

            startPoints = getAllPos(self.Arm, straightAngles)
            P = point(startPoints[-1][0], startPoints[-1][1], startPoints[-1][2], 0, 0, 0)
        
        self.addP(P)

class pointSet:
    def __init__(self, path, Arm, type="p2p", PPs=30, v=1,a=0.5,angular_v=1,angular_a=0.5,live=False):
        self.nP = 0

        self.speed = abs(v)
        self.acceleration = abs(a)
        self.angular_speed = abs(angular_v)
        self.angular_acceleration = abs(angular_a)
        
        if not(type == "p2p" or type == "smooth" or type == "p2p_trap" or type == "smooth_trap"):
            raise ValueError("invalid type")
        self.type = type
        
        if all(isinstance(item, link) for item in Arm):
            self.Arm = Arm
        else:
            raise ValueError("arm must be an array of links from the links class")
        
        if PPs > 1:
            self.PPs = PPs

        self.uvec = []

        self.points = []

        if len(path.keyPos) > 1 or live == True:
            self.path = path
        else:
            raise ValueError("not enough points in the path")
        
        self.updateCheck()

    def getPathData(self):
        angles = self.getPathAngles()
        return self.Arm, angles, self.points, self.path.keyPos

    def applyTransform(self, meshItem, P1, P2):
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

    def updatenP(self):
        self.nP = len(self.path.keyPos)

    def generatePoints(self):
        if self.type == "smooth":
            self.generateSpline()

        elif self.type == "p2p":
            self.generateP2P()
        
        elif self.type == "p2p_trap":
            self.generateTrapezoidP2P()
        
        elif self.type == "smooth_trap":
            self.generateTrapezoidSpline()
        
        print("Number of points: " + str(len(self.points)))

    def updateUVec(self):
        self.uvec.clear()
        for i in range(self.nP - 1):
            v = []
            length = 0
            for j in range (3):
                v.append(self.path.keyPos[i + 1][j] - self.path.keyPos[i][j])
                length += v[j] ** 2

            length = math.sqrt(length)
            v.append(length)

            if length > 0:
                for j in range (3):
                    v[j] = v[j] / length
            self.uvec.append(v)

    def updateCheck(self):
        #print("nP value:")
        self.updatenP()
        #print(str(self.nP))

        #print("POINTS:")
        #print(str(self.path.keyPos))

        #print("UVECTORS:")
        self.updateUVec()
        #print(str(self.uvec))

    def generateP2P(self): #speed = u/s, and specified earlier, PPs = points per second 
        self.points.clear()

        self.points.append(self.path.keyPos[0])

        for i in range(self.nP-1):
            initial = self.path.keyPos[i]
            final = self.path.keyPos[i + 1]
            currentLength = self.uvec[i][3]
            direction = self.uvec[i][:3]            

            pos_diff = np.linalg.norm(np.array(final[:3]) - np.array(initial[:3]))
            rot_diff = np.linalg.norm(np.array(final[3:]) - np.array(initial[3:]))

            metric = max(pos_diff, rot_diff)

            n = int(metric * (self.PPs / self.speed))
            n = max(1, n)

            interval = currentLength / n

            for j in range(1, n+1):
                step = j / n
                x = initial[0] + (direction[0] * j * interval)
                y = initial[1] + (direction[1] * j * interval)
                z = initial[2] + (direction[2] * j * interval)

                roll = initial[3] + ((final[3] - initial[3]) * step)
                pitch = initial[4] + ((final[4] - initial[4]) * step)
                yaw = initial[5] + ((final[5] - initial[5]) * step)

                x = round(x, 10)
                y = round(y, 10)
                z = round(z, 10)

                roll = round(roll, 10)
                pitch = round(pitch, 10)
                yaw = round(yaw, 10)

                self.points.append([x, y, z, roll, pitch, yaw])

    def generateTrapezoidP2P(self): #speed = u/s, and specified earlier, PPs = points per second 
        self.updateCheck()
        self.points.clear()
        path_points = []

        key_pos = self.path.keyPos
        path_points.append(key_pos[0])

        for i in range(len(key_pos) - 1):
            cur_key_pos = np.array(key_pos[i])
            next_key_pos = np.array(key_pos[i + 1])

            dt = 1 / self.PPs

            pos_diff_vec = next_key_pos[:3] - cur_key_pos[:3]
            rot_diff_vec = next_key_pos[3:] - cur_key_pos[3:]
            

            pos_diff = np.linalg.norm(pos_diff_vec)
            rot_diff = np.linalg.norm(rot_diff_vec)

            if rot_diff == 0 and pos_diff == 0:
                continue

            segment_time = self.get_longer(pos_diff, rot_diff)
            num_points = max(1, int(np.ceil(segment_time * self.PPs)))

            actual_time = num_points / self.PPs

            for j in range(num_points):
                current_time = j * dt

                current_pos = cur_key_pos[:3]

                if pos_diff != 0:
                    current_pos_fraction = self.get_value_fraction(current_time=current_time, total_time=actual_time,distance=pos_diff)
                    current_pos = (pos_diff_vec * current_pos_fraction) + cur_key_pos[:3]
                
                else:
                    current_pos = cur_key_pos[:3]
                
                if rot_diff != 0:
                    current_rot_fraction = self.get_value_fraction(current_time=current_time, total_time=actual_time,angle=rot_diff)
                    current_rot = (rot_diff_vec * current_rot_fraction) + cur_key_pos[3:]
                
                else:
                    current_rot = cur_key_pos[3:]
                
                num_decimals = 10

                rounded_pos = np.round(current_pos,num_decimals).tolist()
                rounded_rot = np.round(current_rot,num_decimals).tolist()

                x, y, z = rounded_pos
                roll, pitch, yaw = rounded_rot

                self.points.append([x, y, z, roll, pitch, yaw])

    def get_time(self, angle=None,distance=None):
        if angle is None:
            speed = self.speed
            acceleration = self.acceleration
            value = abs(distance)

        elif distance is None:
            speed = self.angular_speed
            acceleration = self.angular_acceleration
            value = abs(angle)
        
        max_acceleration_time = speed / acceleration
        max_acceleration_value = (acceleration / 2) * (max_acceleration_time ** 2)
        half_value = value / 2
        value_acceleration_time = math.sqrt((2 * half_value) / acceleration)

        if max_acceleration_time > value_acceleration_time:
            return value_acceleration_time * 2
        
        else:
            remaining_value = value - (max_acceleration_value * 2)
            const_speed_time = remaining_value / speed
            return const_speed_time + (max_acceleration_time * 2)

    def get_value_fraction(self, current_time, total_time, angle=None,distance=None):
        if angle is None:
            speed = self.speed
            acceleration = self.acceleration
            value = abs(distance)
            true_time = self.get_time(distance=distance)

        elif distance is None:
            speed = self.angular_speed
            acceleration = self.angular_acceleration
            value = abs(angle)
            true_time = self.get_time(angle=angle)
        
        if true_time < total_time and true_time > 0:
            scale_factor = true_time / total_time
            speed *= scale_factor
            acceleration *= (scale_factor ** 2)

        current_time = max(0, min(current_time, total_time))
        
        max_acceleration_time = speed / acceleration
        acceleration_distance = (acceleration / 2) * (max_acceleration_time ** 2)
        flat_time = total_time - (max_acceleration_time * 2)

        if total_time < (2 * max_acceleration_time):
            half_time = total_time / 2
            half_value = (acceleration / 2) * (half_time ** 2)

            if current_time < half_time:
                new_value = (acceleration / 2) * (current_time ** 2)

            else:
                speed_0 = half_time * acceleration
                time_over_half = current_time - half_time
                value_over_half = (speed_0 * time_over_half) - ((acceleration / 2) * (time_over_half ** 2))
                new_value = half_value + value_over_half
        
        else:
            if current_time < max_acceleration_time:
                new_value = (acceleration / 2) * (current_time ** 2)
            
            elif current_time < (total_time - max_acceleration_time):
                const_speed_time = current_time - max_acceleration_time
                new_value = acceleration_distance + (const_speed_time * speed)
            
            else:
                flat_distance = flat_time * speed
                time_left = current_time - flat_time - max_acceleration_time
                remaining_value = (speed * time_left) - ((acceleration / 2) * (time_left ** 2))
                new_value = acceleration_distance + flat_distance + remaining_value





        fraction = new_value / value
        return fraction

    def get_longer(self, pos_diff, rot_diff):
        pos_time = self.get_time(distance=pos_diff)
        rot_time = self.get_time(angle=rot_diff)
        segment_time = max(pos_time, rot_time)
        return segment_time

    def generateSpline(self):
        self.points.clear()
        m = []                  # assuming start & stop at first and last point
        m.append([0, 0, 0])

        for i in range(1, (self.nP - 1)):
            PA = self.path.keyPos[i + 1]
            PB = self.path.keyPos[i - 1]
            

            mx = (PA[0] - PB[0]) / 2
            my = (PA[1] - PB[1]) / 2
            mz = (PA[2] - PB[2]) / 2
            
            slope = [mx, my, mz]
            m.append(slope)
        m.append([0, 0, 0])
        
        for i in range(0, self.nP - 1):
            initial = self.path.keyPos[i]
            final = self.path.keyPos[i + 1]

            posx = final[0] - initial[0]
            posy = final[1] - initial[1]
            posz = final[2] - initial[2]

            currentLength = math.sqrt((posx ** 2) + (posy ** 2) + (posz ** 2))

            PPL = int(currentLength * (self.PPs / self.speed))
            PPL = max(1, PPL)

            Pi = self.path.keyPos[i][:3]
            Pi1 = self.path.keyPos[i + 1][:3]
            mi = m[i]
            mi1 = m[i + 1]

            

            for j in range(1, PPL+1):
                t = (1 / PPL) * j

                roll = initial[3] + ((final[3] - initial[3]) * t)
                pitch = initial[4] + ((final[4] - initial[4]) * t)
                yaw = initial[5] + ((final[5] - initial[5]) * t)

                pos = getPatT(t, Pi, Pi1, mi, mi1)
                pos = pos + [roll, pitch, yaw]

                self.points.append(pos)

    def generateTrapezoidSpline(self):
        self.points.clear()
        m = []                  # assuming start & stop at first and last point
        m.append([0, 0, 0])
        dt = 1 / self.PPs

        key_pos = self.path.keyPos
        for i in range(1, (self.nP - 1)):
            PA = key_pos[i + 1]
            PB = key_pos[i - 1]
            
            mx = (PA[0] - PB[0]) / 2
            my = (PA[1] - PB[1]) / 2
            mz = (PA[2] - PB[2]) / 2
            
            slope = [mx, my, mz]
            m.append(slope)
        m.append([0, 0, 0])
        
        for i in range(0, self.nP - 1):
            cur_key_pos = np.array(key_pos[i])
            next_key_pos = np.array(key_pos[i + 1])

            pos_diff_vec = next_key_pos[:3] - cur_key_pos[:3]
            rot_diff_vec = next_key_pos[3:] - cur_key_pos[3:]
            
            pos_diff = np.linalg.norm(pos_diff_vec)
            rot_diff = np.linalg.norm(rot_diff_vec)

            segment_time = self.get_longer(pos_diff, rot_diff)
            num_points = max(1, int(segment_time * self.PPs))
            actual_time = num_points / self.PPs

            Pi = key_pos[i][:3]
            Pi1 = key_pos[i + 1][:3]
            mi = m[i]
            mi1 = m[i + 1]

            for j in range(1, num_points+1):
                current_time = j * dt

                if pos_diff >= rot_diff and pos_diff > 0:
                    t = self.get_value_fraction(current_time, actual_time, distance=pos_diff)

                elif rot_diff > 0:
                    t = self.get_value_fraction(current_time, actual_time, angle=rot_diff)

                else:
                    t = 1

                pos = getPatT(t, Pi, Pi1, mi, mi1)

                current_rot = cur_key_pos[3:] + (rot_diff_vec * t)

                self.points.append(pos + current_rot.tolist())

    def getPathAngles(self):
        angles = get_pointset_angles(self.Arm,point_set=self.points)
        return angles

    def printPoints(self, offset=0):
        for i, row in enumerate(self.points):
            x, y, z, roll, pitch, yaw = row

            print(f"P_{{{i+offset}}} = ({x}, {y}, {z})")

            dx = math.cos(pitch) * math.cos(yaw)
            dy = math.cos(pitch) * math.sin(yaw)
            dz = math.sin(pitch)

            dx = round(dx, 6) + x
            dy = round(dy, 6) + y
            dz = round(dz, 6) + z

            print(f"\\operatorname{{vector}}(P_{{{i+offset}}}, ({dx}, {dy}, {dz}))\n")

    def printAllPoints(self):
        print("p2p points:")
        print("")
        self.generateP2P()
        self.printPoints()
        n = int(len(self.points))
        
        print("spline points:")
        print("")
        self.generateSpline()
        self.printPoints(n)

def getH(h, t):
    if h == "00":
        h00 = (2 * t ** 3) - (3 * t ** 2) + 1
        return h00
    elif h == "10":
        h10 = (t ** 3) - (2 * t ** 2) + t
        return h10
    elif h == "01":
        h01 = (-2 * t ** 3) + (3 * t ** 2)
        return h01
    elif h == "11":
        h11 = (t ** 3) - (t ** 2)
        return h11
    else:
        raise ValueError("not a valid h function")

def getPatT(t, Pi, Pi1, mi, mi1):
    Pi = np.array(Pi)
    Pi1 = np.array(Pi1)
    mi = np.array(mi)
    mi1 = np.array(mi1)

    P00 = getH("00",t) * Pi
    
    P10 = getH("10",t) * mi
    P01 = getH("01",t) * Pi1
    P11 = getH("11",t) * mi1
    P = P00 + P10 + P01 + P11
    
    return P.tolist()

def getRotAxisPoints(Arm,valueSet,armPositions,cLength=1):
    halfLen = cLength / 2
    allRotAxes = getAllRotPos(Arm,valueSet)
    jointPointSet = []

    for frameAxes, framePos in zip(allRotAxes, armPositions):
        frame = []

        for i in range(len(frameAxes)):
            pos = np.array(framePos[i + 1])
            rotAxis = np.array(frameAxes[i])

            modified = rotAxis * halfLen

            start = pos + modified
            end = pos - modified

            frame.append([start, end])
        
        jointPointSet.append(frame)
    
    return jointPointSet

    helper = []
    for i in range(len(P1set)):
            P1 = P1set[i]
            P2 = P2set[i]
            linkMesh = getCylinderMesh(P1, P2, type)
            if linkMesh:
                view.addItem(linkMesh)
                helper.append(linkMesh)
    return np.array(helper)
