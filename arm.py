import sympy as sym
import numpy as np

class arm:
    def __init__(self, linkd, axis, setAngle, angleLim = False):
        
        self.linkd = linkd

        if axis in['x', 'y', 'z']:
            this.axis = axis
        else:
            raise ValueError("not a valid rotational axis, must be x y or z")
        
        if type(anglelim) == 'list' and len(angleLim) == 2:
            if anglelim[0] != anglelim[1]:
                if anglelim[0] > anglelim[1]:
                    this.anglelim[0] = anglelim[1]
                    this.anglelim[1] = anglelim[0]

                else:
                    this.anglelim = anglelim

            else:
                raise ValueError("invalid angle range")
        else:
            raise ValueError("Angle limit invalid, leave empty for no limit or enter 2 different angles")
        
        if this.angleLim[0] <= setAngle <= this.angleLim[1]:
                this.angle = angle
        else:
            raise ValueError("Start angle outside of limits")
        






        
