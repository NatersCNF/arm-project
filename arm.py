import sympy as sym
import numpy as np
import math

class link:
    def __init__(self, omega, d, a, alpha, Jlim, joint='r'):
        this.omega = omega
        this.d = d
        this.a = a
        this.alpha = alpha

        if Jlim[0] != Jlim[1] and len(Jlim) == 2:
            this.Jlim = JLim.sort()

        this.joint = joint

    def getDH_revolute(self):
        a

    def getDH_prismatic(self):
        a

    def getDH(self):
        return self.getDH_revolute() if self.joint == 'r' else self.getDH_prismatic