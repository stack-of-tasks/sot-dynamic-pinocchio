from dynamic import Dynamic as DynamicOld
from angle_estimator import AngleEstimator
from zmp_from_forces import ZmpFromForces
import numpy as np
from numpy import arctan2, arcsin, sin, cos, sqrt

#DynamicOld = Dynamic

class Dynamic (DynamicOld):
    def __init__(self, name):
        DynamicOld.__init__(self, name)
        self.model = None
        self.data = None

    def setData(self, pinocchio_data):
        dynamic.wrap.set_pinocchio_data(self.obj,pinocchio_data)
        self.data = pinocchio_data
        return
        
    def setModel(self, pinocchio_model):
        dynamic.wrap.set_pinocchio_model(self.obj,pinocchio_model)
        self.model = pinocchio_model
        return

def fromSotToPinocchio(q_sot, freeflyer=True):
    if freeflyer:
        [r,p,y] = q_sot[3:6]
        cr = cos(r)
        cp = cos(p)
        cy = cos(y)
        sr = sin(r)
        sp = sin(p)
        sy = sin(y)

        rotmat = np.matrix([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                           [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                           [-sp, cp*sr, cp*cr]])

        d0 = rotmat[0,0]
        d1 = rotmat[1,1]
        d2 = rotmat[2,2]
        rr = 1.0+d0+d1+d2

        if rr>0:
            s = 0.5 / sqrt(rr)
            _x = (rotmat[2,1] - rotmat[1,2]) * s
            _y = (rotmat[0,2] - rotmat[2,0]) * s
            _z = (rotmat[1,0] - rotmat[0,1]) * s
            _r = 0.25 / s
        else:
            #Trace is less than zero, so need to determine which
            #major diagonal is largest
            if ((d0 > d1) and (d0 > d2)):
                s = 0.5 / sqrt(1 + d0 - d1 - d2)
                _x = 0.5 * s
                _y = (rotmat[0,1] + rotmat[1,0]) * s
                _z = (rotmat[0,2] + rotmat[2,0]) * s
                _r = (rotmat[1,2] + rotmat[2,1]) * s
            elif (d1 > d2):
                s = 0.5 / sqrt(1 + d0 - d1 - d2)
                _x = (rotmat[0,1] + rotmat[1,0]) * s
                _y = 0.5 * s
                _z = (rotmat[1,2] + rotmat[2,1]) * s
                _r = (rotmat[0,2] + rotmat[2,0]) * s
            else:
                s = 0.5 / sqrt(1 + d0 - d1 - d2)
                _x = (rotmat[0,2] + rotmat[2,0]) * s
                _y = (rotmat[1,2] + rotmat[2,1]) * s
                _z = 0.5 * s
                _r = (rotmat[0,1] + rotmat[1,0]) * s

        return np.matrix([q_sot[0:3]+(_x,_y,_z,_r)+q_sot[6:]])
    else:
        return np.matrix([q_sot[0:]])
