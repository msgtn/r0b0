import numpy as np
from numpy import cos, sin, pi
from scipy.spatial.transform import Rotation
import time


class Kinematics(object):
    def __init__(self):
        pass

    # get direction cosine matrix (R) from given euler angles
    # gives O_R_B (rotation from body frame to the inertial frame)
    # def angle2dcm(self, euler: 'np.ndarray[np.float]') -> dcm: 'np.ndarray[np.float]':
    def angle2dcm(self, euler):
        """
        Get direction cosine matrix (O_R_B) from Euler angles
        args:
            euler = [t1, t2, t3] (yaw, pitch, roll)
        returns:
            direction cosine matrix
        """
        r = Rotation.from_rotvec(euler)
        return np.matrix(r.as_dcm())
        try:
            return np.matrix(r.as_matrix())
        except:
            return np.matrix(r.as_dcm())
