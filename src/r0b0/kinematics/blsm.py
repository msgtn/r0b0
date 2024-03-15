"""
Kinematics for Blossom
"""

import numpy as np
from numpy import cos
from numpy import sin
from numpy import pi
from scipy.spatial.transform import Rotation
import time
from functools import partial

"""
physical params
"""
# get radius from center of tower to attachment point
l = 42.5
r = l / (2 * cos(pi / 6))
# define resting height
h_rest = 82.5  # from loop
# h_rest = 106.5    # from wheel edge
# angle between tower and attachment point
alp = np.arctan2(r, h_rest)
# radius of wheel
# undersize to exaggerate more, oversize to reduce sensitivity
r_w = 9.0

# how much to amplify base motion
base_mult = 1.0
# position of front attachment point wrt inertial frame
p1_0 = np.array([r, 0, 0])
# define rotation matrices to other attachment points
R_12 = np.array(
    [
        [cos(2 * pi / 3), -sin(2 * pi / 3), 0],
        [sin(2 * pi / 3), cos(2 * pi / 3), 0],
        [0, 0, 1],
    ]
)
R_13 = np.transpose(R_12)
# calculate positions of attachment points
p2_0 = np.matmul(R_12, p1_0)
p3_0 = np.matmul(R_13, p1_0)
# concatenate points into one matrix
p = np.array([p1_0, p2_0, p3_0])
p_list = [p1_0, p2_0, p3_0]
# store just x,z components
p_xz = np.array([p1_0[[0, 2]], p2_0[[0, 2]], p3_0[[0, 2]]])
x = [0] * 3
v = x
t = time.time()

# map height to robot
# don't let it go all the way down so that
# it can still tilt at the top and bottom
h_max = 140.0
h_min = -40.0
h_min = -0.0
h_range = h_max - h_min
h_mid = h_min + h_range / 2.0
# factor to limit motion defined by height slider
h_fac = 0.85
# rescale height limits
h_max = h_mid + h_range / h_fac
h_min = h_mid - h_range / h_fac


def get_ears_pos(e):
    """
    Get position of the ears from the app slider
    args:
        e   input from app slider
    returns:
        the angular position of the ear
    """

    # define range of ears
    ears_range = 50
    # define center of ears
    ears_offset = 100

    # interpolate from slider range (0-100)
    e_pos = (e - 50) * (ears_range / 50.0) + ears_offset
    # print(e_pos)
    return e_pos


def fwd_kin(m):
    """
    Forward kinematics (motor_pos -> robot pose)
    args:
        m   motor positions in degrees (list)
    """
    # displacement
    del_h = r_w * np.deg2rad(m)
    del_z = np.cos(alp) * del_h
    return del_z


def integrate_accel(ori, accel):
    """
    Get height by integrating accelerometer
    args:
        ori     orientation readings
        accel   accelerometer readings
    returns:
        vertical position and velocity and time
    """
    [e_1, e_2, e_3] = ori
    global x, v, t
    # save accel values in body frame
    a_B = np.array(accel)

    # get dcm
    dcm = angle2dcm(e_1, e_2, e_3)
    # rotate to inertial frame
    a = np.array(np.dot(dcm, a_B))[0]
    a_thresh = [0.1] * 3
    a = np.greater(np.abs(a), a_thresh) * a
    a = [0, 0, a[2]]
    # a = dcm.dot(a_B)

    # get time difference
    del_t = time.time() - t
    # del_t = del_t/2
    # integrate velocity
    v = integrate(v, a, del_t)
    v_thresh = [0.01] * 3
    v = np.greater(np.abs(v), v_thresh) * v
    # integreate position
    x = integrate(x, v, del_t)

    # update time
    t = time.time()

    # return params
    return (x, v, time.time())


def truncate(n):
    """
    Truncate number to 2 decimal places
    args:
        n   the number to truncate
    """
    a = [np.floor(i * 100) / 100.0 for i in n]
    return a


def integrate(x, x_d, del_t):
    """
    Discrete integration
    args:
        x       present value
        x_d     present change
        del_t   time step
    returns:
        the new present value of x
    """
    return x + np.multiply(x_d, del_t)


# class BlossomKinematics(Kinematics):
#     def __init__(self, **kwargs):
#         super.__init__(**kwargs)
#         pass


def device_motion2motor(ori, portrait=True, sensitivity=1.0):
    """
    Get position of the motors given orientation using inverse kinematics
    args:
        ori     orientation from sensors (Euler angles)
        accel   accelerometer readings
    returns:
        motor positions
    """
    # yaw, pitch, roll
    # zyx / alpha,gamma,beta
    # print(ori)
    # [e_1, e_2, e_3, e_4, yaw_offset] = ori
    # [beta, gamma, alpha,_, yaw_offset] = ori
    # breakpoint()
    beta = ori["x"]
    gamma = ori["y"]
    alpha = ori["z"]
    e_4 = ori["h"]
    yaw_offset = ori["yaw"]

    angle_order = "ZXY"
    angles = [alpha, beta, gamma]

    r = Rotation.from_euler(angle_order, angles=angles)
    r = r * Rotation.from_euler("Z", np.pi / 2)
    if portrait:
        r = r * Rotation.from_euler("Y", np.pi / 2)

    p_0 = [r.apply(_p) for _p in [p1_0, p2_0, p3_0]]

    # calculate height
    h = ((e_4 - 50) / 100.0) * h_range * h_fac + h_mid

    # init array for motor positions (tower_1-4)
    motor_pos = np.array([])
    # calculate distance motor must rotate
    for i, p_i in enumerate(p_0):
        # get just x,z components
        p_i_xz = p_i[[0, 2]]
        # p_i_xz = p_i
        # get displacement and its magnitude
        # del_h = p_i_xz-yaw.apply(p_list[i])
        del_h = p_i_xz - p_xz[i]
        # del_h = del_h[[0,2]]
        mag_del_h = np.linalg.norm(del_h)

        # calculate motor angle
        theta = np.rad2deg(mag_del_h / (r_w / sensitivity)) * np.sign(del_h[1])

        # EXPERIMENTAL - only take the z-difference
        mag_del_h = del_h[1]
        theta = np.rad2deg(mag_del_h / (r_w / sensitivity))

        # print(theta)
        motor_pos = np.append(motor_pos, theta)

    # constrain tower motor range (50-130)
    motor_pos = np.maximum(np.minimum(motor_pos + h, h_max), h_min)
    try:
        alpha -= yaw_offset
    except:
        pass

    if alpha < -np.pi:
        alpha += 2 * np.pi
    elif alpha > np.pi:
        alpha -= 2 * np.pi

    # add the base motor for yaw (-140,140)
    motor_pos = np.append(
        motor_pos, np.maximum(np.minimum(np.rad2deg(base_mult * alpha), 150), -150)
    )

    # 230620 - inverted since motors 2 and 3 swapped
    # in the refactor to r0b0
    motor_pos[3] *= -1

    if ori["mirror"]:
        motor_pos[1], motor_pos[2] = motor_pos[2], motor_pos[1]
        motor_pos[3] *= -1

    return motor_pos


RAD2DXL = [
    [[-10, 140], [0, 2048]],
    [[-10, 140], [0, 2048]],
    [[-10, 140], [0, 2048]],
    [[-140, 140], [0, 4096]],
]


def device_motion2dxl_motor(ori, portrait=True, sensitivity=1.0):
    motor_pos = device_motion2motor(ori, portrait, sensitivity)
    motor_dxl = []
    for _motor_pos, xp_fp in zip(motor_pos, RAD2DXL):
        motor_dxl.append(int(np.interp(_motor_pos, xp=xp_fp[0], fp=xp_fp[1])))
    # motor_pos[3] = int(np.interp(motor_pos[3],xp=[-160,160],fp=[0,4090]))
    return motor_dxl


RAD2DXL320 = [
    [[-10, 140], [0, 500]],
    [[-10, 140], [0, 500]],
    [[-10, 140], [0, 500]],
    [[-140, 140], [0, 1000]],
]


def device_motion2dxl_motor320(ori, portrait=True, sensitivity=1.0):
    motor_pos = device_motion2motor(ori, portrait, sensitivity)
    motor_dxl = []
    for _motor_pos, xp_fp in zip(motor_pos, RAD2DXL320):
        motor_dxl.append(int(np.interp(_motor_pos, xp=xp_fp[0], fp=xp_fp[1])))
    # motor_pos[3] = int(np.interp(motor_pos[3],xp=[-160,160],fp=[0,4090]))
    return motor_dxl


def device_motion2arduino_motor(ori, portrait=True, sensitivity=1.0):
    motor_pos = device_motion2motor(ori, portrait, sensitivity)
    motor_pos = list(
        map(int, map(partial(np.interp, xp=[-10, 140], fp=[10, 180]), motor_pos))
    )
    return motor_pos
