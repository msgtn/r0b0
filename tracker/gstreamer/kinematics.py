'''
Kinematics for Blossom
'''

import numpy as np
from numpy import cos
from numpy import sin
from numpy import pi
from scipy.spatial.transform import Rotation
import time

'''
physical params
'''
# get radius from center of tower to attachment point
l = 42.5
r = l/(2*cos(pi/6))
# define resting height
h_rest = 82.5       # from loop 
# h_rest = 106.5    # from wheel edge
# angle between tower and attachment point
alp = np.arctan2(r,h_rest)
# radius of wheel
# undersize to exaggerate more, oversize to reduce sensitivity
r_w = 9.0
r_w = 5.0
r_w = 9.0

# VideoPose3D - undersize to exaggerate
# r_w = 3.0
# r_w = 12.0


# how much to amplify base motion
base_mult = 1.
# position of front attachment point wrt inertial frame
p1_0 = np.array([r,0,0])
# define rotation matrices to other attachment points
R_12 = np.array([[cos(2*pi/3), -sin(2*pi/3), 0],\
    [sin(2*pi/3), cos(2*pi/3), 0],\
    [0,0,1]])
R_13 = np.transpose(R_12)
# calculate positions of attachment points
p2_0 = np.matmul(R_12,p1_0)
p3_0 = np.matmul(R_13,p1_0)
# concatenate points into one matrix
p = np.array([p1_0,p2_0,p3_0])
p_list = [p1_0, p2_0, p3_0]
# store just x,z components
p_xz = np.array([p1_0[[0,2]],p2_0[[0,2]],p3_0[[0,2]]])
x = [0]*3
v = x
t = time.time()

# map height to robot
# don't let it go all the way down so that
# it can still tilt at the top and bottom
h_max = 140.0
h_min = -40.0
h_min = -0.0
h_range = h_max-h_min
h_mid = h_min + h_range/2.0
# factor to limit motion defined by height slider
h_fac = 0.85
# rescale height limits
h_max = h_mid + h_range/h_fac
h_min = h_mid - h_range/h_fac


# get direction cosine matrix (R) from given euler angles
# gives O_R_B (rotation from body frame to the inertial frame)
def angle2dcm(t1,t2,t3):
    """
    Get direction cosine matrix (O_R_B) from Euler angles
    args:
        t1  yaw
        t2  pitch
        t3  roll
    returns:
        direction cosine matrix
    """
#     return np.transpose(np.matrix([[ cos(t1)*cos(t2), cos(t1)*sin(t2)*sin(t3) - cos(t3)*sin(t1), sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)],\
# [ cos(t2)*sin(t1), cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3), cos(t3)*sin(t1)*sin(t2) - cos(t1)*sin(t3)],\
# [        -sin(t2),                           cos(t2)*sin(t3),                           cos(t2)*cos(t3)]]))

    # trying to implement the matrix from w3c
    # https://www.w3.org/TR/orientation-event/#deviceorientation
    t2 = -t2+np.pi/2
    t3 = -t3
    cX = np.cos(t3)
    cY = np.cos(t2)
    cZ = np.cos(t1)
    sX = np.sin(t3)
    sY = np.sin(t2)
    sZ = np.sin(t1)

    m11 = cZ * cY - sZ * sX * sY;
    m12 = - cX * sZ;
    m13 = cY * sZ * sX + cZ * sY;

    m21 = cY * sZ + cZ * sX * sY;
    m22 = cZ * cX;
    m23 = sZ * sY - cZ * cY * sX;

    m31 = - cX * sY;
    m32 = sX;
    m33 = cX * cY;

    R = np.matrix([\
        [m11,    m12,    m13],\
        [m21,    m22,    m23],\
        [m31,    m32,    m33]\
      ])

    x = np.matrix([
        [1, 0, 0],
        [0, cos(np.pi/2), -sin(np.pi/2)],
        [0, sin(np.pi/2), cos(np.pi/2)]
        ])
    x = np.matrix([
        [1, 0, 0],
        [0, cos(np.pi/2), -sin(np.pi/2)],
        [0, sin(np.pi/2), cos(np.pi/2)]
        ])
    # R = x*R


    cX = np.cos(t3/2)
    cY = np.cos(t2/2)
    cZ = np.cos(t1/2)
    sX = np.sin(t3/2)
    sY = np.sin(t2/2)
    sZ = np.sin(t1/2)
    w = cX * cY * cZ - sX * sY * sZ;
    x = sX * cY * cZ - cX * sY * sZ;
    y = cX * sY * cZ + sX * cY * sZ;
    z = cX * cY * sZ + sX * sY * cZ;
    # print([w,x,y,z])

    # r = Rotation.from_quat([x,y,z,w])
    r = Rotation.from_rotvec([t1,t2,t3])
    try:
        R = np.matrix(r.as_matrix())
    except:
        R = np.matrix(r.as_dcm())

    # print(R)

    return R

def get_quat(x, y, z):
  cX = np.cos( x/2 );
  cY = np.cos( y/2 );
  cZ = np.cos( z/2 );
  sX = np.sin( x/2 );
  sY = np.sin( y/2 );
  sZ = np.sin( z/2 );


    # case 'YXZ':
    #     break;


# w = c1 * c2 * c3 + s1 * s2 * s3;
  w = cX * cY * cZ + sX * sY * sZ;
# x = s1 * c2 * c3 + c1 * s2 * s3;
  x = sX * cY * cZ + cX * sY * sZ;
# y = c1 * s2 * c3 - s1 * c2 * s3;
  y = cX * sY * cZ - sX * cY * sZ;
# z = c1 * c2 * s3 - s1 * s2 * c3;
  z = cX * cY * sZ - sX * sY * cZ;
  # print(w, x, y, z)
  return [w, x, y, z]



def get_motor_pos(ori, portrait=False, sensitivity=1.0):
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
    [e_1, e_2, e_3, e_4, yaw_offset] = ori
    [beta, gamma, alpha,_, yaw_offset] = ori

    angle_order = 'ZXY'
    angles = [alpha, beta, gamma]

    r = Rotation.from_euler(
        angle_order,
        angles=angles)
    # print(alpha, r.as_euler(angle_order)[0], r.as_euler(angle_order.lower())[0])
    r = r*Rotation.from_euler('Z',np.pi/2)
    # print(alpha-gamma)
    # alpha -= gamma
    if portrait:
        r = r*Rotation.from_euler('Y',np.pi/2)
    # print(r.as_rotvec()[-1])

    p_0 = [r.apply(_p) for _p in [p1_0, p2_0, p3_0]]

    # calculate height
    h = ((e_4-50)/100.0)*h_range*h_fac+h_mid

    # init array for motor positions (tower_1-4)
    motor_pos = np.array([])
    # calculate distance motor must rotate
    for i, p_i in enumerate(p_0):
        # get just x,z components
        p_i_xz = p_i[[0,2]]
        # p_i_xz = p_i
        # get displacement and its magnitude
        # del_h = p_i_xz-yaw.apply(p_list[i])
        del_h = p_i_xz-p_xz[i]
        # del_h = del_h[[0,2]]
        mag_del_h = np.linalg.norm(del_h)

        # calculate motor angle
        theta = np.rad2deg(mag_del_h/(r_w/sensitivity))*np.sign(del_h[1])

        # EXPERIMENTAL - only take the z-difference
        mag_del_h = del_h[1]
        theta = np.rad2deg(mag_del_h/(r_w/sensitivity))

        # print(theta)
        motor_pos = np.append(motor_pos,theta)

    # constrain tower motor range (50-130)
    motor_pos = np.maximum(np.minimum(motor_pos+h,h_max),h_min)
    try:
        alpha -= yaw_offset
    except:
        pass

    if (alpha < -np.pi):
        alpha+=2*np.pi
    elif(alpha > np.pi):
        alpha-=2*np.pi

    # add the base motor for yaw (-140,140)
    motor_pos = np.append(motor_pos,np.maximum(np.minimum(np.rad2deg(base_mult*alpha),150),-150))

    return motor_pos

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
    e_pos = (e-50)*(ears_range/50.0)+ears_offset
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
    del_z = np.cos(alp)*del_h
    return del_z

def integrate_accel(ori,accel):
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
    dcm = angle2dcm(e_1,e_2,e_3)
    # rotate to inertial frame
    a = np.array(np.dot(dcm,a_B))[0]
    a_thresh = [0.1]*3
    a = (np.greater(np.abs(a),a_thresh)*a)
    a = [0,0,a[2]]
    # a = dcm.dot(a_B)

    # get time difference
    del_t = time.time()-t
    # del_t = del_t/2
    # integrate velocity
    v = integrate(v,a,del_t)
    v_thresh = [0.01]*3
    v = (np.greater(np.abs(v),v_thresh)*v)
    # integreate position
    x = integrate(x,v,del_t)

    # update time
    t = time.time()

    # return params
    return (x,v,time.time())

def truncate(n):
    """
    Truncate number to 2 decimal places
    args:
        n   the number to truncate
    """
    a = [np.floor(i*100)/100.0 for i in n]
    return a

def integrate(x,x_d,del_t):
    """
    Discrete integration
    args:
        x       present value
        x_d     present change
        del_t   time step
    returns:
        the new present value of x
    """
    return (x+np.multiply(x_d,del_t))
