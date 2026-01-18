import numpy as np
# get radius from center of tower to attachment point
l = 42.5
r = l / (2 * np.cos(np.pi / 6))
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
        [np.cos(2 * np.pi / 3), -np.sin(2 * np.pi / 3), 0],
        [np.sin(2 * np.pi / 3), np.cos(2 * np.pi / 3), 0],
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
