import numpy as np
import kinematics_library as kinlib

"""
This file contains Baxter robot's left arm kinematics
parameters with respect to base frame which can be used
to compute forward kinematics and Jacobian matrix.
Note: Parameters are obtained from URDF file of Baxter robot.
Author: Anirban Sinha.
Affiliation: State University of New York, Stony Brook
Date: Ocotber 20th., 2019.
"""

dof, dimen = 6, 6

# Origins of the joints (wrt base frame)
q1 = [0.000, 0.000, 0.089]
q2 = [0.000, 0.136, 0.089]
q3 = [0.425, 0.016, 0.089]
q4 = [0.817, 0.016, 0.089]
q5 = [0.817, 0.109, 0.089]
q6 = [0.817, 0.109, -0.005]
qee = [0.817, 0.191, -0.005]  # base_link to tool0 frame
qr = [q1, q2, q3, q4, q5, q6]
qr_arr = np.array(qr)

# Joint frame orientations (wrt base frame) in quaternions (q = [q0, q1, q2, q3])
w1 = [1.000, 0.000, 0.000, 0.000]
w2 = [0.707, 0.000, 0.707, 0.000]
w3 = [0.707, 0.000, 0.707, 0.000]
w4 = [0.000, 0.000, 1.000, 0.000]
w5 = [0.000, 0.000, 1.000, 0.000]
w6 = [0.000, 0.000, 1.000, 0.000]
wee = [0.000, 0.000, 0.707, 0.707]    # base_link to tool0 frame
wr = [w1, w2, w3, w4, w5, w6]

# Compute rotation matrix from quaternion of each joint
matR = np.zeros((dof, 3, 3))
axis_arr = np.zeros((dof, 3))
for idx, ele in enumerate(wr):
    matR[idx, :, :] = kinlib.unitquat2rotm(ele)

axis_arr[0, :] = matR[0, :3, 2].reshape(1, 3)
axis_arr[1, :] = matR[1, :3, 1].reshape(1, 3)
axis_arr[2, :] = matR[2, :3, 1].reshape(1, 3)
axis_arr[3, :] = matR[3, :3, 1].reshape(1, 3)
axis_arr[4, :] = matR[4, :3, 2].reshape(1, 3)
axis_arr[5, :] = matR[5, :3, 1].reshape(1, 3)

# Define gst0 wrt base frame of left arm
matRee = kinlib.unitquat2rotm(wee)
gst0 = np.vstack((np.hstack((matRee[:, :], np.array(qee).reshape(3, 1))), np.array([0, 0, 0, 1])))
# print(gst0)
