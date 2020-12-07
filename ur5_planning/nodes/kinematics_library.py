import numpy as np
from numpy.linalg import inv
# import math


# Finds exponential of twist vector
def exp_twist(xi, theta):
    omega = xi[3:]
    v = xi[:3]

    omega_hat = np.array([[0, -omega[2], omega[1]],
                          [omega[2], 0, -omega[0]],
                          [-omega[1], omega[0], 0]])

    omega_hat_theta = np.identity(3) + np.sin(theta)*omega_hat + (1-np.cos(theta))*np.dot(omega_hat, omega_hat)
    p_val = np.dot((np.identity(3) - omega_hat_theta), np.cross(omega, v)) + np.dot(np.outer(omega, omega), v)*theta
    gab = np.vstack((np.hstack((omega_hat_theta, p_val.reshape(3, 1))), np.array([[0, 0, 0, 1]])))
    return gab


# Finds forward kinematics using Exponential coordinates
def exp_direct_kin(gst0, joint_axes, q_axes, th):
    dim = 3
    num_of_joints = len(th)
    gst_temp = np.identity(dim+1)
    tran_upto_joint = np.zeros((num_of_joints+1, dim+1, dim+1))

    for i in range(num_of_joints):
        tran_upto_joint[i, :, :] = gst_temp
        omega = joint_axes[i, :]
        q = q_axes[i, :]
        xi = np.hstack((np.cross(-omega, q), omega))
        gst_joint_i = exp_twist(xi, th[i])
        gst_temp = np.dot(gst_temp, gst_joint_i)

    tran_upto_joint[num_of_joints, :, :] = gst_temp
    gst = np.dot(gst_temp, gst0)
    return gst, tran_upto_joint


# Finds spatial Jacobian using Exponential coordinates
def velocity_direct_kin(gst0, joint_axes, q_axes, th):
    dim = 3
    num_of_joints = len(th)
    spatial_jac = np.zeros((dim+3, num_of_joints))

    gst, transform_upto_joint = exp_direct_kin(gst0, joint_axes, q_axes, th)
    for i in range(num_of_joints):
        if i > 0:
            g = transform_upto_joint[i, :, :]
            R = g[:3, :3]
            p = g[:3, 3]
            p_hat = np.array([[0, -p[2], p[1]],
                              [p[2], 0, -p[0]],
                              [-p[1], p[0], 0]])
            temp1 = np.dot(p_hat, R)
            Ad_g = np.hstack((R, temp1))
            temp2 = np.hstack((np.zeros((3, 3)), R))
            Ad_g = np.vstack((Ad_g, temp2))

        omega = joint_axes[i, :]
        q = q_axes[i, :]
        xi = np.hstack((np.cross(-omega, q), omega))

        if i > 0:
            xi_prime = np.dot(Ad_g, xi)
        else:
            xi_prime = xi

        spatial_jac[:, i] = xi_prime

    # Compute analytical jacobian
    p = gst[:3, 3]
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    pre_mult_mat = np.vstack((np.hstack((np.eye(3), -p_hat)), np.hstack((np.zeros((3, 3)), np.eye(3)))))
    analytical_jac = np.dot(pre_mult_mat, spatial_jac)

    return spatial_jac, analytical_jac


# Quaternion to rotation matrix
def unitquat2rotm(q):
    norm_val = norm(q)
    nq = [ele/norm_val for ele in q]
    return np.array([[nq[0]**2 + nq[1]**2 - nq[2]**2 - nq[3]**2,
            2 * nq[1] * nq[2] - 2 * nq[0] * nq[3], 2 * nq[1] * nq[3] + 2 * nq[0] * nq[2]],
            [2 * nq[1] * nq[2] + 2 * nq[0] * nq[3], nq[0]**2 - nq[1]**2 + nq[2]**2 - nq[3]**2, 2 * nq[2] * nq[3] - 2 * nq[0] * nq[1]],
            [2 * nq[1] * nq[3] - 2 * nq[0] * nq[2], 2 * nq[2] * nq[3] + 2 * nq[0] * nq[1],
            nq[0]**2 - nq[1]**2 - nq[2]**2 + nq[3]**2]])


# Rotation matrix to quaternion
def rotm2quat(rotm):
    nrotm = np.zeros((3, 3))
    for i in range(3):
        nrotm[:, i] = rotm[:, i]/norm(rotm[:, i])
    if (nrotm[0, 0] + nrotm[1, 1] + nrotm[2, 2] - 1) / 2 < -1 or (nrotm[0, 0] + nrotm[1, 1] + nrotm[2, 2] - 1) / 2 > 1:
        return None
    angle = np.arccos((nrotm[0, 0] + nrotm[1, 1] + nrotm[2, 2] - 1) / 2)
    temp = np.sqrt((nrotm[2, 1] - nrotm[1, 2])**2 + (nrotm[0, 2] - nrotm[2, 0])**2 + (nrotm[1, 0] - nrotm[0, 1])**2)
    wx = (nrotm[2, 1] - nrotm[1, 2]) / temp
    wy = (nrotm[0, 2] - nrotm[2, 0]) / temp
    wz = (nrotm[1, 0] - nrotm[0, 1]) / temp
    sin_val = np.sin(angle / 2)
    quat = np.array([np.cos(angle / 2), wx * sin_val, wy * sin_val, wz * sin_val])
    return quat


# Compute adjoint matrix given a transformation
def get_adjoint(g):
    R, p = g[:3, :3], g[:3, 3]
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    temp1 = np.dot(p_hat, R)
    temp1 = np.hstack((R, temp1))
    temp2 = np.hstack((np.zeros((3, 3)), R))
    return np.vstack((temp1, temp2))


# Compute relative jacobian
def relative_jac(gst0_l, gst0_r, gst_l, gst_r, transform_upto_l, transform_upto_r, joint_axes, q_axes, theta):
    dim = 3
    num_of_joints = len(theta)
    spatial_jac = np.zeros((dim + 3, num_of_joints))

    # Fill-in the first 7 columns of spatial jacobian matrix
    num = num_of_joints//2
    for i in range(num):
        if i < num - 1:
            g = np.dot(inv(transform_upto_l[i+1, :, :]), gst_l)
        else:
            g = gst0_l
        Ad_g = get_adjoint(g)
        Ad_g_inv = np.dot(inv(Ad_g), np.eye(6))

        omega = joint_axes[i, :]
        q = q_axes[i, :]
        xi = np.hstack((np.cross(-omega, q), omega))

        xi_prime = -np.dot(Ad_g_inv, xi)
        spatial_jac[:, num - 1 - i] = xi_prime

    # Fill-in the last 7 columns of spatial jacobian matrix
    for j in range(num):
        if j > 0:
            g = np.dot(inv(gst_l), transform_upto_r[j+1, :, :])
        else:
            g = gst_l
        Ad_g = get_adjoint(g)
        omega = joint_axes[j + num, :]
        q = q_axes[j + num, :]
        xi = np.hstack((np.cross(-omega, q), omega))

        if j > 0:
            xi_prime = np.dot(Ad_g, xi)
        else:
            Ad_g_inv = np.dot(inv(Ad_g), np.eye(6))
            xi_prime = np.dot(Ad_g_inv, xi)

        spatial_jac[:, num + j] = xi_prime

    # Compute analytical jacobian and return
    g_rel = np.dot(inv(gst_l), gst_r)
    p = g_rel[:3, 3]
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    pre_mult_mat = np.vstack((np.hstack((np.eye(3), -p_hat)), np.hstack((np.zeros((3, 3)), np.eye(3)))))
    analytical_jac = np.dot(pre_mult_mat, spatial_jac)
    return analytical_jac, spatial_jac


# Compute skew-symmetric quaternion
def skew_quat(q):
    # This matches with Yan Bin Jia's Quaternion Notes
    return np.array([[-q[1], q[0], -q[3], q[2]],
                     [-q[2], q[3], q[0], -q[1]],
                     [-q[3], -q[2], q[1], q[0]]])
    # Below one does not match with Yan Bin Jia's Quaternion Notes.
    # Obtained from Barfoot's derivation which we think has a
    # potentially wrong sign.
    # return np.array([[-q[1], q[0], q[3], -q[2]],
    #                 [-q[2], -q[3], q[0], q[1]],
    #                 [-q[3], q[2], -q[1], q[0]]])


# Compute norm of a vector
def norm(v):
    return np.sqrt(np.dot(v, v))


# Position error bound
def position_err_bound(jp, cval):
    w, v = np.linalg.eig(np.dot(jp, jp.T))
    max_id = np.argmax(w)
    max_eig = w[max_id]
    return np.sqrt(cval*max_eig)


# Jacobian relating [v, w] to [p_dot, q_dot]
def jac_vw2dpdq(p, q):
    j1 = skew_quat(q)
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    temp_upper = np.hstack((np.eye(3), 2*np.dot(p_hat, j1)))
    temp_lower = np.hstack((np.zeros((3, 3)), 2*j1))
    return np.vstack((temp_upper, temp_lower))
