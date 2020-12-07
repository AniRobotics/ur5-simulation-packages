import numpy as np
from numpy.linalg import inv
import kinematics_library as kinlib
import dual_quat_library as dqlib
import robot_parameters_new as rpn
from scipy.io import savemat
import rospkg
from math import sqrt

distTol = 0.05

def theta_next(jac, dq_old, dq_new, th, beta_val):
    p_old = dq_old.retrieve_position_vec()
    q_old = dq_old.real.total
    gamma_old = np.hstack((p_old, q_old))
    gamma_new = np.hstack((dq_new.retrieve_position_vec(), dq_new.real.total))

    # Compute p_hat and jacobian1 as per Prof's note
    phat = np.array([[0, -p_old[2], p_old[1]], [p_old[2], 0, -p_old[0]], [-p_old[1], p_old[0], 0]])
    j1 = np.array([[-q_old[1], q_old[0], -q_old[3], q_old[2]],
                   [-q_old[2], q_old[3], q_old[0], -q_old[1]],
                   [-q_old[3], -q_old[2], q_old[1], q_old[0]]])

    # Construct jacobian2 as per Prof's note
    temp1 = np.hstack((np.eye(3, 3), 2*np.dot(phat, j1)))
    temp2 = np.hstack((np.zeros((3, 3)), 2*j1))
    j2 = np.vstack((temp1, temp2))

    # Compute B matrix
    matB = np.dot(np.dot(jac.T, inv(np.dot(jac, jac.T))), j2)
    del_th = beta_val * np.dot(matB, gamma_new - gamma_old)
    return th + del_th


def compute_motion_plan(theta_init, theta_final):
    beta = 0.02

    # Solve forward kinematics for left arm
    gst_init, transform_upto_joint_init = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, theta_init)
    gst_final, transform_upto_joint_final = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, theta_final)

    # Convert the initial and final end-effector poses into dual-quaternion form
    dq_initial = dqlib.homo2dualquat(gst_init)
    dq_final = dqlib.homo2dualquat(gst_final)

    # Planning loop
    theta_traj = list()
    theta_prev = theta_init
    theta_prev_added = theta_init
    dq_prev = dq_initial.copy_new()
    reached = False
    while not reached:
        interpolate_obj = dqlib.IntpRigidDispDualQuat(dq_prev, dq_final)
        dq_current = interpolate_obj.interpolate_one_step(tau=0.5)

        # Compute Jacobian
        jac_s, jac_a = kinlib.velocity_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, theta_prev)

        # Compute theta for next step
        theta_current = theta_next(jac_s, dq_prev, dq_current, theta_prev, beta)

        # Solve forward kinematics for left arm
        gst, transform_upto_joint = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, theta_current)
        dq_current = dqlib.homo2dualquat(gst.copy())

        # Termination criterion: Find the distance between dq_current and dq_final
        current_positional_dist = kinlib.norm(dq_current.retrieve_position_vec() - dq_final.retrieve_position_vec())

        # print("current_positional_dist: %2.2f"%(current_positional_dist))
        if current_positional_dist < 0.0010:
            reached = True

        # check if you have moved enough distance in joint space
        theta_current = theta_current.tolist()
        moved_dist = 0
        for idx in range(len(theta_current)):
            moved_dist += (theta_current[idx]-theta_prev_added[idx])**2

        if (sqrt(moved_dist) > distTol):
            theta_traj += theta_current
            theta_prev_added = theta_current

        # Overwrite previous variables with current ones
        theta_prev = theta_current
        dq_prev = dq_current.copy_new()

    # save the plan in a *.mat file to make the plan available for execution
    traj_data = dict()
    traj_data["theta_traj"] = theta_traj
    savemat(rospkg.RosPack().get_path("ur5_planning")+"/"+"ScLERP_plan.mat",traj_data)
    print("Plan is saved in file named ScLERP_plan.mat")
    return theta_traj
