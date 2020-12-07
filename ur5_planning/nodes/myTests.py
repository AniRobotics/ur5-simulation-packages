import numpy as np
import kinematics_library as kinlib
import robot_parameters_new as rpn

# Solve forward kinematics
theta_init = [-0.03,-0.95,0.79,-1.46,-1.58,-0.03]
gst_init, transform_upto_joint_init = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, theta_init)
print("g_initial")
print(gst_init)

theta_final = [1.35,-0.95,0.79,-1.46,-1.58,-0.03]
gst_final, _ = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, theta_final)
print("g_final")
print(gst_final)
