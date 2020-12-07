#!/usr/bin/env python

from scipy.io import loadmat
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import kinematics_library as kinlib
import robot_parameters_new as rpn
import rospkg
import rospy
import marker_plotter as mkp
# from geometry_msgs.msg import Point
from math import sqrt

# imports for marker plotting
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

import threading

# Global variables
dof = 6
tolDist = 0.1
pointList = []
execution_completed = False

# def callback(data):
#     print("angles: ", angles)
#     pass
#
# def all_joints_reached(angles):
#     for _ in range(10):
#         rospy.Subscriber("/joint_states", JointState, callback)
#     return True

def plot_intermediate_markers(marker_publisher):
    while not execution_completed:
        marker1 = Marker(
                type=Marker.SPHERE_LIST,
                id=5,
                lifetime=rospy.Duration(),
                points=pointList,
                scale=Vector3(0.03,0.03,0.03),
                header=Header(frame_id="base_link"),
                color=ColorRGBA(1.0,1.0,1.0,0.8))
        marker_publisher.publish(marker1)
        print("different thread running")
        rospy.sleep(1)
    print("marker plotting thread exitting . . . .")

def pub_js():
    # Initialize the node
    rospy.init_node("joint_value_publisher", anonymous=True)
    rate = rospy.Rate(5) #10hz

    # Publishers
    pubList = [rospy.Publisher("/joint"+str(i+1)+"_position_controller/command", Float64, queue_size=10) for i in range(dof)]

    # Marker publisher
    marker_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=5)
    thread = threading.Thread(target=plot_intermediate_markers,args=[marker_publisher])
    thread.setDaemon(True)
    thread.start()

    # Message list
    msgList = [Float64()]*dof

    # Load the planned jointspace path
    planned_js = loadmat(rospkg.RosPack().get_path("ur5_planning")+"/"+"ScLERP_plan.mat")["theta_traj"]
    numRows = planned_js.shape[1]/dof
    planned_js = planned_js.reshape(numRows,dof)

    # Get the initial and final end-effector poses
    gst_init, _ = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, planned_js[0,:])
    gst_final, _ = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, planned_js[planned_js.shape[0]-1,:])
    mkp.plot_markers(gst_init[:3,3],kinlib.rotm2quat(gst_init[:3,:3]),gst_final[:3,3],kinlib.rotm2quat(gst_final[:3,:3]))
    # mkp.plot_intermediate_markers(gst_init[:3,3],gst_final[:3,3])

    # Start executing the planned motion
    itr = 0
    # pointList = []
    gst_prev = gst_init
    while (itr < planned_js.shape[0]):
        # Read the joint angle
        angles = planned_js[itr,:]
        gst_inter, _ = kinlib.exp_direct_kin(rpn.gst0, rpn.axis_arr, rpn.qr_arr, angles)

        # Plot the intermediate end-effector positions
        if (sqrt((gst_inter[0,3]-gst_prev[0,3])**2+(gst_inter[1,3]-gst_prev[1,3])**2+(gst_inter[2,3]-gst_prev[2,3])**2) > tolDist):
            pointList.append(Point(gst_inter[0,3],gst_inter[1,3],gst_inter[2,3]))

            # Update gst_prev with the current value
            gst_prev = gst_inter

        # Publish the messages to the respective publishers
        if itr > 0:
            for i in range(dof):
                msgList[i].data = angles[i]
                pubList[i].publish(msgList[i])
        else:
            for _ in range(5):
                for i in range(dof):
                    msgList[i].data = angles[i]
                    pubList[i].publish(msgList[i])
                rospy.sleep(1)

        # Sleep for a certain duration
        rate.sleep()
        itr += 1

    # Turn execution_completed flag to True
    execution_completed = True


if __name__ == "__main__":
    try:
        pub_js()
    except rospy.ROSInterruptException:
        pass
