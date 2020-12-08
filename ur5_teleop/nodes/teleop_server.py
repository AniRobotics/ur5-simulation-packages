#!/usr/bin/env python

from ur5_teleop.srv import TeleOp, TeleOpResponse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import threading

# Publishers to all the joints
dof = 6
pubList = [rospy.Publisher("/joint"+str(i+1)+"_position_controller/command", Float64, queue_size=10) for i in range(dof)]
current_angles = None

def callback(msg):
    global current_angles
    current_angles = list(msg.position)
    # print(current_angles)

def handle_teleop(req):
    global current_angles
    msg = Float64()

    num = req.joint_movement_index
    idx = None
    # Be careful about the order of the current_angles:
    # current_angles = [elbow_joint, shoulder_lift_joint, shoulder_pan_joint,
    # wrist_1_joint, wrist_2_joint, wrist_3_joint]
    if (num+ord('a') == ord('a')):
        pub_idx, ang_idx = 0, 2
        print("increase shoulder_link")
    elif (num+ord('a') == ord('s')):
        pub_idx, ang_idx = 1, 1
        print("increase upper_arm_link")
    elif (num+ord('a') == ord('d')):
        pub_idx, ang_idx = 2, 0
        print("increase forearm_link")
    elif (num+ord('a') == ord('q')):
        pub_idx, ang_idx = 3, 3
        print("increase wrist_1_link")
    elif (num+ord('a') == ord('w')):
        pub_idx, ang_idx = 4, 4
        print("increase wrist_2_link")
    elif (num+ord('a') == ord('e')):
        pub_idx, ang_idx = 5, 5
        print("increase wrist_3_link")

    if not ang_idx is None:
        print(current_angles[ang_idx])
        msg.data = 0.1+current_angles[ang_idx]
        print(msg.data)
        for _ in range(2):
            pubList[pub_idx].publish(msg)
            rospy.sleep(0.1)
        print(current_angles[ang_idx])

    return TeleOpResponse(1)

# def joint_state_subscriber():
#      rospy.Subscriber("/joint_states",JointState,callback)
#      rospy.spin()

def teleop_server():
    rospy.Subscriber("/joint_states",JointState,callback)
    rospy.Service("teleop_service",TeleOp,handle_teleop)
    print("Ready to tele operate UR5 ...")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("teleop_server")
    # thread = threading.Thread(target=joint_state_subscriber,args=[])
    # thread.setDaemon(True)
    # thread.start()
    # joint_state_subscriber()
    teleop_server()
