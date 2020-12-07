#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def pub_js():
    pub_j1 = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=10)
    pub_j2 = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=10)
    pub_j3 = rospy.Publisher("/joint3_position_controller/command", Float64, queue_size=10)
    pub_j4 = rospy.Publisher("/joint4_position_controller/command", Float64, queue_size=10)
    pub_j5 = rospy.Publisher("/joint5_position_controller/command", Float64, queue_size=10)
    pub_j6 = rospy.Publisher("/joint6_position_controller/command", Float64, queue_size=10)

    rospy.init_node("move_joint", anonymous=True)
    rate = rospy.Rate(10) #10hz

    msg_j1 = Float64()
    msg_j2 = Float64()
    msg_j3 = Float64()
    msg_j4 = Float64()
    msg_j5 = Float64()
    msg_j6 = Float64()

    while not rospy.is_shutdown():
        # angles = [-0.03,-0.95,0.79,-1.46,-1.58,-0.03]
        angles = [1.35,-0.95,0.79,-1.46,-1.58,-0.03]
        msg_j1.data = angles[0]
        msg_j2.data = angles[1]
        msg_j3.data = angles[2]
        msg_j4.data = angles[3]
        msg_j5.data = angles[4]
        msg_j6.data = angles[5]

        pub_j1.publish(msg_j1)
        pub_j2.publish(msg_j2)
        pub_j3.publish(msg_j3)
        pub_j4.publish(msg_j4)
        pub_j5.publish(msg_j5)
        pub_j6.publish(msg_j6)

        rate.sleep()

if __name__ == "__main__":
    try:
        pub_js()
    except rospy.ROSInterruptException:
        pass
