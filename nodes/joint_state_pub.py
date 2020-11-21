#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_states():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('ur5_joint_state_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    joint_state_obj = JointState()
    joint_state_obj.header = Header()
    joint_state_obj.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    joint_state_obj.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    counter = 0
    while not rospy.is_shutdown():
        joint_state_obj.header.stamp = rospy.Time.now()
        joint_state_obj.position[0] = 0.8
        joint_state_obj.position[1] = 0.0
        joint_state_obj.position[2] = 0.0
        joint_state_obj.position[3] = 0.0
        joint_state_obj.position[4] = 0.0
        joint_state_obj.position[5] = 0.0
        joint_state_obj.velocity = []
        joint_state_obj.effort = []
        pub.publish(joint_state_obj)
        counter += 1
        rate.sleep()
        # Keep publishing the last joint angles
        joint_state_obj.header.stamp = rospy.Time.now()
        pub.publish(joint_state_obj)

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass


