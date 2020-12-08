#!/usr/bin/env python

import sys
import rospy
import getch
from ur5_teleop.srv import TeleOp, TeleOpResponse

def teleop_client():
    rospy.wait_for_service("teleop_service")
    obj = rospy.ServiceProxy("teleop_service",TeleOp)
    try:
        while not rospy.is_shutdown():
            char_val = getch.getch()
            resp = obj(ord(char_val)-ord('a'))
            print("Got service response: ")
            print(resp.movement_success)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    teleop_client()
