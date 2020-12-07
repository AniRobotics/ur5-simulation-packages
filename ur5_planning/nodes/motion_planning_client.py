#!/usr/bin/env python

import sys
import rospy
from ur5_planning.srv import GetPlan, GetPlanResponse

def conv_float_list(str_data):
    res = str_data.split(",")
    return [float(i) for i in res]

def motion_plan_client(x, y):
    rospy.wait_for_service('ScLERP_motion_plan')
    try:
        obj = rospy.ServiceProxy('ScLERP_motion_plan', GetPlan)
        resp = obj(x, y)
        return resp.plan_out
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    # th_start = [0.10967962633380708, -0.5541505596236057, -0.6807039746241523, 1.5458691389915258, 0.8321845774278369, -0.782330201821561, -0.06481068828815872]
    # goal_position = [1.138, 0.085, 0.193]
    print("================")
    th_start = conv_float_list(sys.argv[1])
    th_goal = conv_float_list(sys.argv[2])
    comp_plan = motion_plan_client(th_start, th_goal)

    dof = len(th_start)
    print("\n=====================================================\n")
    print("\nLast joint solution: ")
    print(comp_plan[-dof:])
    print("Total number of joint solutions: %d"%(len(comp_plan)/dof))
    print("\n=====================================================\n")
