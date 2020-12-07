#!/usr/bin/env python

from ur5_planning.srv import GetPlan,GetPlanResponse
import motionplan_library as mplib
import rospy

def handle_motion_plan(req):
    print("Planning request received . . . .")
    joint_plan = mplib.compute_motion_plan(req.initial_joint_vec, req.final_joint_vec)
    print("Planning done . . . .\n")
    return GetPlanResponse(joint_plan)

def motion_plan_server():
    rospy.init_node('motion_plan_server')
    s = rospy.Service('ScLERP_motion_plan', GetPlan, handle_motion_plan)
    print "Ready to compute motion plan . . . . . . "
    rospy.spin()

if __name__ == "__main__":
    motion_plan_server()
