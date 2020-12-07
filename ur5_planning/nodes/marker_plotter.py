#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

def plot_markers(pi,qi,pf,qf):
    marker_publisher = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=5)

    startPose = Pose(Point(pi[0],pi[1],pi[2]), Quaternion(qi[3],qi[0],qi[1],qi[2]))
    goalPose = Pose(Point(pf[0],pf[1],pf[2]), Quaternion(qf[3],qf[0],qf[1],qf[2]))

    marker1 = Marker(
            type=Marker.SPHERE,
            id=0,
            lifetime=rospy.Duration(),
            pose=startPose,
            scale=Vector3(0.06,0.06,0.06),
            header=Header(frame_id="base_link"),
            color=ColorRGBA(0.0,1.0,0.0,0.8))

    marker2 = Marker(
            type=Marker.SPHERE,
            id=1,
            lifetime=rospy.Duration(),
            pose=goalPose,
            scale=Vector3(0.06,0.06,0.06),
            header=Header(frame_id="base_link"),
            color=ColorRGBA(1.0,0.0,0.0,0.8))

    marker3 = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=2,
            lifetime=rospy.Duration(),
            pose=startPose,
            scale=Vector3(0.06,0.06,0.06),
            header=Header(frame_id="base_link"),
            color=ColorRGBA(0.0,1.0,0.0,0.8),
            text="start")

    marker4 = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=3,
            lifetime=rospy.Duration(),
            pose=goalPose,
            scale=Vector3(0.06,0.06,0.06),
            header=Header(frame_id="base_link"),
            color=ColorRGBA(1.0,0.0,0.0,0.8),
            text="goal")

    rospy.sleep(1)
    marker_publisher.publish([marker1,marker2,marker3,marker4])

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
