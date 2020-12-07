#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

def show_text_in_rviz(marker_publisher):
    startPose = Pose(Point(0.1,0.1,0.2), Quaternion(0,0,0,1))
    goalPose = Pose(Point(0.5,0.5,0.2), Quaternion(0,0,0,1))

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

if __name__ == "__main__":
    rospy.init_node("marker_publisher", anonymous=True)
    marker_publisher = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=5)
    try:
        show_text_in_rviz(marker_publisher)
    except rospy.ROSInterruptException:
        pass
