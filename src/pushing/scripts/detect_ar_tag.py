#!/usr/bin/env python
import rospy
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from geometry_msgs


rospy.init_node("detect_ar_tag")
ar_tag_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback, queue_size=0)

def ar_callback(msg):
    for marker in msg.markers: #AlvarMarker
        pose = marker.pose.pose
