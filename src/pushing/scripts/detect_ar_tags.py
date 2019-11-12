#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, Quaternion, PoseStamped

# get the ar tag data: id, position, orientation
# a marker list, every item is a ARTag object

class ARTag:
    def __init__(self, marker):
        self.id = marker.id
        self.position = Pose()
        self.position = marker.pose.pose.position #Pose
        self.orientation = Quaternion()
        self.orientation = marker.pose.pose.orientation #Quaternion


class ARTags:
    def __init__(self):
        rospy.init_node("detect_ar_tag")
        self.ar_tag_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
        self.num = 0 # number of ar tag
        self.markers = [] #ar tag list

    def ar_callback(self, msg):
        self.num = len(msg.markers)
        self.markers = []
        for marker in msg.markers: #AlvarMarker
            self.markers.append(ARTag(marker))


if __name__ == "__main__":
    robot = ARTags()
    while not rospy.is_shutdown():
        if robot.num != 0:
            for marker in robot.markers:
                print("id:", marker.id)
                print("position:")
                print("x:", marker.position.x)
                print("y:", marker.position.y)
                print("z:", marker.position.z)
                print("orientation:")
                print("x:", marker.orientation.x)
                print("y:", marker.orientation.y)
                print("z:", marker.orientation.z)
                print("w:", marker.orientation.w)
                print("-------------------------------")
        else:
            print("no ARTag")
