#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, Quaternion

class ARTag:
    def __init__(self):
        rospy.init_node("detect_ar_tag")
        self.ar_tag_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
        self.artag_exist = False # number of ar tag
        self.marker = AlvarMarker() #ar tag

        self.id = self.marker.id
        self.position = Pose()
        self.orientation = Quaternion()

    def ar_callback(self, msg):
        if msg.markers:
            self.artag_exist = True
            self.marker = msg.markers[0]
            self.id = self.marker.id
            self.position = self.marker.pose.pose.position #Pose
            self.orientation = self.marker.pose.pose.orientation #Quaternion
        else:
            self.artag_exist = False

if __name__ == "__main__":
    robot = ARTag()
    while not rospy.is_shutdown():
        if robot.artag_exist:
            print("id:", robot.id)
            print("position:")
            print("x:", robot.position.x)
            print("y:", robot.position.y)
            print("z:", robot.position.z)
            print("orientation:")
            print("x:", robot.orientation.x)
            print("y:", robot.orientation.y)
            print("z:", robot.orientation.z)
            print("w:", robot.orientation.w)
            print("-------------------------------")
        else:
            print("no ARTag")
