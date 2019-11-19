#!/usr/bin/env python
import rospy 
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Twist

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
        ## rospy.init_node("detect_ar_tag")
        self.ar_tag_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
        self.num = 0 # number of ar tag
        self.markers = [] #ar tag list

    def ar_callback(self, msg):
        self.num = len(msg.markers)
        self.markers = []
        for marker in msg.markers: #AlvarMarker
            self.markers.append(ARTag(marker))
    
class DirectRobo:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.vel_msg = Twist()
        self.ar_tags = ARTags()

    def rotation(self, err):
        if (err > 0):
           self.vel_msg.angular.z = 0.1
        else:
           self.vel_msg.angular.z = -0.1
  
    def directing(self):
        rospy.init_node("direct_robot")
        if (self.ar_tags.num > 0 and (self.ar_tags.markers[0] == 0 or self.ar_tags.marker[0] == 9)):
           marker = self.ar_tags.markers[0]
           x = marker.position.x
           y = marker.position.y
           z = marker.position.z
           while (abs(y) > 0.3):
             rotation(self, y)
             self.vel_pub.publish(self.vel_msg)
           self.vel_msg.angular.z = 0
           self.vel_pub.publish(self.vel_msg)
           while (x > 0.3):
             self.vel_msg.linear.x = 0.2
             self.vel_pub.publish(self.vel_msg)
           self.vel_msg.linear.x = 0 
           self.vel_pub.publish(self.vel_msg)


if __name__ == "__main__":
    robot = DirectRobo()
    while not rospy.is_shutdown():
      robot.directing() 



                                                                               1,1           Top

