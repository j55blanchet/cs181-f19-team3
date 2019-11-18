#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from find_object_2d.msg import ObjectsStamped

class DetectGoal:
  def __init__(self):
    self.cmd_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1)
    self.find_sub = rospy.Subscriber("objectsStamped", ObjectsStamped, self.callback, queue_size = 1)
    self.found = False
    self.rate = rospy.Rate(5)

  def callback(self, msg):
    if (len(msg.objects.data) > 0):
      self.found = True
    else:
      self.found = False
    print(msg, self.found)

  def spin(self):
    vel_msg = Twist()
    while not rospy.is_shutdown():
      if (self.found):
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.3
      else:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
      self.cmd_pub.publish(vel_msg)
      self.rate.sleep()

if __name__ == "__main__":
  rospy.init_node("goaldetect_node")
  r = DetectGoal()
  r.spin()      
