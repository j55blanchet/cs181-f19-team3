#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

RATE = 5
LIN_VEL = 0.2
SAFETY_THRESHOLD = 1.5


class RobotState:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.rate = rospy.Rate(RATE)
        self.state = 0 # 0: stopped, 1: move forward

    def scan_callback(self, msg):
        index = 0
        measurement = msg.ranges[index]
        if measurement <SAFETY_THRESHOLD:
            self.state = 0
        else:
            self.state = 1

        print("Measurement: %0.2f, state: %d" % (measurement, self.state))

    def spin(self):
       vel_msg = Twist()
       while not rospy.is_shutdown():
           if self.state == 0:
               vel_msg.linear.x = 0
           elif self.state == 1:
               vel_msg.linear.x = LIN_VEL
           self.cmd_pub.publish(vel_msg)

           self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fsm_node")
    r = RobotState()
    r.spin()
