#!/usr/bin/env python

import rospy
import numpy
import math
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

RATE = 10 # rospy.rate

LIN_VEL = 0.3 # linear velocity
TURN_VEL = 0.3 # angular velocity only in case of evacuation
SAFETY_THRESHOLD = 0.6 # wall distance for following
DURATION_STOP = 3
DURATION_FWD = 7

class WanderingRobot:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.rate = rospy.Rate(RATE)
        self.distance_ahead = 0 # initial value
        self.driving_state = True
        self.stop_watch = rospy.Time.now()
        self.rate = rospy.Rate(RATE)


    def scan_callback(self, msg):
        filtering = msg.ranges[len(msg.ranges)*4/12:len(msg.ranges)*8/12]
        self.distance_ahead = min(msg.ranges)

    def spin(self):
        while not rospy.is_shutdown():
            vel_msg = Twist()
            if self.driving_state:
                if self.distance_ahead < SAFETY_THRESHOLD or rospy.Time.now() > self.stop_watch:
                    self.driving_state = False
                    self.stop_watch = rospy.Time.now() + rospy.Duration(DURATION_STOP)

            else:
                if rospy.Time.now() > self.stop_watch:
                    self.driving_state = True
                    self.stop_watch = rospy.Time.now() + rospy.Duration(DURATION_FWD)

            if self.driving_state:
                vel_msg.linear.x = LIN_VEL
                print("STATE", self.driving_state)
                print("I am going forward")
                print("Distance", self.distance_ahead)
            else:
                vel_msg.angular.z = TURN_VEL
                print("STATE", self.driving_state)
                print("I am turning")
                print("Distance", self.distance_ahead)

            self.cmd_pub.publish(vel_msg)
            self.rate.sleep()

def main():
    # class instance initiation
    robot = WanderingRobot()
    # initial decision of where_to_go depending on the starting sensor measurement
    robot.spin()

if __name__ == "__main__":
    rospy.init_node("wandering")
    main()
