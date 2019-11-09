#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

IN_DISTANCE = 0.1 # object inside distance: meter

fl_distance = 0
fr_distance = 0

def fl_callback(msg):
    global fl_distance
    fl_distance = msg.range

def fr_callback(msg):
    global fr_distance
    fr_distance = msg.range

rospy.init_node("object_inside")
range_fl_sub = rospy.Subscriber("/range/fl", Range, fl_callback, queue_size = 1)
range_fr_sub = rospy.Subscriber("/range/fr", Range, fr_callback, queue_size = 1)
object_inside = rospy.Publisher("/object_inside", Bool, queue_size = 1)


# check if object is inside the bumper
def inside(fl, fr):
    # if fl and fr distances are both smaller than IN_DISTANCE, object is inside the bumper
    if max(fl, fr) < IN_DISTANCE:
        return True
    return False

if __name__ == "__main__":
    while not rospy.is_shutdown():
        object_inside.publish(inside(fl_distance, fr_distance))
