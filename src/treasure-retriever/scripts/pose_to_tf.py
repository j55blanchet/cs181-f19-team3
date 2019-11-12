#!/usr/bin/env python

"""Simple node that publishes pose of robot as a transformation (compared to odom)"""

import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion, Transform
from tf import TransformBroadcaster

def pose_callback(msg, tf_broadcaster):

    pose = msg.pose

    q = Quaternion()
    q.x = pose.orientation.x
    q.y = pose.orientation.y
    q.z = pose.orientation.z
    q.w = pose.orientation.w

    tf_broadcaster = TransformBroadcaster()
    tf_broadcaster.sendTransform(
        translation=Vector3(pose.position.x, pose.position.y, 0.0),
        rotation=q,
        time=rospy.Time(), 
        child="odom",
        parent="base_link")

def main():
    rospy.init_node("pose_to_tf")
    tf_broadcaster = tf_broadcaster = TransformBroadcaster()
    rospy.Subscriber("/pose", PoseStamped, pose_callback, queue_size=1, callback_args=tf_broadcaster)
    rospy.spin()
    
if __name__ == "__main__":
    main()