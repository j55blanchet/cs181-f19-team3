#!/usr/bin/env python

"""Simple node that publishes pose of robot as a transformation (compared to odom)"""

import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion, Transform, Pose
from tf import TransformBroadcaster
from nav_msgs.msg import Odometry

def main():
    rospy.init_node("pose_to_tf2")

    tf_broadcaster = TransformBroadcaster()

    def pose_callback(msg):

        pose = msg.pose
        q = pose.orientation
        # q = Quaternion()
        # q.x = pose.orientation.x
        # q.y = pose.orientation.y
        # q.z = pose.orientation.z
        # q.w = pose.orientation.w

        tf_broadcaster.sendTransform(
            translation=(pose.position.x, pose.position.y, 0),
            rotation=(q.x, q.y, q.z, q.w),
            time=rospy.Time.now(),
            child="map",
            parent="odom")

    rospy.Subscriber("/map", Pose, pose_callback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
