#!/usr/bin/env python

"""Simple node that publishes pose of robot as a transformation (compared to odom)"""

import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion, Transform
from tf import TransformBroadcaster

def main():
    rospy.init_node("pose_to_tf")

    tf_broadcaster = TransformBroadcaster()

    def pose_callback(msg):

        pose = msg.pose
        q = pose.orientation

        rospy.loginfo_throttle(10, "Updating tf from odom to base_link")

        tf_broadcaster.sendTransform(
            translation=(pose.position.x, pose.position.y, 0),
            rotation=(q.x, q.y, q.z, q.w),
            time=rospy.Time.now(), 
            child="base_link",
            parent="odom")

    rospy.Subscriber("/pose", PoseStamped, pose_callback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
