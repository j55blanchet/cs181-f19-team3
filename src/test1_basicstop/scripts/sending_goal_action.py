#!/usr/bin/env python

import rospy
import actionlib
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Quaternion


class ARTag:
    def __init__(self, marker):
        self.id = marker.id
        self.position = Pose()
        self.position = marker.pose.pose.position #Pose
        self.orientation = Quaternion()
        self.orientation = marker.pose.pose.orientation #Quaternion

class ARTags:
    def __init__(self):
        self.ar_tag_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
        self.num = 0 # number of ar tag
        self.markers = [] #ar tag list

    def ar_callback(self, msg):
        self.num = len(msg.markers)
        self.markers = []
        for marker in msg.markers: #AlvarMarker
            self.markers.append(ARTag(marker))

class Husarion_bot():
    def __init__(self):
        self.treasure_marker = None
        self.treasure_goal = None
        self.final_marker = None
        self.final_goal = None
        self.client = None
        self.treasure_goal_state = 0 # 0: not reached, 1: reached
        self.final_goal_state = 0 # 0: not reached, 1: reached

    def move_base_client_treasure(self):
        # declaration of action client using the topic "move_base" and msg type "MoveBaseAction"

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        # define the treasure goal based on msg type MoveBaseGoal
        self.treasure_goal = MoveBaseGoal()
        self.treasure_goal.target_pose.header.frame_id = "map"
        self.treasure_goal.target_pose.header.stamp = rospy.Time.now()
        # TODO: receive the pose of the treasure_goal
        self.treasure_goal.target_pose.pose.position.x = self.treasure_marker.position.x
        self.treasure_goal.target_pose.pose.position.y = self.treasure_marker.position.y
        self.treasure_goal.target_pose.pose.orientation = self.treasure_marker.orientation
        #self.treasure_goal.target_pose.pose.orientation.w = 1.0

        # After updating, sending the robot to the goal
        self.client.send_goal(self.treasure_goal)
        waiting = self.client.wait_for_result()

        if not waiting:
            rospy.logerr("Action server is not available")
            rospy.signal_shutdwon("Action server not available")
        else:
            return self.client.get_result()

    def move_base_client_final_goal(self):
        # declaration of action client using the topic "move_base" and msg type "MoveBaseAction"

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        # define the treasure goal based on msg type MoveBaseGoal
        self.final_goal = MoveBaseGoal()
        self.final_goal.target_pose.header.frame_id = "map"
        self.final_goal.target_pose.header.stamp = rospy.Time.now()
        # TODO: receive the pose of the treasure_goal
        self.final_goal.target_pose.pose.position.x = self.final_marker.position.x
        self.final_goal.target_pose.pose.position.y = self.final_marker.position.y
        self.final_goal.target_pose.pose.orientation = self.final_marker.orientation
        #self.final_goal.target_pose.pose.orientation.w = 1.0

        # After updating, sending the robot to the goal
        self.client.send_goal(self.final_goal)
        waiting = self.client.wait_for_result()

        if not waiting:
            rospy.logerr("Action server is not available")
            rospy.signal_shutdwon("Action server not available")
        else:
            return self.client.get_result()


if __name__ == '__main__':
    rospy.init_node("finding_AR_tag_and_move_base_client")
    Rosbot = Husarion_bot()
    AR_box = ARTags()
    # going to the treasure point
    while True:
        for marker in AR_box.markers:
            if marker.id == 0: #starting point
                Rosbot.treasure_marker = marker
                print("Treasure found")
            elif marker.id == 9:
                Rosbot.final_marker = marker
                print("Goal found")

        if Rosbot.final_marker is not None and Rosbot.treasure_marker is not None:
            break

    if Rosbot.treasure_marker is not None and Rosbot.final_marker is not None and Rosbot.treasure_goal_state == 0 and Rosbot.final_goal_state == 0:
        try:
            rospy.loginfo("Heading to the treasure")
            result_treasure = Rosbot.move_base_client_treasure()
            # success to treasure point
            if result_treasure:
                Rosbot.treasure_goal_state = 1
                rospy.loginfo("Treasure found")

                # going to the goal point
                if Rosbot.treasure_goal_state == 1 and Rosbot.final_goal_state == 0:
                    try:
                        rospy.loginfo("Heading to the goal")
                        result_final_goal = Rosbot.move_base_client_final_goal()
                    # success to the goal point
                        if result_final_goal:
                            Rosbot.final_goal_state = 1
                            rospy.loginfo("Fianl goal found")
                    # failure to the goal point
                    except rospy.ROSInterruptException:
                        rospy.loginfo("Finding a final goal has failed")

    # failure to the treasure point
        except rospy.ROSInterruptException:
            rospy.loginfo("Finding a treasure has failed")
