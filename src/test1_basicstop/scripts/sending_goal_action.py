#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class Husarion_bot():
    def __init__(self):
        self.treasure_goal = None
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
        self.treasure_goal.target_pose.pose.position.x = 3.0
        self.treasure_goal.target_pose.pose.position.y = 3.0
        self.treasure_goal.target_pose.pose.orientation.w = 1.0


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
        self.final_goal.target_pose.pose.position.x = 0.5
        self.final_goal.target_pose.pose.position.y = 0.3
        self.final_goal.target_pose.pose.orientation.w = 1.0


        self.client.send_goal(self.final_goal)
        waiting = self.client.wait_for_result()

        if not waiting:
            rospy.logerr("Action server is not available")
            rospy.signal_shutdwon("Action server not available")
        else:
            return self.client.get_result()


if __name__ == '__main__':
    rospy.init_node("move_base_client")
    Rosbot = Husarion_bot()
    if Rosbot.treasure_goal_state == 0 and Rosbot.final_goal_state == 0:
        try:
            rospy.loginfo("Heading to the tresure")
            result_treasure = Rosbot.move_base_client_treasure()
            if result_treasure:
                Rosbot.treasure_goal_state = 1
                rospy.loginfo("Treasure found")

                if Rosbot.treasure_goal_state == 1 and Rosbot.final_goal_state == 0:
                    try:
                        rospy.loginfo("Heading to the goal")
                        result_final_goal = Rosbot.move_base_client_final_goal()
                        if result_final_goal:
                            Rosbot.final_goal_state = 1
                            rospy.loginfo("Fianl goal found")
                    except rospy.ROSInterruptException:
                        rospy.loginfo("Finding a final goal has failed")

        except rospy.ROSInterruptException:
            rospy.loginfo("Finding a treasure has failed")
