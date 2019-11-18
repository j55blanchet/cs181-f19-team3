#!/usr/bin/env python
import sys
import select
import os
import rospy
import actionlib
import roslaunch
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Twist, Vector3, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from grid_map_msgs.msg import GridMap

class State:
    INIT = 0
    SEARCH_OBJECTIVES = 1
    FETCH_TREASURE = 2
    DELIVER_TREASURE = 3
    DONE = 4

class MarkerIds:
    GOAL_ZONE = 0
    TREASURE  = 1

map_save_file = "/home/husarion/catkin_ws/maps/autogenmap"
if 'MAP_SAVE_FILE' in os.environ:
    map_save_file = os.environ['MAP_SAVE_FILE']

class MainController:
    
    def __init__(self):
        self.state = State.INIT
        self.rate = rospy.Rate(10)
        self.action_seq = 0

        self.map_saver_process = None
        self.map_server_process = None
        self.keyboard_teleop_process = None

        self.goalzone_pose = None
        self.treasure_pose = None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.ar_tag_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
        self.visualizations_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()

    
    def perform_action(self):
        self.action_seq += 1

        rospy.loginfo_throttle(1, "MainController Action#[%04d] State: %s" % (self.action_seq, str(self.state)))

        if self.state is State.INIT:
            rospy.loginfo("Starting objective search")
            self.keyboard_teleop_process = self.start_node("teleop_twist_keyboard", "teleop_twist_keyboard.py", "")
            self.state = State.SEARCH_OBJECTIVES
            rospy.loginfo("Starting teleoperation. Enter 'q' when finished mapping")

        elif self.state is State.SEARCH_OBJECTIVES:
            if not self.keyboard_teleop_process.is_alive():
                rospy.loginfo("Mapping complete!")
                self.cmd_pub.publish(Twist())
                self.state = State.FETCH_TREASURE
            
        elif self.state is State.FETCH_TREASURE:
            if self.goto_pose(self.treasure_pose):
                self.state = State.DELIVER_TREASURE
                rospy.loginfo("Reached treasure! Heading to delivery now")
        
        elif self.state is State.DELIVER_TREASURE:
            if self.goto_pose(self.goalzone_pose):
                self.state = State.DONE
                rospy.loginfo("Delivered treasure! All done now!")

        elif self.state is State.DONE:
            # We're done :D - don't do anything
            rospy.loginfo("Delivery was completed")
            pass

    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.perform_action()
                self.rate.sleep()
            except KeyboardInterrupt:
                if self.state is State.SEARCH_OBJECTIVES and self.mapping_is_complete():
                    rospy.loginfo("Stopping teleop keyboard")
                    self.keyboard_teleop_process.stop()
                    rospy.loginfo("Teleop keyboard stopped")
                elif self.state is State.SEARCH_OBJECTIVES:
                    feedback = []
                    if self.goalzone_pose is None: feedback.append("Goalzone not located")
                    if self.treasure_pose is None: feedback.append("Treasure not located")
                    rospy.logwarn("Can't quit mapping yet! %s", "and ".join(feedback))
                else:
                    rospy.loginfo("Quitting!")
                    break

    def mapping_is_complete(self):
        return self.treasure_pose is not None and self.goalzone_pose is not None

    def goto_pose(self, pose):
        
        # self.goalzone_pose
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        rospy.loginfo("Target pose: %s", str(pose))
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1) #a no-rotation quaternion
        client.send_goal(goal)

        waiting = client.wait_for_result()

        if not waiting:
            rospy.logerr("Action server is not available")
            # rospy.signal_shutdwon("Action server not available")
            return False
        else:
            rospy.loginfo("Treasure seeking result" + str(client.get_result()))
            return True

    def start_node(self, package, executable, args):
        node = roslaunch.core.Node(
            package=package, 
            node_type=executable,
            args=args)
        
        return self.launcher.launch(node)
    
    def ar_callback(self, msg):
        if not msg.markers:
            return

        marker_ids = [str(m.id) for m in msg.markers]
        rospy.logdebug("Markers found: %s" % ", ".join(marker_ids))

        for marker in msg.markers:
            if marker.id is 0:
                if self.treasure_pose is None:
                    rospy.loginfo("Located treasure")
                self.treasure_pose = marker.pose
            
            elif marker.id is 9:
                if self.goalzone_pose is None:
                    rospy.loginfo("Located goal zone")    
                self.goalzone_pose = marker.pose

    def visualize_info(self):
        if self.goalzone_pose is not None:
            rospy.logdebug("Visualizing GoalZone")
            marker = Marker(
                type=Marker.SPHERE,
                action=Marker.ADD,
                id=MarkerIds.GOAL_ZONE,
                lifetime=0,
                scale=Vector3(1, 1, 1),
                header=self.goalzone_pose.header,
                color=ColorRGBA(1.0, 0, 0),
                pose=self.goalzone_pose.pose,
                points=[self.goalzone_pose.pose.position],
                text="Goal Zone"
            )
            self.visualizations_pub.publish(marker)
        
        if self.treasure_pose is not None:
            rospy.logdebug("Visualizing Treasure")
            marker = Marker(
                type=Marker.SPHERE,
                action=Marker.ADD,
                id=MarkerIds.TREASURE,
                lifetime=0,
                scale=Vector3(1, 1, 1),
                header=self.Marker.header,
                color=ColorRGBA(0, 1.0, 0),
                pose=self.Marker.pose,
                points=[self.Marker.pose.position],
                text="Treasure Cube"
            )
            self.visualizations_pub.publish(marker)

def main():
    rospy.init_node("main_controller")
    controller = MainController()
    controller.spin()

if __name__ == "__main__":
    main()
