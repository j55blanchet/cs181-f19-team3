#!/usr/bin/env python
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
    SAVE_MAP = 2
    LOAD_MAP = 3
    FETCH_TREASURE = 4
    DELIVER_TREASURE = 5
    DONE = 6

class MarkerIds:
    GOAL_ZONE = 0
    TREASURE  = 1

class MainController:
    
    def __init__(self):
        self.state = State.INIT
        self.rate = rospy.Rate(10)
        self.action_seq = 0
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.ar_tag_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
        self.visualizations_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
        self.init_time = rospy.Time.now()

        self.map_saver_process = None
        self.map_server_process = None
        self.keyboard_teleop_process = None

        self.goalzone_pose = None
        self.treasure_pose = None
    
    def perform_action(self):
        self.action_seq += 1

        rospy.loginfo_throttle(1, "[%04d] State: %s" % (self.action_seq, str(self.state)))

        if self.state is State.INIT:
            rospy.loginfo("Starting objective search")
            self.keyboard_teleop_process = self.start_node("teleop_twist_keyboard", "teleop_twist_keyboard.py", "")
            self.state = State.SEARCH_OBJECTIVES

        elif self.state is State.SEARCH_OBJECTIVES:

            if self.mapping_is_complete():
                rospy.loginfo("Mapping complete! Stopping teleop keyboard")

                self.keyboard_teleop_process.stop()

                # stop robot at current position
                self.cmd_pub.publish(Twist())

                rospy.loginfo("Saving map")
                self.state = State.SAVE_MAP
                self.save_map()
                return
            
        elif self.state is State.SAVE_MAP:

            if self.map_saver_process is not None and not self.map_saver_process.is_alive():
                
                # TODO: Find a better way to detect when the map is complete
                rospy.loginfo("Map saving complete")
                self.start_map_server()
                self.state = State.LOAD_MAP
        
        elif self.state is State.LOAD_MAP:
            # TODO: detect when map is loaded and switch to FETCH_TREASURE
            if self.map_server_process is not None and self.map_server_process.is_alive():
                # Switch to FETCH_TREASURE
                rospy.loginfo("Map loading complete")
                self.state = State.FETCH_TREASURE

        elif self.state is State.FETCH_TREASURE:
            self.goto_treasure()
            # TODO:
            #    1) Determine target pose (far side of treasure pointing to goal zone)
            #    2) Get path to target pose
            #    3) Forward command from move_base
            pass
        
        elif self.state is State.DELIVER_TREASURE:
            # TODO: Use move_base with target point as goal zone
            pass

        elif self.state is State.DONE:
            # We're done :D - don't do anything
            pass

    def spin(self):
        while not rospy.is_shutdown():
            self.perform_action()
            self.rate.sleep()

    def mapping_is_complete(self):
        return self.treasure_pose is not None and self.goalzone_pose is not None

    def goto_treasure(self):
        
        # self.goalzone_pose
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        rospy.loginfo("Treasure pose: %s", str(self.treasure_pose))
        goal.target_pose.header.frame_id = "map"
        # rospy.loginfo("Source frame_id = %s", str(self.treasure_pose.header.frame_id))
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = self.treasure_pose.pose.position
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1) #a no-rotation quaternion
        client.send_goal(goal)

        waiting = client.wait_for_result()

        if not waiting:
            rospy.logerr("Action server is not available")
            rospy.signal_shutdwon("Action server not available")
        else:
            rospy.loginfo("Treasure seeking result" + str(client.get_result()))

    def start_node(self, package, executable, args):
        node = roslaunch.core.Node(
            package=package, 
            node_type=executable,
            args=args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        return launch.launch(node)

    def save_map(self):
        rospy.loginfo("Saving map")

        map_save_file_path = os.environ['MAP_SAVE_FILE']

        # rosrun map_server map_saver -f ~/test_map
        self.map_saver_process = self.start_node("map_server", "map_saver", "-f %s" % map_save_file_path)

    def start_map_server(self):
        rospy.loginfo("Loading map")

        map_save_file_path = os.environ['MAP_SAVE_FILE']
        map_save_file_path += ".yaml"

        # rosrun map_server map_saver -f ~/test_map
        self.map_server_process = self.start_node("map_server", "map_server", map_save_file_path)
    
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
            print "Visualizing GoalZone"
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
            print "Visualizing Treasure"
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
        #     PoseStamped()

def main():
    rospy.init_node("main_controller")
    controller = MainController()
    controller.spin()

if __name__ == "__main__":
    main()
