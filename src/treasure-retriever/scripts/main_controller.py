#!/usr/bin/env python
import os
import rospy
import roslaunch
from geometry_msgs.msg import Twist
# from grid_map_msgs.msg import GridMap

class State:
    INIT = 0
    SEARCH_OBJECTIVES = 1
    SAVE_MAP = 2
    LOAD_MAP = 3
    FETCH_TREASURE = 4
    DELIVER_TREASURE = 5
    DONE = 6

class MainController:
    
    def __init__(self):
        self.state = State.INIT
        self.rate = rospy.Rate(30)
        self.action_seq = 0
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.init_time = rospy.Time.now()

        self.map_saver_process = None
        self.map_server_process = None
    
    def perform_action(self):
        self.action_seq += 1

        cmd_vel = Twist()

        if self.state is State.INIT:
            rospy.loginfo("Starting objective search")
            self.state = State.SEARCH_OBJECTIVES

        elif self.state is State.SEARCH_OBJECTIVES:

            if self.mapping_complete():
                rospy.loginfo("Mapping complete! Saving map")
                self.state = State.SAVE_MAP
                self.save_map()
                return
            
            # cmd_vel.linear.x = 0.1
            # cmd_vel.angular.z = 0.1

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
                self.state = State.FETCH_TREASURE

        elif self.state is State.FETCH_TREASURE:
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
        
        self.cmd_pub.publish(cmd_vel)

    def spin(self):
        while not rospy.is_shutdown():
            self.perform_action()
            self.rate.sleep()

    def mapping_complete(self):
        return rospy.Time.now().secs - self.init_time.secs > 10

    def save_map(self):
        rospy.loginfo("Saving map")

        map_save_file_path = os.environ['MAP_SAVE_FILE']
        package = "map_server"
        executable = "map_saver"
        node = roslaunch.core.Node(
            package=package, 
            node_type=executable,
            args="-f %s" % map_save_file_path)
    
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # rosrun map_server map_saver -f ~/test_map
        self.map_saver_process = launch.launch(node)

    def start_map_server(self):
        rospy.loginfo("Loading map")

        map_save_file_path = os.environ['MAP_SAVE_FILE']
        map_save_file_path += ".yaml"
        package = "map_server"
        executable = "map_server"
        node = roslaunch.core.Node(
            package=package, 
            node_type=executable,
            args=map_save_file_path)
    
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # rosrun map_server map_saver -f ~/test_map
        self.map_server_process = launch.launch(node)

def main():
    rospy.init_node("main_controller")
    controller = MainController()
    controller.spin()

if __name__ == "__main__":
    main()
