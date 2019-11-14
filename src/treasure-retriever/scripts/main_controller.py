#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
# from grid_map_msgs.msg import GridMap

class State:
    INIT = 0
    SEARCH_OBJECTIVES = 1
    FETCH_TREASURE = 2
    DELIVER_TREASURE = 3
    DONE = 4

class MainController:
    
    def __init__(self):
        self.state = State.INIT
        self.rate = rospy.Rate(30)
        self.action_seq = 0
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.init_time = rospy.Time.now()
    
    def perform_action(self):

        rospy.loginfo("Action % 6d State %d" % (self.action_seq, self.state))
        self.action_seq += 1

        cmd_vel = Twist()

        if self.state is State.INIT:
            self.state = State.SEARCH_OBJECTIVES

        elif self.state is State.SEARCH_OBJECTIVES:

            if self.mapping_complete():
                self.state = State.FETCH_TREASURE
                return
            
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.3
            

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
        # rosrun map_server map_saver -f ~/test_map
        pass



def main():
    rospy.init_node("main_controller")
    controller = MainController()
    controller.spin()

if __name__ == "__main__":
    main()
