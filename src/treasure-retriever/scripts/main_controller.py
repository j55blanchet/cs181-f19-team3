#!/usr/bin/env python
import rospy

# from grid_map_msgs.msg import GridMap

def main():
    rospy.init_node("main_controller")
    rospy.spin()

if __name__ == "__main__":
    main()
