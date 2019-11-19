
# Treasure-Retriever

Team 3 | CS 181 | Fall 2019

## Setup & Operation

### Setup

1. `git submodule init`
1. `git submodule update`

1. Install packages (NOTE: THIS MIGHT NOT BE NECESSARY)
    * `sudo apt-get install -y ros-kinetic-ar-track-alvar ros-kinetic-depthimage-to-laserscan ros-kinetic-find-object-2d ros-kinetic-frontier-exploration ros-kinetic-gmapping ros-kinetic-grid-map`

1. If you plan on using Gazebo simulations, run: `rosdep install --from-paths src --ignore-src -r -y`

1. Run `catkin_make` in repo base

### Running
1. SSH into rosbot, run git pull to get latest code
1. Setup RVIZ connection so you can visualize robot from local machine
    * ROS_IP should be set to wired default of `192.168.0.1` - check `~/.bashrc` on the robot to ensure it's what you want.
    * On local machine you should set `ROS_IP` and `ROS_MASTER_URI` accordingly.
1. Run code
    * `roslaunch treasure-retriever treasure-retriever.launch` (launches service nodes)
    * `rosrun treasure-retriever main_controller.py` (launches main controller)
    * Launch rviz (we recommend using `docs/rviz_for_slam.rviz`)
1. Manually control robot during mapping period.
    * Standard controls for keyboard teleop
    * `i`, `j`, `k`, `l` control robot direction
    * Recommended first pressing `z` and `x` a few times to reduce movement speeds during this period.
    * You'll see in terminal (and rviz if you add `/visualization_marker` topic) when the treasure and goal zone have been detected.
1. When done mapping, with `main_controller` terminal in focus, hit `ctl-C` to make program move onto automatic delivery mode.

## System Architecture

Hardware Control Nodes
* `serial_bridge.sh`: for odometry
* `rpilidarNode`: operates lidar 

Third Party Components
* `gmapping`: performs SLAM: creates `/map` from lidar scans and publishes robot's pose
* `move_base`: performs path planning and path following using DWA algorithm.

Utility Nodes
* `static_transform_publisher`: used to publish transforms for lidar sensor and camera
* `pose_to_tf_transform`: publishes `base_link` frame with respect to `odom`
* `tf_to_pose`: used create pose from `map` to `base_link` for visualization purposes

Business Logic Nodes
* `main_controller.py`: contains FSM. Launches and stops keyboard teleop during manual mapping mode.

### Used Packages

Here's a list of packages we used in our project, along with a description of what each one was used for.

* Common Packages
  * `rospy`
  * `std_msgs`
  * `sensor_msgs`
  * `geometry_msgs`
  * `nav_msgs`
  * `tf`
* SLAM & Navigation
  * `gmapping`: computes occupancy grid from lidar sensor
  * `move_base`: used for path planning. Only need to provide it a target (goal) node.
* Visual Object Recognition
  * `ar_track_alvar`: allows for detection of AR Tags (used to recognized treasure cube)
  * `grid_map`: allows us to keep track of which obstacles were checked (robot will check obstacles detected on lidar to see if they're a goal zone or a treasure cube).
  * `actionlib`: allows us to send destination points to `move_base`
  
## Sources Consulted

* [Husarion ROS Tutorials](https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/)
  * SLAM navigation tutorial
  * Path planning tutorial
  * Unknown environment exploration tutorial
  * Object search tutorial
* ROS Documentation and ROS Answers