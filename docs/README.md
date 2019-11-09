
# Treasure-Retriever

Team 3 | CS 181 | Fall 2019

## Setup & Operation

### Setup

1. Install alvar package `sudo apt-get install ros-kinetic-ar-track-alvar`
    * for melodic, use `sudo apt-get install ros-melodic-ar-track-alvar`
2. Run `catkin_make` in repo base

## System Architecture

### Used Packages

Here's a list of packages we used in our project, along with a description of what each one was used for.

* Common Packages
  * `rospy`
  * `std_msgs`
  * `sensor_msgs`
  * `geometry_msgs`
  * `nav_msgs`
  * `diagnostic_msgs` (for reporting strength of wifi connection)
  * `tf`
* SLAM & Navigation
  * `gmapping`: computes occupancy grid from lidar sensor
  * `frontier_exploration`: generates navigational targets given target area to explore
  * `move_base`: used for path planning. Only need to provide it a target (goal) node.
* Visual Object Recognition
  * `ar_track_alvar`: allows for detection of AR Tags (used to recognized treasure cube)
  * `find_object_2d`: allows for recognition of objects using camera input (used to recognize goal zone)
  * `grid_map`: allows us to keep track of which obstacles were checked (robot will check obstacles detected on lidar to see if they're a goal zone or a treasure cube).
  * `depthimage_to_laserscan`: allows us to locate objects found via camera within our occupancy grid
* Utility Libraries
  * `actionlib`: assists with orchestration of robot commands by providing preemptable task framework
  * `image_transport`: compresses video stream, allowing us to view it on an external computer without consuming large amounts of bandwith
  
## Sources Consulted

* [Husarion ROS Tutorials](https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/)
  * SLAM navigation tutorial
  * Path planning tutorial
  * Unknown environment exploration tutorial
  * Object search tutorial

## TEMP

### Topic List from roswebui demo

* `/*battery`
* `/*camera/camera_nodelet_manager/bond`
* `/*camera/depth/camera_info`
* `/*camera/depth/image`
* `/*camera/depth/image/compressed`
* `/*camera/depth/image/compressed/parameter_descriptions`
* `/*camera/depth/image/compressed/parameter_updates`
* `/*camera/depth/image/compressedDepth`
* `/*camera/depth/image/compressedDepth/parameter_descriptions`
* `/*camera/depth/image/compressedDepth/parameter_updates`
* `/*camera/depth/image_raw`
* `/*camera/depth/image_raw/compressed`
* `/*camera/depth/image_raw/compressed/parameter_descriptions`
* `/*camera/depth/image_raw/compressed/parameter_updates`
* `/*camera/depth/image_raw/compressedDepth`
* `/*camera/depth/image_raw/compressedDepth/parameter_descriptions`
* `/*camera/depth/image_raw/compressedDepth/parameter_updates`
* `/*camera/depth/image_rect`
* `/*camera/depth/image_rect/compressed`
* `/*camera/depth/image_rect/compressed/parameter_descriptions`
* `/*camera/depth/image_rect/compressed/parameter_updates`
* `/*camera/depth/image_rect/compressedDepth`
* `/*camera/depth/image_rect/compressedDepth/parameter_descriptions`
* `/*camera/depth/image_rect/compressedDepth/parameter_updates`
* `/*camera/depth/image_rect_raw`
* `/*camera/depth/image_rect_raw/compressed`
* `/*camera/depth/image_rect_raw/compressed/parameter_descriptions`
* `/*camera/depth/image_rect_raw/compressed/parameter_updates`
* `/*camera/depth/image_rect_raw/compressedDepth`
* `/*camera/depth/image_rect_raw/compressedDepth/parameter_descriptions`
* `/*camera/depth/image_rect_raw/compressedDepth/parameter_updates`
* `/*camera/depth/points`
* `/*camera/depth_rectify_depth/parameter_descriptions`
* `/*camera/depth_rectify_depth/parameter_updates`
* `/*camera/depth_registered/camera_info`
* `/*camera/depth_registered/image_raw`
* `/*camera/depth_registered/image_raw/compressed`
* `/*camera/depth_registered/image_raw/compressed/parameter_descriptions`
* `/*camera/depth_registered/image_raw/compressed/parameter_updates`
* `/*camera/depth_registered/image_raw/compressedDepth`
* `/*camera/depth_registered/image_raw/compressedDepth/parameter_descriptions`
* `/*camera/depth_registered/image_raw/compressedDepth/parameter_updates`
* `/*camera/depth_registered/points`
* `/*camera/depth_registered/sw_registered/camera_info`
* `/*camera/depth_registered/sw_registered/image_rect`
* `/*camera/depth_registered/sw_registered/image_rect/compressed`
* `/*camera/depth_registered/sw_registered/image_rect/compressed/parameter_descriptions`
* `/*camera/depth_registered/sw_registered/image_rect/compressed/parameter_updates`
* `/*camera/depth_registered/sw_registered/image_rect/compressedDepth`
* `/*camera/depth_registered/sw_registered/image_rect/compressedDepth/parameter_descriptions`
* `/*camera/depth_registered/sw_registered/image_rect/compressedDepth/parameter_updates`
* `/*camera/depth_registered/sw_registered/image_rect_raw`
* `/*camera/depth_registered/sw_registered/image_rect_raw/compressed`
* `/*camera/depth_registered/sw_registered/image_rect_raw/compressed/parameter_descriptions`
* `/*camera/depth_registered/sw_registered/image_rect_raw/compressed/parameter_updates`
* `/*camera/depth_registered/sw_registered/image_rect_raw/compressedDepth`
* `/*camera/depth_registered/sw_registered/image_rect_raw/compressedDepth/parameter_descriptions`
* `/*camera/depth_registered/sw_registered/image_rect_raw/compressedDepth/parameter_updates`
* `/*camera/driver/parameter_descriptions`
* `/*camera/driver/parameter_updates`
* `/*camera/ir/camera_info`
* `/*camera/ir/image`
* `/*camera/ir/image/compressed`
* `/*camera/ir/image/compressed/parameter_descriptions`
* `/*camera/ir/image/compressed/parameter_updates`
* `/*camera/ir/image/compressedDepth`
* `/*camera/ir/image/compressedDepth/parameter_descriptions`
* `/*camera/ir/image/compressedDepth/parameter_updates`
* `/*camera/projector/camera_info`
* `/*camera/rgb/camera_info`
* `/*camera/rgb/image_raw`
* `/*camera/rgb/image_raw/compressed`
* `/*camera/rgb/image_raw/compressed/parameter_descriptions`
* `/*camera/rgb/image_raw/compressed/parameter_updates`
* `/*camera/rgb/image_raw/compressedDepth`
* `/*camera/rgb/image_raw/compressedDepth/parameter_descriptions`
* `/*camera/rgb/image_raw/compressedDepth/parameter_updates`
* `/*camera/rgb/image_rect_color`
* `/*camera/rgb/image_rect_color/compressed`
* `/*camera/rgb/image_rect_color/compressed/parameter_descriptions`
* `/*camera/rgb/image_rect_color/compressed/parameter_updates`
* `/*camera/rgb/image_rect_color/compressedDepth`
* `/*camera/rgb/image_rect_color/compressedDepth/parameter_descriptions`
* `/*camera/rgb/image_rect_color/compressedDepth/parameter_updates`
* `/*camera/rgb_rectify_color/parameter_descriptions`
* `/*camera/rgb_rectify_color/parameter_updates`
* `/*clicked_point`
* `/*client_count`
* `/*clipping/distance`
* `/*clipping/output`
* `/*cmd_vel`
* `/*connected_clients`
* `/*diagnostics`
* `/*exploration_polygon_marker`
* `/*explore_server/cancel`
* `/*explore_server/explore_costmap/costmap`
* `/*explore_server/explore_costmap/costmap_updates`
* `/*explore_server/explore_costmap/explore_boundary/frontiers`
* `/*explore_server/explore_costmap/explore_boundary/parameter_descriptions`
* `/*explore_server/explore_costmap/explore_boundary/parameter_updates`
* `/*explore_server/explore_costmap/footprint`
* `/*explore_server/explore_costmap/inflation/parameter_descriptions`
* `/*explore_server/explore_costmap/inflation/parameter_updates`
* `/*explore_server/explore_costmap/parameter_descriptions`
* `/*explore_server/explore_costmap/parameter_updates`
* `/*explore_server/explore_costmap/static/parameter_descriptions`
* `/*explore_server/explore_costmap/static/parameter_updates`
* `/*explore_server/feedback`
* `/*explore_server/goal`
* `/*explore_server/result`
* `/*explore_server/status`
* `/*gmapping_node/entropy`
* `/*initialpose`
* `/*joint_states`
* `/*map`
* `/*map_image/tile`
* `/*map_image/tile/compressed`
* `/*map_image/tile/compressed/parameter_descriptions`
* `/*map_image/tile/compressed/parameter_updates`
* `/*map_image/tile/compressedDepth`
* `/*map_image/tile/compressedDepth/parameter_descriptions`
* `/*map_image/tile/compressedDepth/parameter_updates`
* `/*map_metadata`
* `/*map_updates`
* `/*map_zoom`
* `/*move_base/NavfnROS/plan`
* `/*move_base/TrajectoryPlannerROS/cost_cloud`
* `/*move_base/TrajectoryPlannerROS/global_plan`
* `/*move_base/TrajectoryPlannerROS/local_plan`
* `/*move_base/TrajectoryPlannerROS/parameter_descriptions`
* `/*move_base/TrajectoryPlannerROS/parameter_updates`
* `/*move_base/cancel`
* `/*move_base/current_goal`
* `/*move_base/feedback`
* `/*move_base/global_costmap/costmap`
* `/*move_base/global_costmap/costmap_updates`
* `/*move_base/global_costmap/footprint`
* `/*move_base/global_costmap/inflation_layer/parameter_descriptions`
* `/*move_base/global_costmap/inflation_layer/parameter_updates`
* `/*move_base/global_costmap/obstacle_layer/parameter_descriptions`
* `/*move_base/global_costmap/obstacle_layer/parameter_updates`
* `/*move_base/local_costmap/inflation_layer/parameter_updates`
* `/*move_base/local_costmap/obstacle_layer/parameter_descriptions`
* `/*move_base/local_costmap/obstacle_layer/parameter_updates`
* `/*move_base/local_costmap/parameter_descriptions`
* `/*move_base/local_costmap/parameter_updates`
* `/*move_base/parameter_descriptions`
* `/*move_base/parameter_updates`
* `/*move_base/result`
* `/*move_base/status`
* `/*move_base_simple/goal`
* `/*odom`
* `/*pose`
* `/*range/fl`
* `/*range/fr`
* `/*range/rl`
* `/*range/rr`
* `/*reset_map`
* `/*reset_odom`
* `/*rosbot_on_map_pose`
* `/*rosout`
* `/*rosout_agg`
* `/*rpy`
* `/*scan`
* `/*tf`
* `/*tf_static`
* `/*wifi_status`