# How to run in Gazebo

#### Commands to run rosbot simulation in gazebo environment:
1. `roslaunch rosbot_gazebo rosbot_world.launch`
    * Launches world in gazebo, rosbot isn't there yet
2. `roslaunch rosbot_description rosbot_gazebo.launch`
    * Adds rosbot to gazebo simulation
3. `roslaunch rosbot_navigation gmapping_demo.launch`
    * Adds mapping node, generates occupancy grid based of laser scans
4. Run Mingi's node `wandering.py`
    * Makes robot go in straightline and turn when facing obstacles 
    * TODO: add code for detecting AR Tag & Goal Zone and locate them within occupancy grid 
      * At this point, mapping is complete. We'll have to run `rosrun map_server map_saver <save_location>`
5.  `roslaunch rosbot_navigation amcl_demo.launch <save_location>`
    * This node can perform path planning based on saved map. Computes a costmap and uses a (likely DWA) search algorithm.
    * This node subscribes to `goal_point` topic, allowing us to set destination,
    * This node publishes to `cmd_vel`, instructing robot to go forward.
    * Configurable costmap and other parameters based on `*.yaml`
    * gmapping node is still running, which continues to update map which is used for local planning.