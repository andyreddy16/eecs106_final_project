# EECS 106A Final Project: Autonomous Ball Bouncing Robot

#### Contributors: Andy Reddy, Akash Velu, Michael Wan, Ryan Dehmoubed

## Packages
* Object Detection (obj_det): Contains files supporting the vision and tracking nodes/scripts of our project
  * kalman.py is responsible for filtering and tracking centroid outputs over time to get updated 3D positions and velocity components for the ball
  * segment_point_cloud.py is resposible for synchronized point cloud filtering and centroid computation for ball localization using Kinetic point cloud data and Kinetic depth images
  * segment_stereo.py is resposible for synchronized image processing and centroid computation for ball localization using Kinetic camera images and triangulation
  
* Path Planning (planning): 
  * path_planner.py
  * path_test_left.py
  * path_test_right.py
  * physics_inference.py
  * spawn_ball.py 
  
* Joint Control (joint_ctrl): 
  * board_tilt_class.py
  * gripper_close.py
  
## Additional Folders
* Gazebo world files (worlds): 
  * final_world_no_ball.world was the main world file we used for demonstrations

* Gazebo launch files (launch):
  * baxter_world.launch was the main launch file we used to load the world, configure the Baxter robot properly, and adjust initial environment variables for suitable testing

