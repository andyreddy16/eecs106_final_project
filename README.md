# EECS 106A Final Project: Autonomous Ball Bouncing Robot

#### Contributors: Andy Reddy, Akash Velu, Michael Wan, Ryan Dehmoubed

## Packages
* Object Detection (obj_det): Contains files supporting the vision and tracking nodes/scripts of our project
  * kalman.py is responsible for filtering and tracking centroid outputs over time to get updated 3D positions and velocity components for the ball
  * segment_point_cloud.py is resposible for synchronized point cloud filtering and centroid computation for ball localization using Kinetic point cloud data and Kinetic depth images
  * segment_stereo.py is resposible for synchronized image processing and centroid computation for ball localization using Kinetic camera images and triangulation
  
* Path Planning (planning): Contains files support the general path planning (limb movement with specified targets + board tilt mathematic logic), and actuation support for the board grasping portion.
  * path_planner.py is responsible for initializing and configuring Move It's planner algorithm for faster and more accurate path planning of Baxter limbs.
  * path_test_left.py is responsible for the left arm motion in grabbing the board.
  * path_test_right.py is responsible for the right arm motion in grabbing the board.
  * physics_inference.py is responsible for calculating the required board tilt to control the next ball location to target near the center of the board for stabler bouncing.
  * spawn_ball.py is resposible for spawning our custom ball in Gazebo with any given initial pose and velocity.
  
* Joint Control (joint_ctrl): Contains files supporting the actuation logic for the board tilt 
  * board_tilt_class.py is responsible for the actuation of the board tilt commands sent from the physics inference node in the planning package.
  * gripper_close.py is responsible for actuating the grippers to grab the board after the arms have moved to the proper locations via path_test_left and path_test_right in the planning package.
  
## Additional Folders
* Gazebo world files (worlds)
  * final_world_no_ball.world was the main world file we used for demonstrations

* Gazebo launch files (launch)
  * baxter_world.launch was the main launch file we used to load the world, configure the Baxter robot properly, and adjust initial environment variables for suitable testing

