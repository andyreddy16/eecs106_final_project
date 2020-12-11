import copy 
import argparse
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION, Limb, settings
import sys
import numpy as np
import traceback
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from path_planner import PathPlanner
# from planning import path_planner
# from path_planner import PathPlanner
from baxter_interface import gripper as robot_gripper
import baxter_dataflow
from threading import Thread
import std_msgs
from geometry_msgs.msg import PointStamped, TwistStamped

# POSTIVE Y: points to right of robot
# POSITIVE X: points away from robot

# constants. TODO: fill this in
BOARD_CENTER_X = 1.30
BOARD_CENTER_Y = 0.0
BOARD_HEIGHT = 0.95
GRAVITY_Z = -0.1

class PhysicsInference:

	def __init__(self):
		rospy.init_node('physics_inference')
		self.pub = rospy.Publisher('/physics_inference', std_msgs.msg.String, queue_size=50)
		# TODO: subscribe to the tracker
		rospy.Subscriber("/vision/pc/kf_centroid", TwistStamped, self.tracking_listener)
		# TODO: subscribe to the begin tilt computation and return to neutral topics and create listeners for that

		rospy.spin()

	def predict_collision_point(self, initial_position, initial_velocity, current_board_z):
		delta_z = current_board_z - initial_position[2]
		sol_one_t = (-initial_velocity[2] + np.sqrt(initial_velocity[2] ** 2 + 2 * GRAVITY_Z * delta_z)) / GRAVITY_Z
		sol_two_t = (initial_velocity[2] + np.sqrt(initial_velocity[2] ** 2 + 2 * GRAVITY_Z * delta_z)) / GRAVITY_Z
		sol_two_t = -1 * sol_two_t

		time_to_collision = max(sol_one_t, sol_two_t)
		final_x = initial_position[0] + initial_velocity[0] * time_to_collision
		final_y = initial_position[1] + initial_velocity[1] * time_to_collision

		final_vel_x = initial_velocity[0]
		final_vel_y = initial_velocity[1]

		final_vel_z = -1 * np.sqrt(initial_velocity[2]**2 + 2 * abs(GRAVITY_Z) * abs(delta_z))

		return np.array([final_x, final_y, current_board_z]), np.array([final_vel_x, final_vel_y, final_vel_z]), time_to_collision


	def solve_for_target_vel(self, contact_velocity, correction_distance, negative_target):
		# negative target: determines if target velocity in the first dimension shoiuld point in negative or positive direction
		target_vel_mag = np.linalg.norm(contact_velocity)
		target_exit_angle = 0.5 * np.arcsin((-correction_distance * GRAVITY_Z) / (target_vel_mag ** 2))
		target_exit_angle = max((np.pi / 2) - target_exit_angle, target_exit_angle)
		target_vel_z = target_vel_mag * np.sin(target_exit_angle)

		target_vel_hor_dim = target_vel_mag * np.cos(target_exit_angle)
		if negative_target:
			target_vel_hor_dim = -target_vel_hor_dim

		return np.array([target_vel_hor_dim, target_vel_z]), target_exit_angle


	def get_board_tilt_angle(self, contact_velocity, correction_distance):
		# first dimension here will be x/y, 2nd dimension is always z
		negative_target = True if contact_velocity[0] > 0 else False
		target_vel, target_exit_angle = self.solve_for_target_vel(contact_velocity, correction_distance,
																  negative_target)

		velocity_mat = np.array(
			[[contact_velocity[0], contact_velocity[1]], [-1 * contact_velocity[1], contact_velocity[0]]])
		angle_vec = np.dot(np.linalg.inv(velocity_mat), np.array(target_vel))

		tilt_angle = 0.5 * np.arctan(angle_vec[1] / angle_vec[0])

		return tilt_angle


	def determine_tilt(self, initial_position, initial_velocity, current_board_z):
		col_point, col_vel, time_to_col = self.predict_collision_point(initial_position, initial_velocity,
																	   current_board_z)
		if abs(col_point[0] - BOARD_CENTER_X) > abs(col_point[1] - BOARD_CENTER_Y):
			twoD_velocity = np.array([col_vel[0], col_vel[2]])
			correction_delta = col_point[0] - BOARD_CENTER_X
			tilt_along_x = 1
			tilt_angle = self.get_board_tilt_angle(twoD_velocity, correction_delta)
		else:
			twoD_velocity = np.array([col_vel[1], col_vel[2]])
			correction_delta = col_point[1] - BOARD_CENTER_Y
			tilt_along_x = 0
			tilt_angle = self.get_board_tilt_angle(twoD_velocity, correction_delta)

		self.pub.publish(std_msgs.msg.String(str(float(tilt_angle)) + "_" + str(tilt_along_x)))

	
	def tracking_listener(self, msg):
		# TODO: 
		twist = msg.twist 
		position = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
		velocity = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

		left_arm =  baxter_interface.Limb('left')
		right_arm = baxter_interface.Limb('right')
		curr_left_pos = self.left_arm.endpoint_pose()['position']
		curr_right_pos = self.right_arm.endpoint_pose()['position']
		left_x, left_y, left_z = curr_left_pos.x, curr_left_pos.y, curr_left_pos.z 
		right_x, right_y, right_z = curr_right_pos.x, curr_right_pos.y, curr_right_pos.z
		board_z = 0.5 * (right_z + left_z)

		self.determine_tilt(position, velocity, board_z)


