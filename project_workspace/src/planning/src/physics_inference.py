#!/usr/bin/env python

import copy 
import argparse
import rospy
import baxter_interface
import baxter_external_devices
# from baxter_interface import CHECK_VERSION, Limb, settings
import sys
import numpy as np
import traceback
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
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
GRAVITY_Z = -0.03

class PhysicsInference:

	def __init__(self):
		# rospy.init_node('physics_inference')
		self.pub = rospy.Publisher('/physics_inference', std_msgs.msg.String, queue_size=50)
		# TODO: subscribe to the tracker
		rospy.Subscriber("/vision/pc/kf_centroid", TwistStamped, self.tracking_listener)
		# TODO: subscribe to the begin tilt computation and return to neutral topics and create listeners for that

		# rospy.spin()

	def predict_collision_point(self, initial_position, initial_velocity, current_board_z):
		# print("Predicting collision point")
		delta_z = current_board_z - initial_position[2]
		sol_one_t = (-initial_velocity[2] + np.sqrt(initial_velocity[2] ** 2 + 2 * GRAVITY_Z * delta_z)) / GRAVITY_Z
		sol_two_t = (initial_velocity[2] + np.sqrt(initial_velocity[2] ** 2 + 2 * GRAVITY_Z * delta_z)) / GRAVITY_Z
		sol_two_t = -1 * sol_two_t

		time_to_collision = max(sol_one_t, sol_two_t)
		final_x = initial_position[0] + initial_velocity[0] * time_to_collision
		final_y = initial_position[1] + initial_velocity[1] * time_to_collision

		final_vel_x = initial_velocity[0]
		final_vel_y = initial_velocity[1]

		# final_vel_z = -1 * np.sqrt(initial_velocity[2] ** 2 + 2 * abs(GRAVITY_Z) * abs(delta_z))
		final_vel_z = -1 * np.sqrt(abs(initial_velocity[2] ** 2 + 2 * GRAVITY_Z * delta_z))

		return np.array([final_x, final_y, current_board_z]), np.array(
			[final_vel_x, final_vel_y, final_vel_z]), time_to_collision


	def solve_for_target_vel(self, contact_velocity, correction_distance, negative_target):
		# print("Solving target velocity")
		# negative target: determines if target velocity in the first dimension shoiuld point in negative or positive direction
		target_vel_mag = np.linalg.norm(contact_velocity)
		target_exit_angle = 0.5 * np.arcsin((-correction_distance * GRAVITY_Z) / (target_vel_mag ** 2))
		target_exit_angle = np.sign(target_exit_angle) * max((np.pi / 2) - abs(target_exit_angle), abs(target_exit_angle))
		target_vel_z = abs(target_vel_mag * np.sin(target_exit_angle))

		target_vel_hor_dim = target_vel_mag * np.cos(target_exit_angle)
		if negative_target:
			target_vel_hor_dim = -target_vel_hor_dim

		return np.array([target_vel_hor_dim, target_vel_z]), target_exit_angle

	def get_board_tilt_angle(self, contact_velocity, correction_distance):
		# print("Getting board tilt angle")
		# first dimension here will be x/y, 2nd dimension is always z
		negative_target = True if correction_distance > 0 else False ## used to be contact_velocity[0]
		target_vel, target_exit_angle = self.solve_for_target_vel(contact_velocity, correction_distance,
																  negative_target)

		velocity_mat = np.array(
			[[contact_velocity[0], contact_velocity[1]], [-1 * contact_velocity[1], contact_velocity[0]]])
		angle_vec = np.dot(np.linalg.inv(velocity_mat), np.array(target_vel))

		tilt_angle = 0.5 * np.arctan(angle_vec[1] / angle_vec[0])

		return tilt_angle


	def determine_tilt(self, initial_position, initial_velocity, current_board_z):
		# print("Determining tilt")
		col_point, col_vel, time_to_col = self.predict_collision_point(initial_position, initial_velocity,
																	   current_board_z)
		if abs(col_point[0] - BOARD_CENTER_X) > abs(col_point[1] - BOARD_CENTER_Y):
			twoD_velocity = np.array([col_vel[0], col_vel[2]])
			correction_delta = col_point[0] - BOARD_CENTER_X
			tilt_along_x = 1
			tilt_angle = -1 * self.get_board_tilt_angle(twoD_velocity, correction_delta)
			print("Tilt along x with angle ", tilt_angle)
		else:
			twoD_velocity = np.array([col_vel[1], col_vel[2]])
			correction_delta = col_point[1] - BOARD_CENTER_Y
			tilt_along_x = 0
			tilt_angle = -1 *self.get_board_tilt_angle(twoD_velocity, correction_delta)
			print("Tilt along y with angle ", tilt_angle)
		message = str(float(tilt_angle)) + "_" + str(tilt_along_x)
		self.pub.publish(message)

	
	def tracking_listener(self, msg):
		twist = msg.twist
		position = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
		velocity = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
		print("Current Position: ", position)
		print("Current Velocity: ", velocity)

		self.determine_tilt(position, velocity, BOARD_HEIGHT)


def main():
    rospy.init_node('physics_inference')
    process = PhysicsInference()
    rospy.spin()

if __name__ == '__main__':
    main()