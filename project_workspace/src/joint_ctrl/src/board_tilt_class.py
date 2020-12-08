#!/usr/bin/env python

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

# define constants
# grasp positions of end effectors
NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, NEUTRAL_Z_LEFT = 0.85, 0.3002, -0.033
NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, NEUTRAL_Z_RIGHT = 0.85, -0.2995, -0.033

# joint position of end effectors during initial grasp
NEUTRAL_JOINTS_LEFT = {'left_w0': 0.689529664615443, 'left_w1': 1.4659524973800782, 'left_w2': -0.12685787673721194, 'left_e0': -0.757109078019897, 'left_e1': 0.6733622553430516, 'left_s0': -0.4675242116126368, 'left_s1': -0.40205941585160954}
NEUTRAL_JOINTS_RIGHT = {'right_s0': 0.45687498174440755, 'right_s1': -0.3931044727643451, 'right_w0': -0.7215030244174878, 'right_w1': 1.4716411980595003, 'right_w2': 0.13357577080887495, 'right_e0': 0.791219346992718, 'right_e1': 0.6734590902341049}

# length of board in Y axis
BOARD_LEN_Y = 0.50
# index of wrist to control for tilt
W1_IND = 5
# increment size for tilt movement
X_TILT_STEP_SIZE = 0.025

class TiltController:
	def __init__(self):
		self.left_arm = baxter_interface.Limb('left')
		self.lj = self.left_arm.joint_names()
		self.left_gripper = robot_gripper.Gripper('left')

		self.right_arm = baxter_interface.Limb('right')
		self.rj = self.right_arm.joint_names()
		self.right_gripper = robot_gripper.Gripper('right')

		self.planner_left = PathPlanner('left_arm')
		self.planner_right = PathPlanner('right_arm')

		self.orien_const_left_vert = OrientationConstraint()
		self.orien_const_left_vert.link_name = "left_gripper"
		self.orien_const_left_vert.header.frame_id = "base"
		self.orien_const_left_vert.orientation.y = -1.0
		self.orien_const_left_vert.absolute_x_axis_tolerance = 0.1
		self.orien_const_left_vert.absolute_y_axis_tolerance = 0.1
		self.orien_const_left_vert.absolute_z_axis_tolerance = 0.1
		self.orien_const_left_vert.weight = 1.0

		self.orien_const_right_vert = copy.deepcopy(self.orien_const_left_vert)
		self.orien_const_right_vert.link_name = "right_gripper"

	def move_to_joint_positions(self, limb, positions, timeout=15.0, move_rate=0.98, threshold=settings.JOINT_ANGLE_TOLERANCE, test=None):
		"""
		(Blocking) Commands the limb to the provided positions.
	    Waits until the reported joint state matches that specified.
	    This function uses a low-pass filter to smooth the movement.
	    @type positions: dict({str:float})
	    @param positions: joint_name:angle command
	    @type timeout: float
	    @param timeout: seconds to wait for move to finish [15]
	    @type threshold: float
	    @param threshold: position threshold in radians across each joint when
	    move is considered successful [0.008726646]
	    @param test: optional function returning True if motion must be aborted
	    """
		cmd = limb.joint_angles()

		def filtered_cmd():
		    # First Order Filter - 0.2 Hz Cutoff
		    for joint in positions.keys():
		        # cmd[joint] = 0.012488 * positions[joint] + 0.98751 * cmd[joint]
		        cmd[joint] = (1 - move_rate) * positions[joint] + move_rate * cmd[joint]
		    return cmd

		def genf(joint, angle):
		    def joint_diff():
		        return abs(angle - limb._joint_angle[joint])
		    return joint_diff

		diffs = [genf(j, a) for j, a in positions.items() if
		         j in limb._joint_angle]


		limb.set_joint_positions(filtered_cmd())
		baxter_dataflow.wait_for(
		    test=lambda: callable(test) and test() == True or \
		                 (all(diff() < threshold for diff in diffs)),
		    timeout=timeout,
		    timeout_msg=("%s limb failed to reach commanded joint positions" %
		                 (limb.name.capitalize(),)),
		    rate=100,
		    raise_on_error=False,
		    body=lambda: limb.set_joint_positions(filtered_cmd())
		    )

	def execute_movement(self, limb, joint_names, joint_angles, move_rate=0.99):
	    joint_command = {}
	    for i in range(len(joint_names)):
	        if type(joint_angles) == dict:
	            joint_command[joint_names[i]] = joint_angles[joint_names[i]]
	        else:
	            joint_command[joint_names[i]] = joint_angles[i]
	        # limb.set_joint_positions(joint_command)
	    self.move_to_joint_positions(limb, joint_command, move_rate=move_rate)
	    # limb.set_joint_positions(joint_command)

	def find_joint_positions(self, planner, x, y, z, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
	    while not rospy.is_shutdown():
	        try:
	            goal = PoseStamped()
	            goal.header.frame_id = "base"

	            #x, y, and z position
	            goal.pose.position.x = x
	            goal.pose.position.y = y
	            goal.pose.position.z = z

		    #Orientation as a quaternion
	            goal.pose.orientation.x = or_x
	            goal.pose.orientation.y = or_y
	            goal.pose.orientation.z = or_z
	            goal.pose.orientation.w = or_w

	            plan = planner.plan_to_pose(goal, orien_const)

	            return plan.joint_trajectory.points[-1].positions
	        except Exception as e:
	            print e
	            traceback.print_exc()

	def return_to_neutral(self):
		print("Moving left arm to neutral pose: ")
		self.execute_movement(self.left_arm, self.lj, NEUTRAL_JOINTS_LEFT)
		print("Moving right arm to neutral pose: ")
		self.execute_movement(self.right_arm, self.rj, NEUTRAL_JOINTS_RIGHT)

	def tilt_along_y(self, target_angle):
		curr_left_pos = self.left_arm.endpoint_pose()['position']
		curr_right_pos = self.right_arm.endpoint_pose()['position']
		left_x, left_y, left_z = curr_left_pos.x, curr_left_pos.y, curr_left_pos.z 
		right_x, right_y, right_z = curr_right_pos.x, curr_right_pos.y, curr_right_pos.z

		z_delta_magnitude = 0.5 * BOARD_LEN_Y * np.sin(target_angle)
		if target_angle > 0:
			right_target_z = right_z + z_delta_magnitude
			left_target_z = left_z - z_delta_magnitude
		else:
			right_target_z = right_z - z_delta_magnitude
			left_target_z = left_z + z_delta_magnitude

		print("Finding left position")
		left_target_joint_pos = self.find_joint_positions(self.planner_left, left_x, left_y, left_target_z, orien_const=[self.orien_const_left_vert])
		print("Finding right position")
		right_target_joint_pos = self.find_joint_positions(self.planner_right, right_x, right_y, right_target_z, orien_const=[self.orien_const_right_vert])

		print("Moving left arm")
		self.execute_movement(self.left_arm, self.lj, left_target_joint_pos)
		print("Moving right arm")
		self.execute_movement(self.right_arm, self.rj, right_target_joint_pos)


	def tilt_along_x(self, target_angle_delta):
		print("Tilting arms simultaneously along x")
		left_curr = self.left_arm.joint_angles()
		right_curr = self.right_arm.joint_angles()

		left_delta = left_curr['left_w1'] - (NEUTRAL_JOINTS_LEFT['left_w1'] + target_angle_delta)
		right_delta = right_curr['right_w1'] - (NEUTRAL_JOINTS_RIGHT['right_w1'] + target_angle_delta)
		left_sgn = 1 if left_delta > 0 else -1
		right_sgn = 1 if right_delta > 0 else -1

		iter_count = int(min(abs(right_delta) // X_TILT_STEP_SIZE, abs(left_delta) // X_TILT_STEP_SIZE))
		left_rem, right_rem = left_delta % X_TILT_STEP_SIZE, right_delta % X_TILT_STEP_SIZE

		for i in range(iter_count):
		    left_curr['left_w1'] += left_sgn * X_TILT_STEP_SIZE
		    right_curr['right_w1'] += right_sgn * X_TILT_STEP_SIZE
		    self.left_arm.set_joint_positions(left_curr)
		    self.right_arm.set_joint_positions(right_curr)
		    rospy.sleep(0.1)

		left_curr['left_w1'] += left_sgn * left_rem
		right_curr['right_w1'] += right_sgn * right_rem
		self.left_arm.set_joint_positions(left_curr)
		self.right_arm.set_joint_positions(right_curr)


 	def close_grippers_simult(self):
	 	self.rg.close()
	 	self.lg.close()




def main():
	tilt_controller = TiltController()
	# tilt_controller.tilt_along_y(0.2)
	tilt_controller.tilt_along_x(-0.3)
	rospy.sleep(0.8)
	tilt_controller.return_to_neutral()

if __name__ == '__main__':
	rospy.init_node('BoardTiltingNode')
	main()










