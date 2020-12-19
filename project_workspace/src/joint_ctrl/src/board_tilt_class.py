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
import std_msgs

# define constants
# grasp positions of end effectors

# name: [head_pan, l_gripper_l_finger_joint, l_gripper_r_finger_joint, left_e0, left_e1, left_s0,
#   left_s1, left_w0, left_w1, left_w2, r_gripper_l_finger_joint, r_gripper_r_finger_joint,
#   right_e0, right_e1, right_s0, right_s1, right_w0, right_w1, right_w2]
# position: [3.162681415691537e-05, 0.011747230037799388, -0.011301894348559022, -0.3711648285163207, 0.6447759727041227, -0.5983051345409285, -0.2853998975032006, 0.3751089018312417, 1.2529409061087229, -0.04519162051499759, 0.011184207897133973, -0.0116188520886955, 0.007903188131067829, 0.6565351573119829, 0.730617402345259, -0.3247707732777254, -0.0077135720534320384, 1.2393440664209763, -0.049436686954040354]


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
X_TILT_STEP_SIZE = 0.015

# Clip value for tilting
UPPER_TILT_THRESH = 0.12
LOWER_TILT_THRESH = 0.02

class TiltController:
	def __init__(self):
		print("Starting init")
		self.left_arm = baxter_interface.Limb('left')
		print("Left arm obtrained")
		self.lj = self.left_arm.joint_names()
		self.left_gripper = robot_gripper.Gripper('left')
		self.left_joint_neutral = self.left_arm.joint_angles()

		self.right_arm = baxter_interface.Limb('right')
		self.rj = self.right_arm.joint_names()
		self.right_gripper = robot_gripper.Gripper('right')
		self.right_joint_neutral = self.right_arm.joint_angles()

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

		self.previous_deviation = (None, None)
		self.setting_to_neutral = False
		self.tilt_after_neutral = True

		self.pub = rospy.Publisher("/board_controller_log", std_msgs.msg.String, queue_size=10)
		rospy.Subscriber("/physics_inference", std_msgs.msg.String, self.perform_tilt)
		rospy.Subscriber("/control/neutral_set", std_msgs.msg.Bool, self.neutral_listener)
		rospy.Subscriber("control/tilt_set", std_msgs.msg.Bool, self.tilt_listener)

	def move_to_joint_positions(self, limb, positions, timeout=15.0, move_rate=0.982, threshold=settings.JOINT_ANGLE_TOLERANCE, test=None):
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
		self.setting_to_neutral = True 
		if self.previous_deviation[0] == 'y_tilt' or self.previous_deviation[0] == None:
			print("Moving right arm to neutral pose: ")
			self.execute_movement(self.right_arm, self.rj, self.right_joint_neutral)
			print("Moving left arm to neutral pose: ")
			rospy.sleep(0.1)
			self.execute_movement(self.left_arm, self.lj, self.left_joint_neutral)
		

		else:
			self.tilt_along_x(0.0)

		self.setting_to_neutral = False
		self.tilt_after_neutral = False

	def tilt_along_y(self, target_angle):
		# Positive angle = board faces +y dir, negative angle = board faces -y dir
		curr_left_pos = self.left_arm.endpoint_pose()['position']
		curr_right_pos = self.right_arm.endpoint_pose()['position']
		left_x, left_y, left_z = curr_left_pos.x, curr_left_pos.y, curr_left_pos.z 
		right_x, right_y, right_z = curr_right_pos.x, curr_right_pos.y, curr_right_pos.z

		z_delta_magnitude = abs(0.5 * BOARD_LEN_Y * np.sin(target_angle))
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

		print("Moving right arm")
		self.execute_movement(self.right_arm, self.rj, right_target_joint_pos)
		rospy.sleep(0.1)
		print("Moving left arm")
		self.execute_movement(self.left_arm, self.lj, left_target_joint_pos)

		self.previous_deviation = ("y_tilt", None)


	def tilt_along_x(self, target_angle_delta):
		# Positive angle = towards robot, negative angle = away from robot
		print("Tilting arms simultaneously along x")
		left_curr = self.left_arm.joint_angles()
		right_curr = self.right_arm.joint_angles()

		left_delta = (self.left_joint_neutral['left_w1'] + target_angle_delta) - left_curr['left_w1']
		right_delta = (self.right_joint_neutral['right_w1'] + target_angle_delta) - right_curr['right_w1']
		left_sgn = 1 if left_delta > 0 else -1
		right_sgn = 1 if right_delta > 0 else -1

		iter_count = int(min(abs(right_delta) // X_TILT_STEP_SIZE, abs(left_delta) // X_TILT_STEP_SIZE))
		print("ITER COUNT: ", iter_count)
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

		self.previous_deviation = ("x_tilt", target_angle_delta)


 	def neutral_listener(self, msg):
 		print("Returning to neutral")
 		self.return_to_neutral()
 		self.pub.publish("Return to neutral message processed.")


 	def tilt_listener(self, msg):
 		print("Setting the tilt")
 		self.tilt_after_neutral = True
 		self.pub.publish("Set to tilt message processed.")


	def perform_tilt(self, msg):
		print("Tilt performing: ", msg)
		if not self.setting_to_neutral and self.tilt_after_neutral: 
			angle, x_tilt = msg.data.split("_")
			angle, x_tilt = float(angle), int(x_tilt)
			print("Actual tilt execution: ", msg)
			angle = np.sign(angle) * min(abs(angle), UPPER_TILT_THRESH)

			if abs(angle) <= LOWER_TILT_THRESH:
				self.tilt_after_neutral = False 
			elif x_tilt:
				self.tilt_along_x(angle)
			else:
				self.tilt_along_y(angle)
			self.tilt_after_neutral = False
			self.pub.publish("Board tilt actuation message processed.")


	def calibrate_demo(self):
		# Tilt +y first
		self.tilt_along_y(0.2)
		rospy.sleep(1.)

		# Tilt -y first
		self.tilt_along_y(-0.4)
		rospy.sleep(1.)

		# Return to neutral
		self.return_to_neutral()
		rospy.sleep(1.)

		# Tilt +x first
		self.tilt_along_x(0.2)
		rospy.sleep(1.)

		# Tilt -x first
		self.tilt_along_x(-0.2)
		rospy.sleep(1.)

		# Return to neutral
		self.return_to_neutral()
		rospy.sleep(1.)

		print("Finish calibration")


def main():
    rospy.init_node('tilt_controller')
    process = TiltController()
    # process.calibrate_demo()
    rospy.spin()

if __name__ == '__main__':
    main()











