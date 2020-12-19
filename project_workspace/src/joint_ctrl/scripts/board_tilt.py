#!/usr/bin/env python

import argparse
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION, Limb
import sys
import numpy as np
import traceback
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from path_planner import PathPlanner
# from planning import path_planner
# from path_planner import PathPlanner
from baxter_interface import gripper as robot_gripper
from threading import Thread

NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, NEUTRAL_Z_LEFT = 0.85, 0.3001, -0.04
NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, NEUTRAL_Z_RIGHT = 0.85, -0.2995, -0.044
BOARD_LEN_Y = 0.50

orien_const_left_vert = OrientationConstraint()
orien_const_left_vert.link_name = "left_gripper";
orien_const_left_vert.header.frame_id = "base";
orien_const_left_vert.orientation.y = -1.0;
orien_const_left_vert.absolute_x_axis_tolerance = 0.1;
orien_const_left_vert.absolute_y_axis_tolerance = 0.1;
orien_const_left_vert.absolute_z_axis_tolerance = 0.1;
orien_const_left_vert.weight = 1.0;

orien_const_right_vert = OrientationConstraint()
orien_const_right_vert.link_name = "left_gripper";
orien_const_right_vert.header.frame_id = "base";
orien_const_right_vert.orientation.y = -1.0;
orien_const_right_vert.absolute_x_axis_tolerance = 0.1;
orien_const_right_vert.absolute_y_axis_tolerance = 0.1;
orien_const_right_vert.absolute_z_axis_tolerance = 0.1;
orien_const_right_vert.weight = 1.0;

def execute_movement(limb, joint_names, joint_angles):
    joint_command = {}
    for i in range(len(joint_names)):
        joint_command[joint_names[i]] = joint_angles[i]
        limb.set_joint_positions(joint_command)


def find_joint_positions(planner, x, y, z, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
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

            return planner.joint_trajectory.points[-1].positions
        except Exception as e:
            print e
            traceback.print_exc()

def return_to_neutral(left_arm, right_arm, lj, rj, left_planner, right_planner):
	left_joint_positions = find_joint_positions(left_planner, NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, NEUTRAL_Z_LEFT, orien_const=[orien_const_left_vert])
	right_joint_positions = fine_joint_positions(right_planner, NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, NEUTRAL_Z_RIGHT, orien_const=[orien_const_right_vert])
	execute_movement(left_arm, lj, left_joint_positions)
	execute_movement(right_arm, rj, right_joint_positions)

def get_target_positions(left_arm, right_arm, lj, rj, left_planner, right_planner, target_angle):
	# target_angle: angle made by the board with the y axis: if positive, right arm moves up, left down. If negative, vice versa
	# only call this from neutral position
	z_delta_magnitude = 0.5 * BOARD_LEN_Y * np.sin(target_angle)

	if target_angle > 0:
		right_target_z = NEUTRAL_Z_RIGHT + z_delta_magnitude
		left_target_z = NEUTRAL_Z_LEFT - z_delta_magnitude
	else:
		right_target_z = NEUTRAL_Z_RIGHT - z_delta_magnitude
		left_target_z = NEUTRAL_Z_LEFT + z_delta_magnitude

	left_target_joint_pos = find_joint_positions(left_planner, NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, left_target_z, orien_const=[orien_const_left_vert])
	right_target_joint_pos = find_joint_positions(right_planner, NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, right_target_z, orien_const=[orien_const_right_vert])
	execute_movement(left_arm, lj, left_target_joint_pos)
	execute_movement(right_arm, rj, right_target_joint_pos)

def simult_gripper_close(lg, rg):
	rg_close = lambda: rg.close()
	lg_close = lambda: lg.close()

	t1 = Thread(target=rg_close)
	t2 = Thread(target=lg_close)

	t1.start()
	t2.start()

	t1.join()
	t2.join()


def main():
    left = baxter_interface.Limb('left')
    lj = left.joint_names()
    left_gripper = robot_gripper.Gripper('left')

    right = baxter_interface.Limb('right')
    rj = right.joint_names()
    right_gripper = robot_gripper.Gripper('right')

    simult_gripper_close(left_gripper, right_gripper)

if __name__ == '__main__':
    rospy.init_node('boardtilt_node')

    main()

