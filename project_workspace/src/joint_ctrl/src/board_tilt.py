#!/usr/bin/env python

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

NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, NEUTRAL_Z_LEFT = 0.85, 0.3001, -0.04
NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, NEUTRAL_Z_RIGHT = 0.85, -0.2995, -0.044
BOARD_LEN_Y = 0.50
MOVE_RATE = 0.98

orien_const_left_vert = OrientationConstraint()
orien_const_left_vert.link_name = "left_gripper";
orien_const_left_vert.header.frame_id = "base";
orien_const_left_vert.orientation.y = -1.0;
orien_const_left_vert.absolute_x_axis_tolerance = 0.1;
orien_const_left_vert.absolute_y_axis_tolerance = 0.1;
orien_const_left_vert.absolute_z_axis_tolerance = 0.1;
orien_const_left_vert.weight = 1.0;

orien_const_right_vert = OrientationConstraint()
orien_const_right_vert.link_name = "right_gripper";
orien_const_right_vert.header.frame_id = "base";
orien_const_right_vert.orientation.y = -1.0;
orien_const_right_vert.absolute_x_axis_tolerance = 0.1;
orien_const_right_vert.absolute_y_axis_tolerance = 0.1;
orien_const_right_vert.absolute_z_axis_tolerance = 0.1;
orien_const_right_vert.weight = 1.0;

def move_to_joint_positions(limb, positions, timeout=15.0,
                            threshold=settings.JOINT_ANGLE_TOLERANCE,
                            test=None):
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
            cmd[joint] = (1 - MOVE_RATE) * positions[joint] + MOVE_RATE * cmd[joint]
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


def execute_movement(limb, joint_names, joint_angles):
    joint_command = {}
    for i in range(len(joint_names)):
        joint_command[joint_names[i]] = joint_angles[i]
        # limb.set_joint_positions(joint_command)
    move_to_joint_positions(limb, joint_command)
    # limb.set_joint_positions(joint_command)


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

            return plan.joint_trajectory.points[-1].positions
        except Exception as e:
            print e
            traceback.print_exc()

def return_to_neutral(left_arm, right_arm, lj, rj, left_planner, right_planner):
    print("finding left pos")
    left_joint_positions = find_joint_positions(left_planner, NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, NEUTRAL_Z_LEFT, orien_const=[orien_const_left_vert])
    print("finding right pos")
    right_joint_positions = find_joint_positions(right_planner, NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, NEUTRAL_Z_RIGHT, orien_const=[orien_const_right_vert])
    print("executing left mov")
    execute_movement(left_arm, lj, left_joint_positions)
    print("executing right mov")
    execute_movement(right_arm, rj, right_joint_positions)

def move_at_angle(left_arm, right_arm, lj, rj, left_planner, right_planner, target_angle):
	# target_angle: angle made by the board with the y axis: if positive, right arm moves up, left down. If negative, vice versa
	# only call this from neutral position
    print("in function")
    z_delta_magnitude = 0.5 * BOARD_LEN_Y * np.sin(target_angle)
    if target_angle > 0:
		right_target_z = NEUTRAL_Z_RIGHT + z_delta_magnitude
		left_target_z = NEUTRAL_Z_LEFT - z_delta_magnitude
    else:
		right_target_z = NEUTRAL_Z_RIGHT - z_delta_magnitude
		left_target_z = NEUTRAL_Z_LEFT + z_delta_magnitude
    print("finding left pos")
    left_target_joint_pos = find_joint_positions(left_planner, NEUTRAL_X_LEFT, NEUTRAL_Y_LEFT, left_target_z, orien_const=[orien_const_left_vert])
    print("finding right pos")
    right_target_joint_pos = find_joint_positions(right_planner, NEUTRAL_X_RIGHT, NEUTRAL_Y_RIGHT, right_target_z, orien_const=[orien_const_right_vert])
    print("executing left movement")
    execute_movement(left_arm, lj, left_target_joint_pos)
    print("executing right movement")
    execute_movement(right_arm, rj, right_target_joint_pos)

def simult_gripper_close(lg, rg):
    print(rg)
    rg.close()
    # lg.close()

def main():
    # rospy.spin()
    left = baxter_interface.Limb('left')
    lj = left.joint_names()
    left_gripper = robot_gripper.Gripper('left')

    right = baxter_interface.Limb('right')
    rj = right.joint_names()
    right_gripper = robot_gripper.Gripper('right')
    print(right_gripper.force())

    left.set_joint_position_speed(0.8)
    right.set_joint_position_speed(0.8)
    print("Initializing planner")
    planner_right = PathPlanner("right_arm")
    planner_left = PathPlanner("left_arm")
    print("Calling function")
    # move_at_angle(left, right, lj, rj, planner_left, planner_right, 0.4)
    return_to_neutral(left, right, lj, rj, planner_left, planner_right)

if __name__ == '__main__':
    rospy.init_node('boardtilt_node')

    main()

