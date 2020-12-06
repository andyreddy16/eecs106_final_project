#!/usr/bin/env python
"""
Path Planning Script for Lab 5
Author: Tiffany Cappellari
"""
import sys

from baxter_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import gripper as robot_gripper

# Uncomment this line for part 5 of Lab 5
# from controller import Controller


def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")


	# K values for Part 5
    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Borrowed from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Borrowed from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

	# Initialize the controller for Part 5
	# controller = Controller( . . . )

    #-----------------------------------------------------#
    ## Add any obstacles to the planning scene here
    #-----------------------------------------------------#
    size = np.array([0.4, 1.2, 0.1])
    pose = PoseStamped()
    pose.header.frame_id = "base"
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.z = 1.0
    # planner.add_box_obstacle(size, "box", pose)
    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    def move_to_goal(x, y, z, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
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

                # raw_input("Press <Enter> to move the right arm to goal pose: ")
                rospy.sleep(1)

                # Might have to edit this for part 5
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                else:
                    break
            except Exception as e:
                print e
                traceback.print_exc()
            # else:
            #     break


    while not rospy.is_shutdown():
        right_gripper = robot_gripper.Gripper('right')
        right_gripper.set_moving_force(80.0)
        right_gripper.set_holding_force(80.0)

        right_gripper.calibrate()
        # Set your goal positions here
        print("starting")
        # move_to_goal(0.85, -0.3001, 0.1)
        # rospy.sleep(1.)
        move_to_goal(0.85, -0.2995, 0.1)
        print("opening")
        right_gripper.open()
        rospy.sleep(1.)
        print("executing")
    	move_to_goal(0.85, -0.2995, -0.041)
        print("closings")
        right_gripper.close()
        print("MISSED: ", right_gripper.missed())
        print("FORCEEE: ", right_gripper.force())
        print("Done")
        # move_to_goal(0.4225 + 0.1, -0.1265, 0.7725 - 0.92)
        # right_gripper.close()
        # move_to_goal(0.4225 + 0.1 + 0.05, -0.1265, 0.7725 - 0.92)
        # right_gripper.open()
        raw_input("Press <Enter> to cycle through motion again")
        

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
