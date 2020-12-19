#!/usr/bin/env python
import rospy
from baxter_interface import gripper as robot_gripper
#from intera_interface import gripper as robot_gripper

def main():
	rospy.init_node("gripper_close")
	right_gripper = robot_gripper.Gripper("right")
	left_gripper = robot_gripper.Gripper("left")
	left_gripper.calibrate()
	right_gripper.calibrate()
	rospy.sleep(2.0)

	right_gripper.close()
	rospy.sleep(0.1)
	left_gripper.close()

if __name__ == '__main__':
	main()