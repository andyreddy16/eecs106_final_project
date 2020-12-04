#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from baxter_interface import gripper as robot_gripper


def map_user_input():
    left = baxter_interface.Limb('left')
    lj = left.joint_names()
    left_gripper = robot_gripper.Gripper('left')

    def set_j(limb, joint_names, joint_angles):
        joint_command = {}
        for i in range(len(joint_names)):
            joint_command[joint_names[i]] = joint_angles[i]
            limb.set_joint_velocities(joint_command)
        return joint_command

    done = False
    while not done and not rospy.is_shutdown():
        print(lj)

        user_joint_angles = raw_input("Enter 7 joint angles in comma-delimited format (e.g. 0.0,0.1,-1.2,1,1,0.1,-3.1): ")
        if user_joint_angles == "exit":
            done = True
            rospy.signal_shutdown("Example finished.")
        elif len(user_joint_angles.split(",")) != 7:
            print("7 joint angles were not passed in properly.")
            continue
        else:
            joint_angles = [float(angle) for angle in user_joint_angles.split(",")]
            # left_gripper.open()
            # rospy.sleep(0.5)
            # current_joint_angles = left.joint_angles()
            # for i in range(len(joint_angles)):
            #     joint_angles[i] += current_joint_angles[lj[i]]
            # print(joint_angles)

            commands = set_j(left, lj, joint_angles)

            left_gripper.close()
            print("New positions: ", commands)
        rospy.sleep(0.01)   


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """See help inside the example with the '?' key for key bindings."""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("joint_ctrl")

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()
    
    try:
        map_user_input()
    except rospy.ROSInterruptException:
        print("Done.")


if __name__ == '__main__':
    main()