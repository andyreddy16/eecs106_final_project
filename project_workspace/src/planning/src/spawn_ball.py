#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import baxter_interface



def load_gazebo_models(ball_pose=Pose(position=Point(x=1.25, y=0.0, z=2.5)),
                       ball_reference_frame="world"):



    # Get path for the ball
    model_path = "/home/ryandehmoubed/ros_workspaces/eecs106_final_project/project_workspace/src/meshes/"

    # Load that good stuff
    table_xml = ''
    with open (model_path + "bouncy_ball_final/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
   
   
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("bouncy_ball_final", table_xml, "/",
                             ball_pose, ball_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
   
   

def main():

    rospy.init_node("better_load_shit_yo")

    load_gazebo_models()





if __name__ == '__main__':
    sys.exit(main())
