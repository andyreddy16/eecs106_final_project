#!/usr/bin/env python

import rospy
import std_msgs
import numpy as np
import copy

from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import Bool

ACCEL_Z = -0.1

class SanityChecker:


    def __init__(self, search_distance=10000, disappear_limit=100):
        # ROS Integration
        # rospy.init_node('obj_tracker', anonymous=True)
        self.pub = rospy.Publisher("/vision/pc/gt_centroid", TwistStamped, queue_size=50)
        self.neutral_pub = rospy.Publisher("/control/neutral_set", Bool, queue_size=50)
        rospy.Subscriber("/vision/pc/centroid", PointStamped, self.track_cb)

        # Data Read
        self.objs_found = []

        # Frame information Bounds x(@z=2.7)=[-2.7, 2.7] y(@z=2.7)=[-1.6, 0.1] z=[0.15, 3]
        self.frames = {} # stored as 'id': pointCloud
        self.num_frames = 0
        self.frame_id = 0

        # Objects Tracking
        self.objs = {} # stored as 'id': {'centroid': np.array({}), 'bounding_box': (coordinates), 'kf_params': {params}, 'startFrame': int x, 'frameCount': int y, 'consecutiveMisses': int z}
        self.num_objs = 0
        self.objs_id = 0

        # Search Distance (for tracking matches)
        self.search_distance = search_distance

        # Removal Threshold (for disappeared objects)
        self.disappear_limit = disappear_limit

def main():
    rospy.init_node('obj_tracker', anonymous=True)
    process = Kalman()

    # Control input set to factor in constant acceleration
    process.set_base_u_input(np.array([0, 0, 0.5*ACCEL_Z*(1**2), 0, 0, ACCEL_Z*1]))


    rospy.spin()


if __name__ == '__main__':
    main()