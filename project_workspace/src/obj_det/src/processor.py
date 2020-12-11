#!/usr/bin/env python

"""
Listens to centroid topic and computes optimal angle of board contact
"""

import rospy
import ros_numpy
import message_filters

import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, TwistStamped

def calculate_board_tilt(raw_points, kf_points):
    """Use physics inferencer class to calculate optimal board tilt values"""
    # Skeleton code, replace line below
    return "right", 0.05

class Processor:
    """Listens to KF_CENTROID_TOPIC topic, feeds data to physics inferencer, and publishes
    optimal board tilt values to the BOARD_TILT_TOPIC topic
    """
    def __init__(self):
        RAW_CENTROID_TOPIC = "/vision/pc/centroid"
        KF_CENTROID_TOPIC = "/vision/pc/kf_centroid"

        BOARD_TILT_TOPIC = "/joint_ctrl/board_tilt"

        self.raw_centroid_sub = message_filters.Subscriber(RAW_CENTROID_TOPIC, PointStamped)
        self.kf_centroid_sub = message_filters.Subscriber(KF_CENTROID_TOPIC, TwistStamped)

        self.board_tilt_pub = rospy.Publisher(BOARD_TILT_TOPIC, String)

        self.raw_points = []
        self.kf_points = []

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.raw_centroid_sub, self.kf_centroid_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, raw_msg, kf_msg):
        """Callback method from ApproximateTimeSynchronizer of raw centroid values
        and KF centroid values"""
        raw_header = raw_msg.header
        kf_header = kf_msg.header

        raw_point = (raw_msg.point.x, raw_msg.point.y, raw_msg.point.z)
        kf_point = (kf_msg.twist.linear.x, kf_msg.twist.linear.y, kf_msg.twist.linear.z,
            kf_msg.twist.angular.x, kf_msg.twist.angular.y, kf_msg.twist.angular.z)

        self.raw_points.append(raw_point)
        self.kf_points.append(kf_point)

        direction, angle = calculate_board_tilt(self.raw_points, self.kf_points)
        self.board_tilt_pub.publish("dir=%s, angle=%.4f" % (direction, angle))


def main():
    rospy.init_node('processor')
    process = Processor()
    rospy.spin()

if __name__ == '__main__':
    main()