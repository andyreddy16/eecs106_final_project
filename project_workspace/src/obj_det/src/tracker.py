#!/usr/bin/env python

"""
Tracks centroid and computes optimal angle of board contact
"""

import rospy
import ros_numpy
import message_filters

import numpy as np

from geometry_msgs.msg import PointStamped, TwistStamped

def process(raw_points, kf_points):
    pass

class Tracker:
    def __init__(self):
        RAW_CENTROID_TOPIC = "/vision/pc/centroid"
        KF_CENTROID_TOPIC = "/vision/pc/kf_centroid"

        self.raw_centroid_sub = message_filters.Subscriber(RAW_CENTROID_TOPIC, PointStamped)
        self.kf_centroid_sub = message_filters.Subscriber(KF_CENTROID_TOPIC, TwistStamped)

        self.raw_points = []
        self.kf_points = []

        ts = message_filters.ApproximateTimeSynchronizer([self.raw_centroid_sub, self.kf_centroid_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, raw_msg, kf_msg):
        print("Callback...", len(self.raw_points))

        raw_header = raw_msg.header
        kf_header = kf_msg.header

        raw_point = (raw_msg.point.x, raw_msg.point.y, raw_msg.point.z)
        kf_point = (kf_msg.twist.linear.x, kf_msg.twist.linear.y, kf_msg.twist.linear.z,
            kf_msg.twist.angular.x, kf_msg.twist.angular.y, kf_msg.twist.angular.z)

        self.raw_points.append(raw_point)
        self.kf_points.append(kf_point)

        # do something
        process(self.raw_points, self.kf_points)

def main():
    rospy.init_node('tracker')
    process = Tracker()
    rospy.spin()

if __name__ == '__main__':
    main()