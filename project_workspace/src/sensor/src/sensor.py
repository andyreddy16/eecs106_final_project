#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud #http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud.html
from std_msgs.msg import String

def callback(message):
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud.html
    header = message.header
    points = message.points
    channel_info = message.channels
    # do stuff
    print(points)
    print(channel_info)

def listener():
    rospy.Subscriber("/robot/sonar/head_sonar/state", PointCloud, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pointcloud_listener', anonymous=True)
    listener()
