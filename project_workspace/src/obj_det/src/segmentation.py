#!/usr/bin/env python

"""
Filters out green ball from Kinect 1 and Kinect 2 camera input
"""

import rospy
import message_filters
import numpy as np
import cv2
import ros_numpy
from ros_numpy import numpy_msg
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

def filter(image, lower_thresh=(0, 0, 0), upper_thresh=(50, 255, 50)):
    """Only keep the green ball and filter everything else out"""
    image_copy = np.copy(image)
    for i in range(3):
        mask = ((image_copy[:,:,i] >= lower_thresh[i]).astype(int) * (image_copy[:,:,i] <= upper_thresh[i]).astype(int)).astype(bool)
        image_copy[(1 - mask).astype(bool)] = 0
    return image_copy.astype(np.uint8)

def bound(image, thresh=1):
    img_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret,thresh_img = cv2.threshold(img_grey, thresh, 255, cv2.THRESH_BINARY)
    _, contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    centroid = None
    max_area = 0
    rect = None

    if contours and len(contours):
        for cnt in contours:
            moment = cv2.moments(cnt)
            cx = int(moment['m10'] / moment['m00'])
            cy = int(moment['m01'] / moment['m00'])
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                centroid = (cx, cy)
                rect = cv2.boundingRect(cnt)
        image = cv2.rectangle(image,
            (rect[0], rect[1]),
            (rect[0] + max(rect[2], rect[3]), rect[1] + max(rect[2], rect[3])),
            (255, 0, 0),
            2
        )

    return image, centroid, max_area


class StereoImage:
    def __init__(self):
        self.seq = 1

        k1_cam_info_sub = message_filters.Subscriber('/camera/kinect1/color/camera_info', CameraInfo, queue_size=10)
        k2_cam_info_sub = message_filters.Subscriber('/camera/kinect2/color/camera_info', CameraInfo, queue_size=10)
        k1_img_sub = message_filters.Subscriber('/camera/kinect1/color/image_raw', Image, queue_size=10)
        k2_img_sub = message_filters.Subscriber('/camera/kinect2/color/image_raw', Image, queue_size=10)

        msg_filter = message_filters.ApproximateTimeSynchronizer([k1_img_sub, k2_img_sub, k1_cam_info_sub, k1_cam_info_sub], queue_size=10, slop=0.05)
        msg_filter.registerCallback(self.camera_callback)

        self.k1_centroid_pub = rospy.Publisher('/vision/k1/ball_centroid', PointStamped, queue_size=10)
        self.k2_centroid_pub = rospy.Publisher('/vision/k2/ball_centroid', PointStamped, queue_size=10)

        # transform from frame k1 to frame k2
        self.R = np.eye(3)
        self.T = np.ones((3, 1))

        self.k1_filtered_pub = rospy.Publisher("/vision/k1/filtered_image", Image, queue_size=10)
        self.k2_filtered_pub = rospy.Publisher("/vision/k2/filtered_image", Image, queue_size=10)

    def camera_callback(self, k1_img_msg, k2_img_msg, k1_camera_info_msg, k2_camera_info_msg):
        k1_img = ros_numpy.numpify(k1_img_msg)
        k2_img = ros_numpy.numpify(k2_img_msg)
        self.process_images(k1_img, k2_img)

    def process_images(self, k1_img, k2_img):
        k1_img = filter(k1_img)
        k2_img = filter(k2_img)
        
        k1_img, k1_centroid, k1_bbox_area = bound(k1_img)
        k2_img, k2_centroid, k2_bbox_area = bound(k2_img)

        if k1_centroid and 0 <= k1_centroid[0] < k1_img.shape[1] and 0 <= k1_centroid[1] < k1_img.shape[0]:
            k1_img[k1_centroid[1], k1_centroid[0], 0] = 255
            k1_img[k1_centroid[1], k1_centroid[0], 1] = 0
            k1_img[k1_centroid[1], k1_centroid[0], 2] = 0

        if k2_centroid and 0 <= k2_centroid[0] < k2_img.shape[1] and 0 <= k2_centroid[1] < k2_img.shape[0]:
            k2_img[k2_centroid[1], k2_centroid[0], 0] = 255
            k2_img[k2_centroid[1], k2_centroid[0], 1] = 0
            k2_img[k2_centroid[1], k2_centroid[0], 2] = 0

        if k1_centroid or k2_centroid:
            if k1_centroid and k2_centroid:
                print("K1 and K2 centroids and areas:")
                print(k1_centroid, k1_bbox_area)
                print(k2_centroid, k2_bbox_area)
            elif k1_centroid:
                print("K1 centroid found only: ")
                print(k1_centroid, k1_bbox_area)
            else:
                print("K2 centroid found only: ")
                print(k2_centroid, k2_bbox_area)
        else:
            print("No centroid found...")

        self.k1_filtered_pub.publish(ros_numpy.msgify(Image, k1_img, 'rgb8'))
        self.k2_filtered_pub.publish(ros_numpy.msgify(Image, k2_img, 'rgb8'))

        if k1_centroid:
            k1_centroid_msg = PointStamped()
            k1_centroid_msg.header.seq = self.seq
            k1_centroid_msg.header.stamp = rospy.Time.now()
            k1_centroid_msg.header.frame_id = 'kinect_ros_1_nc'
            k1_centroid_msg.point.x = k1_centroid[0]
            k1_centroid_msg.point.y = k1_centroid[1]
            k1_centroid_msg.point.z = 0
            self.k1_centroid_pub.publish(k1_centroid_msg)

        if k2_centroid:
            k2_centroid_msg = PointStamped()
            k2_centroid_msg.header.seq = self.seq
            k2_centroid_msg.header.stamp = rospy.Time.now()
            k2_centroid_msg.header.frame_id = 'kinect_ros_2_nc'
            k2_centroid_msg.point.x = k2_centroid[0]
            k2_centroid_msg.point.y = k2_centroid[1]
            k2_centroid_msg.point.z = 0
            self.k2_centroid_pub.publish(k2_centroid_msg)

        self.seq += 1

if __name__ == "__main__":
    rospy.init_node('StereoImage')
    node = StereoImage()
    rospy.spin()