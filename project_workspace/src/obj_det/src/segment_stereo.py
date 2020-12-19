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
from epipolar import filter_by_epipolar
from transforms import get_g_0k

# Account for camera frames being defined as +Z pointing outwards from cam
KINECT_OFFSET = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0.036],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])
CENTROID_TOPIC = '/vision/pc/centroid'

def green_dominant_filter(image, coeff=1.5):
    """Filter out pixels where G < coeff*B or G < coeff*R"""
    b,g,r = cv2.split(image)
    zeroed_idxs = (g < coeff*b) | (g < coeff*r)
    g[zeroed_idxs] = 0
    r[zeroed_idxs] = 0
    b[zeroed_idxs] = 0
    # Merge channels back
    return cv2.merge((b,g,r))

def filter(image, lower_thresh=(0, 60, 0), upper_thresh=(40, 255, 40)):
    """Only keep the green ball and filter everything else out"""
    image = green_dominant_filter(image)
    image_copy = np.copy(image)
    return image_copy.astype(np.uint8)

def bound(image, thresh=1):
    """Use OpenCV bounding boxes to detect center of ball"""
    img_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret,thresh_img = cv2.threshold(img_grey, thresh, 255, cv2.THRESH_BINARY)
    _, contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    centroid = None
    max_area = 0
    rect = None

    if contours and len(contours):
        for cnt in contours:
            moment = cv2.moments(cnt)
            if 'm00' in moment and moment['m00'] != 0:
                cx = int(moment['m10'] / moment['m00'])
                cy = int(moment['m01'] / moment['m00'])
                area = cv2.contourArea(cnt)
                if area > max_area and area >= 50:
                    max_area = area
                    centroid = (cx, cy)
                    rect = cv2.boundingRect(cnt)
        if rect:
            image = cv2.rectangle(image,
                (rect[0], rect[1]),
                (rect[0] + max(rect[2], rect[3]), rect[1] + max(rect[2], rect[3])),
                (255, 0, 0),
                2
            )

    return image, centroid, max_area

def least_squares_triangulate(x_1, x_2, R, T, k1_intrinsic, k2_intrinsic):
    k1_intrinsic_inv = np.linalg.inv(k1_intrinsic)
    k2_intrinsic_inv = np.linalg.inv(k2_intrinsic)

    x1 = np.dot(k1_intrinsic_inv, x_1)
    x2 = np.dot(k2_intrinsic_inv, x_2)
  
    A = np.hstack((np.dot(-R, x1), x2))
   
    ls_sol = np.dot(np.linalg.inv(np.dot(A.T, A)), np.dot(A.T, T))    
    lambda_1, lambda_2 = ls_sol[0], ls_sol[1]
    
    # print(lambda_1, lambda_2)
    if lambda_1 > 0 and lambda_2 > 0:
        X1 = lambda_1*np.dot(R, x1) + T
        X2 = lambda_2*x2
        X = .5 * (X1 + X2)
        return X, lambda_1, lambda_2
    elif lambda_1 > 0:
        return lambda_1 * np.dot(R, x1) + T, lambda_1, lambda_2
    elif lambda_2 > 0:
        return lambda_2 * x2, lambda_1, lambda_2
    else:
        return None


class StereoImage:
    def __init__(self, epipolar_threshold=0.05):
        self.seq = 1
        self.feature_detector = cv2.BRISK_create(50, octaves=5)
        self.threshold = epipolar_threshold

        k1_cam_info_sub = message_filters.Subscriber('/camera/kinect1/color/camera_info', CameraInfo, queue_size=10)
        k2_cam_info_sub = message_filters.Subscriber('/camera/kinect2/color/camera_info', CameraInfo, queue_size=10)
        k1_img_sub = message_filters.Subscriber('/camera/kinect1/color/image_raw', Image, queue_size=10)
        k2_img_sub = message_filters.Subscriber('/camera/kinect2/color/image_raw', Image, queue_size=10)

        msg_filter = message_filters.ApproximateTimeSynchronizer([k1_img_sub, k2_img_sub, k1_cam_info_sub, k1_cam_info_sub], queue_size=10, slop=0.05)
        msg_filter.registerCallback(self.camera_callback)

        self.k1_centroid_pub = rospy.Publisher('/vision/k1/ball_centroid', PointStamped, queue_size=10)
        self.k2_centroid_pub = rospy.Publisher('/vision/k2/ball_centroid', PointStamped, queue_size=10)

        self.g01 = get_g_0k(0)
        self.g02 = get_g_0k(1)

        self.g01 = np.dot(self.g01, np.linalg.inv(KINECT_OFFSET))
        self.g02 = np.dot(self.g02, np.linalg.inv(KINECT_OFFSET))

        self.g02_R = self.g02[:3, :3]
        self.g02_T = self.g02[:3, 3].reshape((3, 1))

        # transform between kinect1 and kinect2
        self.G_12 = np.dot(np.linalg.inv(self.g01), self.g02)
        self.R_12 = self.G_12[:3, :3]
        self.T_12 = self.G_12[:3, 3].reshape((3, 1))

        # transform between kinect2 and kinect1
        self.G_21 = np.dot(np.linalg.inv(self.g02), self.g01)
        self.R_21 = self.G_21[:3, :3]
        self.T_21 = self.G_21[:3, 3].reshape((3, 1))

        self.k1_filtered_pub = rospy.Publisher("/vision/k1/filtered_image", Image, queue_size=10)
        self.k2_filtered_pub = rospy.Publisher("/vision/k2/filtered_image", Image, queue_size=10)
        self.match_pub = rospy.Publisher("/vision/matches", Image, queue_size=10)
        self.centroid_pub = rospy.Publisher(CENTROID_TOPIC, PointStamped, queue_size=10)

    def camera_callback(self, k1_img_msg, k2_img_msg, k1_camera_info_msg, k2_camera_info_msg):
        k1_img = ros_numpy.numpify(k1_img_msg)
        k2_img = ros_numpy.numpify(k2_img_msg)
        k1_intrinsic = np.array(k1_camera_info_msg.K).reshape((3,3))
        k2_intrinsic = np.array(k2_camera_info_msg.K).reshape((3,3))

        # self.feature_match(k1_img, k2_img, k1_intrinsic, k2_intrinsic)
        self.process_images(k1_img, k2_img, k1_intrinsic, k2_intrinsic)

    def feature_match(self, k1_img, k2_img, k1_intrinsic, k2_intrinsic):
        kp1, des1 = self.feature_detector.detectAndCompute(k1_img, None)
        kp2, des2 = self.feature_detector.detectAndCompute(k2_img, None)

        matches = None
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        try:
            matches = matcher.match(des1, des2)

            k1_image_matches = np.array([kp1[i.queryIdx].pt for i in matches])
            k2_image_matches = np.array([kp2[i.trainIdx].pt for i in matches])

            match_vis1 = cv2.drawMatches(
                k1_img, kp1, 
                k2_img, kp2, matches, 0,
                flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            
            inlier_mask = np.array(
                filter_by_epipolar(k1_intrinsic, k2_intrinsic,
                    kp1, kp2, self.R_21, self.T_21, self.threshold, matches)) == 1

            k1_image_masked = np.pad(k1_image_matches[inlier_mask], [(0, 0), (0, 1)], mode='constant', constant_values=1)
            k2_image_masked = np.pad(k2_image_matches[inlier_mask], [(0, 0), (0, 1)], mode='constant', constant_values=1)
            
            filtered_matches = [m for m, b in zip(matches, inlier_mask) if b == 1]

            match_vis2 = cv2.drawMatches(
                k1_img, kp1, 
                k2_img, kp2, filtered_matches, 0,
                flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

            match_vis = cv2.vconcat([match_vis1, match_vis2])

            # print("Num filtered matches: %d" % len(filtered_matches))
            # self.match_pub.publish(ros_numpy.msgify(Image, match_vis, 'rgb8'))

            for i in range(k1_image_masked.shape[0]):
                x_1 = k1_image_masked[i, :].reshape((-1, 1))
                x_2 = k2_image_masked[i, :].reshape((-1, 1))
                ret = least_squares_triangulate(x_1, x_2, self.R_21, self.T_21, k1_intrinsic, k2_intrinsic)
                if ret:
                    x_w, l1, l2 = ret
                    # print("Found lambdas: %f, %f" % (l1, l2))
        except:
            pass
        
        return kp1, kp2, matches

    def process_images(self, k1_img, k2_img, k1_intrinsic, k2_intrinsic):
        k1_img = filter(k1_img)
        k2_img = filter(k2_img)
        
        k1_img, k1_centroid, k1_bbox_area = bound(k1_img)
        k2_img, k2_centroid, k2_bbox_area = bound(k2_img)

        if k1_centroid and k2_centroid:
            # print("K1 and K2 centroids and areas:")
            # print(k1_centroid, k1_bbox_area)
            # print(k2_centroid, k2_bbox_area)
            
            k1_homo = np.array([k1_centroid[0], k1_centroid[1], 1]).reshape((-1,1))
            k2_homo = np.array([k2_centroid[0], k2_centroid[1], 1]).reshape((-1,1))
            
            ret = least_squares_triangulate(k1_homo, k2_homo, self.R_21, self.T_21, k1_intrinsic, k2_intrinsic)
            if ret is not None:
                x_w, _, _ = ret
                x_w = x_w.reshape((-1, 1))
                x_w = np.dot(self.g02_R, x_w).reshape((-1, 1)) + self.g02_T.reshape((-1, 1))
                # print("3D point ", x_w)

                centroid_msg = PointStamped()
                centroid_msg.header.seq = self.seq
                centroid_msg.header.stamp = rospy.Time.now()
                centroid_msg.header.frame_id = 'world'
                centroid_msg.point.x = x_w[0]
                centroid_msg.point.y = x_w[1]
                centroid_msg.point.z = x_w[2]
                self.centroid_pub.publish(centroid_msg)

        # self.k1_filtered_pub.publish(ros_numpy.msgify(Image, k1_img, 'rgb8'))
        # self.k2_filtered_pub.publish(ros_numpy.msgify(Image, k2_img, 'rgb8'))

        # if k1_centroid:
        #     k1_centroid_msg = PointStamped()
        #     k1_centroid_msg.header.seq = self.seq
        #     k1_centroid_msg.header.stamp = rospy.Time.now()
        #     k1_centroid_msg.header.frame_id = 'kinect_ros_1_nc'
        #     k1_centroid_msg.point.x = k1_centroid[0]
        #     k1_centroid_msg.point.y = k1_centroid[1]
        #     k1_centroid_msg.point.z = 0
        #     self.k1_centroid_pub.publish(k1_centroid_msg)

        # if k2_centroid:
        #     k2_centroid_msg = PointStamped()
        #     k2_centroid_msg.header.seq = self.seq
        #     k2_centroid_msg.header.stamp = rospy.Time.now()
        #     k2_centroid_msg.header.frame_id = 'kinect_ros_2_nc'
        #     k2_centroid_msg.point.x = k2_centroid[0]
        #     k2_centroid_msg.point.y = k2_centroid[1]
        #     k2_centroid_msg.point.z = 0
        #     self.k2_centroid_pub.publish(k2_centroid_msg)

        self.seq += 1

if __name__ == "__main__":
    rospy.init_node('stereo_segmenter')
    node = StereoImage()
    rospy.spin()