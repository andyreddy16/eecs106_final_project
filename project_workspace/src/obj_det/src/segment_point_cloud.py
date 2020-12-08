#!/usr/bin/env python

"""
Filters out green ball from Kinect 1 and Kinect 2 camera input
"""

from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

import numpy as np
import cv2

from cv_bridge import CvBridge

from ros_numpy import numpy_msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped

from transforms import get_g_world_k1, get_g_world_k2

def filter(image, lower_thresh=(0, 50, 0), upper_thresh=(50, 255, 50)):
    """Only keep the green ball and filter everything else out"""
    image_copy = np.copy(image)
    for i in range(3):
        mask = ((image_copy[:,:,i] >= lower_thresh[i]).astype(int) * (image_copy[:,:,i] <= upper_thresh[i]).astype(int)).astype(bool)
        image_copy[(1 - mask).astype(bool)] = 0
    return image_copy.astype(np.uint8)

def segment_pointcloud(points, segmented_image, cam_matrix, trans, rot):
    print("Starting segment_pointcloud...")
    IDX2D = lambda i, j, dj: dj * i + j

    xyz = points.T

    pixel_coords = project_points(xyz, cam_matrix, trans, rot)

    image_h, image_w = segmented_image.shape[:2]
    in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                & (0 <= pixel_coords[1]) & (pixel_coords[1] < image_h))
    points = points[in_frame]

    if points is not None and len(points):
        print("Found %d pc points that can be seen by camera" % len(points))

    pixel_coords = pixel_coords[:, in_frame]

    j, i = pixel_coords

    greyscale_segmented_image = segmented_image[:,:,1]

    linearized_pixel_coords = IDX2D(i, j, greyscale_segmented_image.shape[1])
    linearized_segmentation = greyscale_segmented_image.reshape(-1)
    point_labels = linearized_segmentation[linearized_pixel_coords]

    print("Found %d pc points that match filtered image" % np.sum(point_labels != 0))

    return points[point_labels != 0]

def project_points(points, cam_matrix, trans, rot):
    """
    This funtion should perform the job of projecting the input pointcloud onto the frame
    of an image captured by a camera with camera matrix as given, of dimensions as given,
    in pixels.

    points is an 3 x N array where the ith entry is an (x, y, z) point in 3D space, in 
    the reference frame of the depth camera. This corresponds to the tf frame
    camera_depth_optical_frame. However, the image is taken by an RGB camera, with
    reference frame camera_color_optical_frame. (trans, rot) together give the translation
    vector and rotation matrix that transform points in the depth camera frame to points
    in the RGB camera frame.

    For each point in points, compute the pixel co-ordinates (u, v) onto which that point
    would be projected.

    This function should return a 2 x N integer array of pixel co-ordinates. The ith entry 
    should  be the index (u, v) of the pixel onto which the ith point in the pointcloud should 
    get projected.

    Use the point projection model introduced in the lab documentation to perform this
    projection.

    Note that this function should be able to operate on large pointclouds very efficiently.
    Make good use of numpy functions to vectorize and to act on the entire pointcloud at once.

    Args:
    
    points: (numpy.ndarray) Array of shape (3, N). ith entry is a 3D array representing
            a single (x, y, z) point in the reference frame of the camera.

    cam_matrix: (numpy.ndarray) Array of shape (3, 3) representing the camera intrinsic
                matrix.

                This parameter takes the standard form of the camera matrix as described
                in the lab doc:

                [[fx, s,  x0],
                 [0,  fy, y0],
                 [0,  0,  1 ]]

    trans: (numpy.ndarray) 1D array of length 3. This is the translation vector that
    offsets points in the depth camera reference frame to the RGB camera reference frame.

    rot: (numpy.ndarray) array of shape (3, 3). This is the 3x3 rotation matrix that takes
    points from the depth camera frame to the RGB camera frame.

    """

    # STEP 1: Transform pointcloud into new reference frame.
    points = np.dot(rot, points) + trans[:, None]

    # STEP 2: Project new pointcloud onto image frame using K matrix.
    # gives a 3 x N array of image plane coordinates in homogenous coordinates.
    homo_pixel_coords = np.dot(cam_matrix, points)

    # STEP 3: Convert homogenous coordinates to regular 2D coordinates.
    # To do this, you need to divide the first two coordinates of homo_pixel_coords
    # by the third coordinate.
    pixel_coords = homo_pixel_coords[:2, :] / homo_pixel_coords[2, :]

    # STEP 4: Convert to integers. Take the floor of pixel_coords then cast it
    # to an integer type, like numpy.int32
    pixel_coords = np.floor(pixel_coords).astype(int)

    return pixel_coords


def isolate_object_of_interest(points, image, cam_matrix, trans, rot, cam_to_world_R, cam_to_world_T):
    segmented_image = filter(image)
    points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    
    print("Isolated ball points in point cloud:")
    print(points)

    if points is not None and len(points):
        centroid = np.median(points, axis=0)
        print("Found centroid at (%.3f, %.3f, %.3f)" % (centroid[0], centroid[1], centroid[2]))
        world_centroid = np.dot(cam_to_world_R, centroid) + cam_to_world_T.flatten()
        print("In world coords: (%.3f, %.3f, %.3f)" % (world_centroid[0], world_centroid[1], world_centroid[2]))
    else:
        print("Ball not detected")
    
    print("\n")
    print("\n")

    return points

def get_camera_matrix(camera_info_msg):
    # TODO: Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    # Hint: numpy.reshape may be useful here.
    # print("HERE")
    # print(camera_info_msg.K)
    # print(np.reshape(np.array(camera_info_msg.K), (3,3)))
    return np.reshape(np.array(camera_info_msg.K), (3,3))

def numpy_to_pc2_msg(points):

    data = np.zeros((points.shape[0],), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('r', np.uint8),
        ('g', np.uint8),
        ('b', np.uint8)])

    data['x'] = points[:, 0]
    data['y'] = points[:, 1]
    data['z'] = points[:, 2]
    data['r'] = 0
    data['g'] = 255
    data['b'] = 0

    return ros_numpy.msgify(PointCloud2, data, frame_id='world')

class PointcloudProcess:
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.

    """
    def __init__(self, points_sub_topic, 
                       image_sub_topic,
                       cam_info_topic,
                       points_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None

        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        # self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)

        self.filtered_pub = rospy.Publisher('/vision/pc/filtered_image', Image, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        
        self.G01 = get_g_world_k1()

        # self.G01 = np.dot(self.G01, self.G_prime)

        # self.camera_offset = np.array([
        #     [0, -1, 0, 0],
        #     [0, 0, -1, 0],
        #     [1, 0, 0, 0],
        #     [0, 0, 0, 1]
        # ])

        self.camera_offset = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])

        self.G01 = np.dot(self.G01, self.camera_offset)

        self.G01_R = self.G01[:3, :3]
        self.G01_T = self.G01[:3, 3].flatten()

        ts.registerCallback(self.callback)

    def callback(self, points_msg, image, info):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            cloud_points = np.array(list(point_cloud2.read_points(points_msg, skip_nans=True, field_names = ("x", "y", "z"))))
        except Exception as e:
            rospy.logerr(e)
            return

        self.messages.appendleft((cloud_points, rgb_image, intrinsic_matrix))
        self.publish_once_from_queue()
        self.num_steps += 1
        

    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()

            if points is not None and len(points):
                points = isolate_object_of_interest(points, image, info, 
                    np.zeros(3), np.eye(3),
                    self.G01_R, self.G01_T)

                if points is not None and points.shape[0]:
                    # print(points)
                    points_msg = numpy_to_pc2_msg(points)

                    self.points_pub.publish(points_msg)
                    # print("Published segmented pointcloud at timestamp:",
                           # points_msg.header.stamp.secs)

def main():
    CAM_INFO_TOPIC = '/camera/kinect1/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/kinect1/color/image_raw'
    POINTS_TOPIC = '/camera/kinect1/depth/points'
    POINTS_PUB_TOPIC = 'segmented_points'

    rospy.init_node('pointcloud_segmenter')
    process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    rospy.spin()

if __name__ == '__main__':
    main()