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

from transforms import get_g_0k

# Account for camera frames being defined as +Z pointing outwards from cam
KINECT_OFFSET = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0.036],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])

FILTER_PRESETS = [
    [(0, 60, 0), (40, 255, 40)],
    [(0, 60, 0), (40, 255, 40)],
    [(100, 150, 100), (255, 255, 255)]
]

def get_RT(G):
    """Extracts R and T from G"""
    R = G[:3, :3]
    T = G[:3, 3].flatten()
    return R, T

def get_camera_matrix(camera_info_msg):
    """Gets camera matrix from camera info msg"""
    return np.reshape(np.array(camera_info_msg.K), (3,3))

def numpy_to_pc2_msg(points):
    """Given Nx3 points, assemble into PointCloud2 data object"""
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

def green_dominant_filter(image):
    b,g,r = cv2.split(image)
    target = g
    other1 = b
    other2 = r

    # Figure out which ones we need to zero & zero them
    should_zero = (target < 1.5*other1) | (target < 1.5*other2)
    g[should_zero] = 0
    r[should_zero] = 0
    b[should_zero] = 0

    # Merge channels back
    return cv2.merge((b,g,r))

def filter(image, lower_thresh=(0, 60, 0), upper_thresh=(40, 255, 40)):
    """Only keep the green ball and filter everything else out"""
    image = green_dominant_filter(image)
    image_copy = np.copy(image)
    # for i in range(3):
    #     mask = ((image_copy[:,:,i] >= lower_thresh[i]).astype(int) * (image_copy[:,:,i] <= upper_thresh[i]).astype(int)).astype(bool)
    #     image_copy[(1 - mask).astype(bool)] = 0
    return image_copy.astype(np.uint8)

def segment_pointcloud(points, segmented_image, cam_mat, trans, rot):
    """Segment the pointcloud by finding relationship between
    image_raw and pointcloud, and only keeping pc points that
    correspond with nonzero pixels in filtered image.
    """
    print("Starting segment_pointcloud...")
    IDX2D = lambda i, j, dj: dj * i + j # convert 2d coord to 1d
    xyz = points.T # needs to be 3 x N

    # project pc points onto image to find relationship
    pixel_coords = project_points(xyz, cam_mat, trans, rot)

    # only look at pc points that could be seen by image_raw
    image_h, image_w = segmented_image.shape[:2]
    in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                & (0 <= pixel_coords[1]) & (pixel_coords[1] < image_h))
    points = points[in_frame]

    if points is not None and len(points):
        print("Found %d pc points that can be seen by camera" % len(points))

    pixel_coords = pixel_coords[:, in_frame]

    j, i = pixel_coords

    # get greyscale image (only take the 'G' channel)
    greyscale_segmented_image = segmented_image[:,:,1]

    # find all the points that correspond with nonzero value in greyscale_segmented_image
    linearized_pixel_coords = IDX2D(i, j, greyscale_segmented_image.shape[1])
    linearized_segmentation = greyscale_segmented_image.reshape(-1)
    point_labels = linearized_segmentation[linearized_pixel_coords]

    print("Found %d pc points that match filtered image" % np.sum(point_labels != 0))

    return points[point_labels != 0]

def project_points(points, cam_mat, trans, rot):
    """Projects the input pointcloud onto the frame of an image captured
    by a camera.
    """
    # transform pointcloud into new reference frame.
    points = np.dot(rot, points) + trans[:, None]
    # project new pointcloud onto image frame using K matrix.
    homo_pixel_coords = np.dot(cam_mat, points)
    # convert homogenous coordinates to regular 2D coordinates.
    pixel_coords = homo_pixel_coords[:2, :] / homo_pixel_coords[2, :]
    # convert to integers coords
    pixel_coords = np.floor(pixel_coords).astype(int)
    return pixel_coords

def isolate_object_of_interest(points, img, cam_mat, depth_to_img_raw, vis=False, pub=None, filter_preset_id=0):
    """Given an image and pointcloud, applies a filter to both image and pointcloud
    """
    depth_to_raw_R = depth_to_img_raw[:3, :3]
    depth_to_raw_T = depth_to_img_raw[:3, 3].flatten()
    segmented_img = filter(img, lower_thresh=FILTER_PRESETS[filter_preset_id][0], upper_thresh=FILTER_PRESETS[filter_preset_id][1])
    if vis and pub:
        pub.publish(ros_numpy.msgify(Image, segmented_img, 'rgb8'))
    points = segment_pointcloud(points, segmented_img, cam_mat, depth_to_raw_T, depth_to_raw_R)    
    if points is None or len(points) == 0:
        return None
    return points


class PointcloudProcess:
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.
    """
    def __init__(self, num_kinects=3):

        self.NUM_KINECTS = num_kinects
        self.seq = 1

        PTS_TOPICS = ['/camera/kinect%d/depth/points' % (i+1) for i in range(self.NUM_KINECTS)]
        CAM_INFO_TOPICS = ['/camera/kinect%d/color/camera_info' % (i+1) for i in range(self.NUM_KINECTS)]
        RGB_IMG_TOPICS = ['/camera/kinect%d/color/image_raw' % (i+1) for i in range(self.NUM_KINECTS)]
        FILTERED_IMG_TOPICS = ['/vision/pc/k%d_filtered_image' % (i+1) for i in range(self.NUM_KINECTS)]
        SEGMENTED_PC_PUB_TOPIC = '/vision/pc/segmented_points'
        CENTROID_TOPIC = '/vision/pc/centroid'

        assert len(PTS_TOPICS) == self.NUM_KINECTS
        assert len(CAM_INFO_TOPICS) == self.NUM_KINECTS
        assert len(RGB_IMG_TOPICS) == self.NUM_KINECTS

        # Transform b/w world and kinect for each kinect
        self.G0k = [get_g_0k(i) for i in range(self.NUM_KINECTS)]

        for i in range(self.NUM_KINECTS):
            self.G0k[i] = np.dot(self.G0k[i], np.linalg.inv(KINECT_OFFSET))

        # Extract R and T for each transform
        self.G0k_R = []
        self.G0k_T = []

        for i in range(self.NUM_KINECTS):
            R, T = get_RT(self.G0k[i])
            self.G0k_R.append(R)
            self.G0k_T.append(T)

        self.num_steps = 0
        self.messages = deque([], 5)

        # Pointcloud, Raw image, and Cam Info subscribers
        self.pts_subs = [message_filters.Subscriber(PTS_TOPICS[i], PointCloud2) for i in range(self.NUM_KINECTS)]
        self.img_subs = [message_filters.Subscriber(RGB_IMG_TOPICS[i], Image) for i in range(self.NUM_KINECTS)]
        self.cam_info_subs = [message_filters.Subscriber(CAM_INFO_TOPICS[i], CameraInfo) for i in range(self.NUM_KINECTS)]

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        # Segmented point cloud, filtered images, centroid publishers
        self.segmented_pc_pub = rospy.Publisher(SEGMENTED_PC_PUB_TOPIC, PointCloud2, queue_size=10)
        self.filtered_pubs = [rospy.Publisher(FILTERED_IMG_TOPICS[i], Image, queue_size=10) for i in range(self.NUM_KINECTS)]
        self.centroid_pub = rospy.Publisher(CENTROID_TOPIC, PointStamped, queue_size=10)

        message = []
        for i in range(self.NUM_KINECTS):
            message.extend([self.pts_subs[i], self.img_subs[i], self.cam_info_subs[i]])

        ts = message_filters.ApproximateTimeSynchronizer(message, 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, k1_pts=None, k1_img=None, k1_info=None,
            k2_pts=None, k2_img=None, k2_info=None,
            k3_pts=None, k3_img=None, k3_info=None
        ):
        """Callback method for ApproximateTimeSynchronizer with optional parameters to
        support 2 Kinects and 3 Kinects"""
        params = locals()
        pts_msgs = []
        img_msgs = []
        info_msgs= []
        intrinsics = []
        rgb_imgs = []
        pc_points = []
        message = []

        for i in range(self.NUM_KINECTS):
            pts_msgs.append(params["k%d_pts" % (i+1)])
            img_msgs.append(params["k%d_img" % (i+1)])
            info_msgs.append(params["k%d_info" % (i+1)])
        try:
            for i in range(self.NUM_KINECTS):
                intrinsics.append(get_camera_matrix(info_msgs[i]))
                rgb_imgs.append(ros_numpy.numpify(img_msgs[i]))
                pc_points.append(np.array(list(point_cloud2.read_points(pts_msgs[i], skip_nans=True, field_names=("x", "y", "z")))))
            for i in range(self.NUM_KINECTS):
                message.extend([pc_points[i], rgb_imgs[i], intrinsics[i]])
        except Exception as e:
            rospy.logerr(e)
            return
        self.messages.appendleft(tuple(message))
        self.process_point_cloud()
        self.num_steps += 1
        return

    def process_point_cloud(self):
        """Process point cloud data given image and camera intrinsic matrix info"""
        if not self.messages:
            return
        message = self.messages.pop()
        pts = []
        imgs = []
        infos = []

        for i in range(self.NUM_KINECTS):
            pts.append(message[3*i])
            imgs.append(message[3*i + 1])
            infos.append(message[3*i + 2])

        cumulative_world_pts = []
        cumulative_world_centroid = None

        print(self.seq)
        for i in range(self.NUM_KINECTS):
            if pts[i] is not None and len(pts[i]):
                pts[i] = isolate_object_of_interest(pts[i], imgs[i], infos[i],
                    np.eye(4), vis=True, pub=self.filtered_pubs[i], filter_preset_id=i)
                if pts[i] is not None:
                    world_pts = np.dot(self.G0k_R[i], pts[i].T).T + self.G0k_T[i]
                    world_centroid = np.median(world_pts, axis=0)
                    print("Centroid found by Kinect %d: (%.3f, %.3f, %.3f)" % (i, world_centroid[0], world_centroid[1], world_centroid[2]))
                    cumulative_world_pts.extend(world_pts)

        if cumulative_world_pts:
            cumulative_world_pts = np.array(cumulative_world_pts)
            cumulative_world_centroid = np.median(cumulative_world_pts, axis=0)
            print("Cumulative centroid: (%.3f, %.3f, %.3f)" % (cumulative_world_centroid[0],
                cumulative_world_centroid[1], cumulative_world_centroid[2]))
            world_points_msg = numpy_to_pc2_msg(cumulative_world_pts)
            self.segmented_pc_pub.publish(world_points_msg)

            centroid_msg = PointStamped()
            centroid_msg.header.seq = self.seq
            centroid_msg.header.stamp = rospy.Time.now()
            centroid_msg.header.frame_id = 'world'
            centroid_msg.point.x = cumulative_world_centroid[0]
            centroid_msg.point.y = cumulative_world_centroid[1]
            centroid_msg.point.z = cumulative_world_centroid[2]
            self.centroid_pub.publish(centroid_msg)

        self.seq += 1
        print("\n")
        print("\n")


def main():
    rospy.init_node('pointcloud_segmenter')
    process = PointcloudProcess(num_kinects=3)
    rospy.spin()

if __name__ == '__main__':
    main()