#!/usr/bin/env python

import rospy
import std_msgs
import numpy as np
import copy

from geometry_msgs.msg import PointStamped, TwistStamped


class Kalman:
    # Testing for simulation
    # 1. Object moving through space
    # 2. See if filter can overcome noisy measurements (adversarial)
    # 3. Objects disappearing and reappearing (from same edgepoint and different edgepoint)
    # 4. Objects crossing paths

    # Sensing capabilities: Point Detection (3-axis), Color Detection (RGB values)

    # Kalman Filter Parameters -- Baseline for each object
    kf_params = {}
    kf_params['x_m'] = [np.zeros(6)] # x_measured: x, y, z, (dx, dy, dz)
    kf_params['x_p'] = [None] # x_predicted: x, y, z, dx, dy, dz
    kf_params['z'] = [] # true measurement
    kf_params['u'] = [np.zeros(6)] # Control input
    kf_params['A'] = np.eye(6) # State Transition
    for i in range(3):
        kf_params['A'][i, i+3] = 1

    kf_params['H'] = np.zeros((3, 6)) # Observation Transition
    for i in range(3):
        kf_params['H'][i, i] = 1
    kf_params['Q'] = np.eye(6) # State Noise
    kf_params['R'] = np.eye(3) # Observation Noise
    kf_params['P_m'] = [np.eye(6)] # Measured Variance
    kf_params['P_p'] = [None] # Predicted Variance

    def __init__(self, search_distance=10000, disappear_limit=100):
        # ROS Integration
        rospy.init_node('obj_tracker', anonymous=True)
        self.pub = rospy.Publisher("/vision/pc/kf_centroid", TwistStamped, queue_size=50)
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

        rospy.spin()


    def set_search_distance(self, search_distance):
        self.search_distance = search_distance


    def set_disappear_limit(self, disappear_limit):
        self.disappear_limit = disappear_limit


    def set_base_Q_var(self, cov):
        assert cov.shape == self.kf_params['Q'].shape
        self.kf_params['Q'] = cov


    def set_base_R_var(self, cov):
        assert cov.shape == self.kf_params['R'].shape
        self.kf_params['R'] = cov


    def set_base_u_input(self, control):
        assert control.shape == self.kf_params['u'].shape
        self.kf_params['u'] = [control]


    def kf_predict(self, obj_id):
        kf_params = self.objs[obj_id]['kf_params']
        # Prediction Step
        kf_params['x_p'].append(np.dot(kf_params['A'], kf_params['x_m'][-1]) + kf_params['u'][-1])
        kf_params['P_p'].append(np.dot(np.dot(kf_params['A'], kf_params['P_m'][-1]), kf_params['A'].T) + kf_params['Q'])
        return kf_params['x_p'][-1], kf_params['P_p'][-1]


    def kf_update_no_measurement(self, obj_id):
        kf_params = self.objs[obj_id]['kf_params']
        # No Measurement Update Step
        kf_params['x_m'].append(kf_params['x_p'][-1])
        kf_params['P_m'].append(kf_params['P_p'][-1])
        return kf_params['x_m'][-1], kf_params['P_m'][-1]


    def kf_update(self, obj_id):
        kf_params = self.objs[obj_id]['kf_params']
        # Measurement Update Step
        K = np.dot(np.dot(kf_params['P_p'][-1], kf_params['H'].T), np.linalg.inv(np.dot(np.dot(kf_params['H'], kf_params['P_p'][-1]), kf_params['H'].T) + kf_params['R']))
        kf_params['x_m'].append(kf_params['x_p'][-1] + np.dot(K, kf_params['z'][-1] - np.dot(kf_params['H'], kf_params['x_p'][-1])))
        kf_params['P_m'].append(np.dot(np.eye(6) - np.dot(K, kf_params['H']), kf_params['P_p'][-1]))
        return kf_params['x_m'][-1], kf_params['P_m'][-1]


    def add_measurement(self, centroid, obj_id):
        self.objs[obj_id]['kf_params']['z'].append(centroid)
        self.objs[obj_id]['frameCount'] += 1


    def update_location(self, obj_d, obj_id):
        self.objs[obj_id]['centroid'] = obj_d['centroid']


    def clear_misses(self, obj_id):
        self.objs[obj_id]['consecutiveMisses'] = 0


    def add_miss(self, obj_id):
        self.objs[obj_id]['consecutiveMisses'] += 1


    def add_object(self, centroid):
        self.objs[self.objs_id] = {'centroid': centroid, 'kf_params': copy.deepcopy(Kalman.kf_params), 'startFrame': self.frame_id, 'frameCount': 1, 'consecutiveMisses': 0}
        self.objs[self.objs_id]['kf_params']['x_m'][0] = np.hstack((centroid, np.zeros(3)))
        self.num_objs += 1
        self.objs_id += 1


    def remove_object(self, obj_id):
        del self.objs[self.objs_id]
        self.num_objs -= 1


    def match_tracking(self):
        objs_detected = self.objs_found

        # No previously tracked objects -> add all for tracking
        if self.num_objs == 0:
            for idx, obj_d in enumerate(objs_detected):
                self.add_object(obj_d['centroid'])
            return len(objs_detected), set(), set()
        # Match with existing objects
        else:
            # # Keep set of matched and unmatched object IDs
            unmatched = list(self.objs.keys())
            matched = set()
            new_objs_count = 0
            self.update_location(objs_detected[0], unmatched[0])
            self.add_measurement(objs_detected[0]['centroid'], unmatched[0])
            self.clear_misses(unmatched[0])
            matched.add(unmatched[0])
            unmatched.remove(unmatched[0])
        
        return new_objs_count, matched, unmatched


    def track_objs(self):
        # Run Kalman Filter predictions on tracked objects
        for obj_id in self.objs.keys():
            self.kf_predict(obj_id)

        # Cross check detected vs tracked objects to add measurement/add new object
        added_objs, matched_objs, unmatched_objs = self.match_tracking()

        # Run Kalman Filter measurement update on matched objects
        for obj_id in matched_objs:
            self.kf_update(obj_id)

        # Run Kalman Filter non-measurement update on unmatched objects
        for obj_id in unmatched_objs:
            self.kf_update_no_measurement(obj_id)

        self.objs_found = []

        if 0 in self.objs and len(self.objs[0]['kf_params']['x_m']):
            kf_centroid = self.objs[0]['kf_params']['x_m'][-1]
            kf_centroid_msg = TwistStamped()
            kf_centroid_msg.header.seq = self.frame_id
            kf_centroid_msg.header.stamp = rospy.Time.now()
            kf_centroid_msg.header.frame_id = 'world'
            kf_centroid_msg.twist.linear.x = kf_centroid[0]
            kf_centroid_msg.twist.linear.y = kf_centroid[1]
            kf_centroid_msg.twist.linear.z = kf_centroid[2]
            kf_centroid_msg.twist.angular.x = kf_centroid[3]
            kf_centroid_msg.twist.angular.y = kf_centroid[4]
            kf_centroid_msg.twist.angular.z = kf_centroid[5]
            self.pub.publish(kf_centroid_msg)
            print("KF centroid: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)" % (kf_centroid[0],
                kf_centroid[1], kf_centroid[2], kf_centroid[3],
                kf_centroid[4], kf_centroid[5]))


    def track_cb(self, msg):
        if True:
            # Append info as dictionary format
            cent = np.array([msg.point.x, msg.point.y, msg.point.z])
            bbox = (np.array([0, 0, 0]), np.array([0, 0, 0]))
            obj_found = {'centroid': cent}
            self.objs_found.append(obj_found)

            # Last object collected --> Run tracking
            self.track_objs()
            self.num_frames += 1
            self.frame_id += 1


if __name__ == '__main__':
    main = Kalman()