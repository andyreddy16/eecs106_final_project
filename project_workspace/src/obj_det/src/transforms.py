
"""
Calculate transforms between frames
"""

import numpy as np

# KINECT_1_POSE = [2.6, 0.5, 0.9, 0, -0.35, -2.773]
KINECT_2_POSE = [2.6, 1.0, 0.9, 0, -0.35, -2.484479]

KINECT_1_POSE = [0.1, 0, 0.95, 0, -0.3, 0]
# KINECT_2_POSE = [0.83, -1.3, 0.95, 0, -0.3, 1.3]

from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R

def get_transform(T, r, p, y):
	q = quaternion_from_euler(r, p, y)
	r = R.from_quat(q)
	g = np.zeros((4, 4))
	g[:3, :3] = r.as_dcm()
	g[:3, 3] = T.flatten()
	g[3, 3] = 1
	return g

def get_g_world_k1():
	T = np.array(KINECT_1_POSE[:3])
	return get_transform(T, KINECT_1_POSE[3], KINECT_1_POSE[4], KINECT_1_POSE[5])

def get_g_world_k2():
	T = np.array(KINECT_2_POSE[:3])
	return get_transform(T, KINECT_2_POSE[3], KINECT_2_POSE[4], KINECT_2_POSE[5])

if __name__ == "__main__":
	print(get_g_world_k1())
	print(get_g_world_k2())