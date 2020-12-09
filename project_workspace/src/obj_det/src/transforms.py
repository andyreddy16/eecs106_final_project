
"""
Calculate transforms between frames
"""

import numpy as np
from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R

# for point_cloud_bounce_two_kinects.bag
# and point_cloud_bounce_three_kinects.bag
KINECT_POSES = [
	[0.1, 0, 0.95, 0, -0.3, 0],
	[2.6, 0.5, 0.9, 0, -0.35, -2.773],
	[1.0, -1.0, 3.0, 0, 1.0, 1.2]
]

def get_transform(angles, T):
	r = angles[0]
	p = angles[1]
	y = angles[2]
	q = quaternion_from_euler(r, p, y)
	rot = R.from_quat(q)
	g = np.zeros((4, 4))
	g[:3, :3] = rot.as_dcm()
	g[:3, 3] = T.flatten()
	g[3, 3] = 1
	return g

def get_g_0k(k):
	angles = np.array(KINECT_POSES[k][3:])
	T = np.array(KINECT_POSES[k][:3])
	return get_transform(angles, T)

if __name__ == "__main__":
	print(get_g_0k(0))
	print(get_g_0k(1))