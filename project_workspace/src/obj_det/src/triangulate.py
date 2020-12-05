#!/usr/bin/env python

"""
Given the centroid of the ball in two frames, find the centroid coordinate in the world frame
"""

# not sure if these are all the imports we need
import numpy as np
import cv2


# not sure if these are all the parameters needed
# feel free to add more params
# (but yeah implement this function pls @ryan)
def triangulate(x1, x2, kinect_frame1, kinect_frame2):
	