"""
Returns matched features b/w two images w/ epipolar constraints
"""


import cv2
import numpy as np

def epipolar_error(x1, x2, l1, l2):
    """
       Task 2
       ------
       Computes the error of the candidate match (x1, x2), given in *normalized* image
       homogeneous coordinates. l1 and l2 are the respective epipolar lines.
       
       x1: np.array of size (3,): (u1, v1, 1)
       x2: np.array of size (3,): (u2, v2, 1)
       l1: np.array of size (3,): (a1, b1, c1)
       l2: np.array of size (3,): (a2, b2, c2)
    """ 
    # calculate the distance between the line l1 and x1.

    dist = lambda point, line: np.abs(point[0]*line[0] + point[1]*line[1] + point[2]*line[2]) / np.sqrt(line[0]**2 + line[1]**2)

    d1 = dist(x1, l1)
    d2 = dist(x2, l2)

    # compute the total error.
    error = d1 + d2

    return error


def filter_by_epipolar(intrinsics1, intrinsics2, points1, points2, R, T, threshold, matches):
    """
        Task 2
        ------
        Returns an array inlier_mask of length equal to the length of matches. inlier_mask[i] is 1
        if matches[i] satisfies the epipolar constraint (i.e. the error is less than the threshold.
        Otherwise, inlier_mask[i] = 0.
        
        intrinsics1: np.array of size (3, 3): intrinsic camera matrix of left camera.
        intrinsics2: np.array of size (3, 3): intrinsic camera matrix of right camera.
        points1: np.array of size (M, 3): homogeneous, unnormalized coordinates of keypoints in left image.
        points2: np.array of size (N, 3): homogeneous, unnormalized coordinates of keypoints in right image.
        matches: list of cv2.Match objects. matches[i].queryIdx is the index in points1 of the first keypoint
                 in the i-th match. matches[i].trainIdx is the index in points2 of the second keypoint in the
                 i-th match.
    """
    # Delete this return statement when you implement this function.
    # return np.ones(len(matches)).astype(np.int32)

    # Compute Essential matrix
    T_hat = np.zeros((3, 3))
    T_hat[0, 1] = -T[2]
    T_hat[0, 2] = T[1]
    T_hat[1, 0] = T[2]
    T_hat[1, 2] = -T[1]
    T_hat[2, 0] = -T[1]
    T_hat[2, 1] = T[1]

    E = np.dot(T_hat, R)

    inlier_mask = []

    intrinsics1_inv = np.linalg.inv(intrinsics1)
    intrinsics2_inv = np.linalg.inv(intrinsics2)
    
    for i in matches:	
        u_v1 = points1[i.queryIdx]
        u_v2 = points2[i.trainIdx]

        (u1,v1) = u_v1.pt
        (u2,v2) = u_v2.pt  
            
        # normalize x1 and x2
        x1 = np.dot(intrinsics1_inv, np.array((u1, v1, 1)))
        x2 = np.dot(intrinsics2_inv, np.array((u2, v2, 1)))

	   # compute epilines l1, l2.
        l1 = np.dot(E.T, x2)
        l2 = np.dot(E, x1)

        error = epipolar_error(x1, x2, l1, l2)
        m = (error < threshold).astype(int)	        
        inlier_mask.append(m)
    
    return np.array(inlier_mask)