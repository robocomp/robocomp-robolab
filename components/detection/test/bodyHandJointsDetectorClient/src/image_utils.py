import cv2
import numpy as np


body_edges = np.array(
    [[0, 1],  # neck - nose
     [1, 16], [16, 18],  # nose - l_eye - l_ear
     [1, 15], [15, 17],  # nose - r_eye - r_ear
     [0, 3], [3, 4], [4, 5],     # neck - l_shoulder - l_elbow - l_wrist
     [0, 9], [9, 10], [10, 11],  # neck - r_shoulder - r_elbow - r_wrist
     [0, 6], [6, 7], [7, 8],        # neck - l_hip - l_knee - l_ankle
     [0, 12], [12, 13], [13, 14]])  # neck - r_hip - r_knee - r_ankle


def draw_pose(img, points):
    # draw only body pose
    # points = points[:19,:]
    for edge in body_edges:
        if (points[edge[0],0] >10 and points[edge[0],1] > 10) and \
                (points[edge[1],0] > 10 and points[edge[1],1] >= 10):
            cv2.line(img, (int(points[edge[0],0]), int(points[edge[0],1])),
                          (int(points[edge[1],0]), int(points[edge[1],1])),
                     (255, 255, 0), 2, cv2.LINE_AA)

    for ind in range(points.shape[0]):
        if points[ind,0] != 0 and points[ind,1] != 0:
            if ind < 19:
                cv2.circle(img, (int(points[ind,0]), int(points[ind,1])), 2, (0, 255, 255), -1, cv2.LINE_AA)
            else:
                cv2.circle(img, (int(points[ind, 0]), int(points[ind, 1])), 4, (0, 0, 255), -1, cv2.LINE_AA)
