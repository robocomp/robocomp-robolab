# import tools
from feature_extraction.tools import *


def body_turn(X, valid_frame_num):
    '''
    calculates the body turn by calculating the distance between the left and the rights shoulder based on 2d coordinates and divides by the
    distance between the same joints based on 3d coordinates
    in_shape = (N, C, T, V)
    out_shape = (N, T)
    '''
    N, C, T, V = X.shape
    out = np.zeros((N, T))
    for i in range(N):
      for t in range(valid_frame_num[i]):
          l_should_2d = X[i, 0:2, t, 3]
          r_should_2d = X[i, 0:2, t, 5]
          dist_2d = dist_to_joint_single(l_should_2d, r_should_2d)
          out[i, t] = dist_2d/dist_to_joint_single(X[i, :, t, 3], X[i, :, t, 5]) 
    return out

def body_incline(X, valid_frame_num):
    '''
    calculates the relation between the distance from mid of shoulders to mid of hips to distance from neck to torso and from torso to mid of hips
    in_shape = (N, C, T, V)
    out_shape = (N, T)
    '''
    N, C, T, V = X.shape
    out = np.zeros((N, T))
    for i in range(N):
        for t in range(valid_frame_num[i]):
            mid_shoulders = np.array([(X[i, 0, t, 5] - X[i, 0, t, 3]) / 2, (X[i, 1, t, 5] - X[i, 1, t, 3]) / 2, (X[i, 2, t, 5] - X[i, 2, t, 3]) / 2])
            mid_hip = np.array([(X[i, 0, t, 9] - X[i, 0, t, 7]) / 2, (X[i, 1, t, 9] - X[i, 1, t, 7]) / 2, (X[i, 2, t, 9] - X[i, 2, t, 7]) / 2])
            dist_should_hip = dist_to_joint_single(mid_shoulders, mid_hip)
            denom = dist_to_joint_single(X[i, :, t, 1], X[i, :, t, 2]) + dist_to_joint_single(X[i, :, t, 2], mid_hip)
            out[i, t] = dist_should_hip/denom
    return out   

def head_tilt(X, valid_frame_num):
    '''calculates the head tilt as the relation between the distance from the head to the torso to distance from head to neck + neck to torso
    in_shape = (N, C, T, V)
    out_shape = (N, T)
    '''
    N, C, T, V = X.shape
    out = np.zeros((N, T))
    for i in range(N):
        for t in range(valid_frame_num[i]):
            head_torso = dist_to_joint_single(X[i, :, t, 0], X[i, :, t, 2])
            head_neck = dist_to_joint_single(X[i, :, t, 0], X[i, :, t, 1])
            neck_torso = dist_to_joint_single(X[i, :, t, 1], X[i, :, t, 2])
            out[i, t] = head_torso/(head_neck + neck_torso) 
    return out

def knee_bend(X, valid_frame_num):
    '''calculates the knee bend as the relation between the distance from the hips to the feet to distance from the hips to the knees + from the knees to the feet
    in_shape = (N, C, T, V)
    out_shape = (N, T)
    '''
    N, C, T, V = X.shape
    out = np.zeros((N, T))
    for i in range(N):
        for t in range(valid_frame_num[i]):
            left_leg = dist_to_joint_single(X[i, :, t, 7], X[i, :, t, 8]) + dist_to_joint_single(X[i, :, t, 8], X[i, :, t, 13])
            right_leg = dist_to_joint_single(X[i, :, t, 9], X[i, :, t, 10]) + dist_to_joint_single(X[i, :, t, 10], X[i, :, t, 14])
            left_hip_foot = dist_to_joint_single(X[i, :, t, 7], X[i, :, t, 13])
            right_hip_foot = dist_to_joint_single(X[i, :, t, 9], X[i, :, t, 14])
            out[i, t] = (left_hip_foot + right_hip_foot)/(left_leg + right_leg) 
    return out