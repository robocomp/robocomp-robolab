import numpy as np
import pickle
import os

_NUM_FEATURES = 16
# this file contains values to normalize the test sample obtained during training
_NORMAL_DENOMS = os.path.join(os.path.dirname(__file__), 'data/normalization_denoms.pkl')


def extract_features(sample):
    '''
    extracts features for 4 frames. _NUM_FEATURES features for each frame.
    '''
    x = center(sample)
    T = x.shape[1]
    features = np.zeros((_NUM_FEATURES, T))
    
    # euclidian distance from each hand to the torso
    features[0, :] = dist_to_joint(x[:, :, 11], x[:, :, 2])
    features[1, :] = dist_to_joint(x[:, :, 12], x[:, :, 2])
    # dist between elbows and torso
    features[2, :] = dist_to_joint(x[:, :, 4], x[:, :, 2])
    features[3, :] = dist_to_joint(x[:, :, 6], x[:, :, 2])
    # dist between two hands 11, 12 
    features[4, :] = dist_to_joint(x[:, :, 11], x[:, :, 12])
    # dist between head and torso
    features[5, :] = dist_to_joint(x[:, :, 0], x[:, :, 2])
    # dist between shoulders and feet 3, 5, 13, 14 
    features[6, :] = dist_to_joint(
        x[:, :, 3], x[:, :, 13]) + dist_to_joint(
        x[:, :, 5], x[:, :, 14])
    # knees to torso
    features[7, :] = dist_to_joint(x[:, :, 8], x[:, :, 2]) + dist_to_joint(
        x[:, :, 10], x[:, :, 2])

    # temporal positional features
    # left hand 11 x, y
    features[8, :] = diff_position_x(x[:, :, 11])
    features[9, :] = diff_position_y(x[:, :, 11])
    # right hand 12
    features[10, :] = diff_position_x(x[:, :, 12])
    features[11, :] = diff_position_y(x[:, :, 12])
    # head 0
    features[12, :] = diff_position_x(x[:, :, 0])
    features[13, :] = diff_position_y(x[:, :, 0])
    # turn of the body to understand if we are viewing from the side
    features[14, :] = body_turn(x[:, :, :])
    # dist hands to knees
    features[15, :] = dist_to_joint(x[:, :, 11], x[:, :, 8]) + dist_to_joint(
        x[:, :, 12], x[:, :, 10])    

    features = np.around(features, decimals=4)
    flat_features = features.flatten()
    final_features = normalize(flat_features)
    return final_features



def center(x):
    '''
    move origin to torso, joint 2
    '''
    C, T, V = x.shape
    for t in range(T):
        torso_coord = x[:, t, 2]
        for v in range(V):
            x[:, t, v] -= torso_coord
    return x


def dist_to_joint(joint1, joint2):
    '''
    in_shape = (3, T) for both
    out_shape = (T)
    '''
    _, T = joint1.shape
    dist_data = np.zeros(T)
    for t in range(T):
        dist_data[t] = np.linalg.norm(joint1[:, t] - joint2[:, t])
    return dist_data

def diff_position_x(x):
    '''
    in_shape of joint (3, T)
    out_shape (T)
    '''
    _, T = x.shape
    out = np.zeros(T)
    init_x = x[0, 0]
    for t in range(T):
        out[t] = init_x - x[0, t]
        # prevent zero values, as in the end we get that for all first frames we have 0 and it will result in NaN when normalizing
        if out[t] < abs(1e-3):
            out[t] = 1e-3
    return out

def diff_position_y(x):
    '''
    in_shape of joint (3, T)
    out_shape (T)
    '''
    _, T = x.shape
    out = np.zeros(T)
    init_y = x[1, 0]
    for t in range(T):
        out[t] = init_y - x[1, t]
        if out[t] < abs(1e-3):
            out[t] = 1e-3
    return out

def body_turn(x):
    '''
    calculates the body turn by calculating the distance between the left and the rights shoulders based on 2d coordinates and divides by the
    distance between the same joints based on the 3d coordinates
    in_shape = (C, T, V)
    out_shape = (T)
    '''
    C, T, V = x.shape
    out = np.zeros(T)
    for t in range(T):
        l_should_2d = x[0:2, t, 3]
        r_should_2d = x[0:2, t, 5]
        dist_2d = dist_to_joint_single(l_should_2d, r_should_2d)
        out[t] = dist_2d/dist_to_joint_single(x[:, t, 3], x[:, t, 5]) 
    return out

def dist_to_joint_single(joint1, joint2):
    '''
    dist_to_torso for a single frame
    '''
    return np.linalg.norm(joint1 - joint2)

def flatten(features):
    '''
    this method is for features of shape = (N, feat, T) to transform them into
    shape (N, feat*T) by means of horizontal stacking
    '''
    N, feat, T = features.shape
    new_features = np.zeros((N, feat * T))
    for i in range(N):
        new_features[i, :] = np.hstack(features[i, :, :])
    return new_features


def normalize(features):
    '''
    we will normalize using the coefficients obtained during training to have the similar features for the test sample
    '''
    with open(_NORMAL_DENOMS, 'rb') as f:
        denominators = pickle.load(f)
    F = features.shape[0]
    # if sampling frequency and number of features have not been changed, the F should be the same as the len of the denominators
    assert(len(denominators) == F)
    for j in range(F):
        features[j] = features[j]/denominators[j]
    return features
