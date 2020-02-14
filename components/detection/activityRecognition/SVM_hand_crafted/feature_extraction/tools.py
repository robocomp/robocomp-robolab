import numpy as np
import pickle


def normalize(X, valid_frame_num):
    '''
    normalize for each feature across all frames, separately for each sample
    in_shape = (N, 14, n_frames)
    '''
    N, feat, T  = X.shape
    out = np.zeros((N, feat, T))
    for i in range(N):
        for f in range(feat):
            denom = np.max(X[i, f, :]) - np.min(X[i, f, :])
            out[i, f, :] = X[i, f, :] * (1/denom)
    return out

def normalize_allsamples(X):
    '''
    normalize for each feature across all samples
    in_shape = (N, num_fatures)
    out_shape = in_shape
    '''
    N, feat = X.shape
    out = np.zeros((N, feat))
    denoms = []
    for i in range(feat):
        denom = np.max(X[:, i]) - np.min(X[:, i])
        if denom == 0:
            denom = X.shape[0]
            out[:, i] = X[:, i] / denom
        else:    
            out[:, i] = X[:, i] * (1/denom)
        denoms.append(denom)
    # with open('normalization_denoms.pkl', 'wb') as f:
    #     pickle.dump(denoms, f)
    return out

def normalize_allsamples_byjoint(X):
    '''
    normalize for each feature across all samples
    in_shape = (N)
    out_shape = in_shape
    '''
    N = X.shape
    out = np.zeros(N)
    denom = np.max(X) - np.min(X)
    out[:] = X[:] * (1/denom)
    return out

def normalize_by_height(feature, X, valid_frame_num):
    '''
    normalize a (distance) feature by the "height" of a person in the frame, where height is calc. as dist(torso, neck)*3
    feature.shape = (N, n_frame)
    out_shape = feature.shape
    '''
    N, C, T, V = X.shape
    normed_feature = np.zeros((feature.shape))
    for i in range(N):
        for t in range(valid_frame_num[i]):
            torso_neck = dist_to_joint_single(X[i, :, t, 2], X[i, :, t, 1])
            height = torso_neck * 3
            normed_feature[i, t] = feature[i, t] / height
    return normed_feature

def center(X, valid_frame_num):
    '''
    origin is moved to torso coordinates for each frame.
    '''
    # c - xyz, t - frames, v - join
    N, C, _, V = X.shape
    for i in range(N):
        for t in range(valid_frame_num[i]):
            # torso is the 3rd joint
            torso_coord = X[i, :, t, 2]
            for v in range(V):
                X[i, :, t, v] -= torso_coord
    return X


def dist_to_joint(joint1, joint2, valid_frame_num1, valid_frame_num2):
    '''
    in_shape = (3, n_frames) for both
    out_shape = (n_frames)
    '''
    assert(valid_frame_num1 == valid_frame_num2)
    dist_data = np.zeros(valid_frame_num1)
    for t in range(valid_frame_num1):
        dist_data[t] = np.linalg.norm(joint1[:, t] - joint2[:, t])
    return dist_data


def dist_to_joint_allsamples(joint1, joint2, valid_frame_num):
    '''
    in_shape = (N, 3, n_frames) for both
    out_shape = (N, n_frames)
    '''
    N, _, T = joint1.shape
    dist_data = np.zeros((N, T))
    for i in range(N):
        for t in range(valid_frame_num[i]):
            dist_data[i, t] = np.linalg.norm(joint1[i, :, t] - joint2[i, :, t])
    return dist_data

def dist_to_joint_single(joint1, joint2):
    '''
    dist_to_torso for single frame
    '''
    return np.linalg.norm(joint1 - joint2)


def horizontal_flip(X, valid_frame_num):
    '''
    flip horizontally all data relative to torso, assumes that data is centered at torso
    in_shape = N, C, T, V
    out_shape = in_shape
    '''
    N, _, _, _ = X.shape
    X_flipped = np.copy(X)
    for i in range(N):
        for t in range(valid_frame_num[i]):
            X_flipped[i, 0, t, :] *= -1
    return X_flipped


def diff_position_x(X, num_frames):
    '''
    in_shape of joint (N, 3, n_frames)
    out_shape (N, n_frames)
    '''
    N, _, T = X.shape
    out = np.zeros((N, T))
    for i in range(N):
        init_x = X[i, 0, 0]
        for t in range(num_frames[i]):
            out[i, t] = init_x - X[i, 0, t]
            if out[i, t] < abs(1e-3):
                out[i, t] = 1e-3
    return out

def diff_position_y(X, num_frames):
    '''
    in_shape of joint (N, 3, n_frames)
    out_shape (N, n_frames)
    '''
    N, _, T = X.shape
    out = np.zeros((N, T))
    for i in range(N):
        init_y = X[i, 1, 0]
        for t in range(num_frames[i]):
            out[i, t] = init_y - X[i, 1, t]
            if out[i, t] < abs(1e-3):
                out[i, t] = 1e-3
    return out


def frame_by_frame_samples(X, valid_frame_num):
    '''
    turns every frame into a sample
    in_shape (n_samples, features, n_frames)
    out_shape (sum(valid_frame_num), features)
    '''
    N, F, _ = X.shape
    total_samples = sum(valid_frame_num)
    X_new = np.zeros((total_samples, F))
    prev = 0
    for i in range(N):
        for j in range(valid_frame_num[i]):
            X_new[prev + j] = X[i, :, j]
        prev += valid_frame_num[i]
    return X_new

def frame_by_frame_labels(labels, valid_frame_num):
    '''
    produces labels for each frame
    in-len = n_samples
    out_len = sum(valid_frame_num)
    '''
    new_labels = []
    for i in range(len(labels)):
        for j in range(valid_frame_num[i]):
            new_labels.append(labels[i])
    return new_labels


def fbf_raw_data(X, valid_frame_num):
    '''
    turns every frame into a sample, but keeps the raw data xyz for each joint, i.e.
    turns four-dim array into three dim-array
    in_shape = (N, C, T, V)
    out_shape = (sum(num_frames), C, V)
    '''
    N, C, T, V = X.shape
    data = np.zeros((sum(valid_frame_num), C, V))
    counter = 0
    for i, frames in enumerate(valid_frame_num):
        # shape of seq = (C, T, V)
        seq = X[i]
        for j in range(frames):
            data[counter, :, :] = seq[:, j, :]
            counter +=1
    return data


def cut_samples(X, valid_frame_num, new_length):
    '''
    takes in data and returns sum(valid_frame_num/new_length) samples
    in_shape = (N, C, T, V)
    out_shape = (sum(valid_frame_num//length), C, length, V)
    returns cut X data and list which keeps information about in how many samples was each samples cut
    '''
    N, C, T, V = X.shape
    new_samples_num = [x // new_length for x in valid_frame_num]
    new_N = sum(new_samples_num)
    cut_samples = np.zeros((new_N, C, new_length, V))
    count = 0
    for i in range(N):
        # curr shape (C, T, V)
        curr = X[i, :, :, :]
        for j in range(new_samples_num[i]):
            cut_samples[count + j, :, :, :] = curr[:, j*new_length : j*new_length + new_length, :]
        count += new_samples_num[i]

    return cut_samples, new_samples_num

def sample_from_cut_sequence(X, new_samples_num, freq=5):
    '''
    assumes sample is cut already
    in_shape = (N_cut, C, T_cut, V)
    samples every 'freq'th frame from the sequence goes over the same sequence several times shifting by 1 frame
    example: if length is 70, and freq = 5, every sequence will sample 5 smaller sequences of length 14
    out_shape = (N_cut*T_cut//freq, C, T_sampled, V)
    '''
    N, C, T, V = X.shape
    # all T are equal now, as we cut all samples in same length sequences
    idx = []
    for i in range(freq):
        idx.append([x + i for x in range(T//freq*freq) if x%freq == 0])
    
    X_reduced = np.zeros((N*freq, C, T//freq, V))
    for i in range(N):
        for j in range(freq):
            for k, ind in enumerate(idx[j]):
                X_reduced[i*freq + j, :, k, :] = X[i, :, ind, :]
    new_samples_num = [x * freq for x in new_samples_num]
    return X_reduced, new_samples_num

def labels_for_cut_samples(labels, new_samples_num):
    '''
    after getting more samples through cutting and sampling we need to adjust the labels list accodringly
    '''
    new_labels = [None]*sum(new_samples_num)
    helper = 0
    for i, l in enumerate(labels):
        new_labels[helper : helper + new_samples_num[i]] = [l]*new_samples_num[i]
        helper += new_samples_num[i]
    return new_labels

def flatten(features):
    '''
    this method if for features of shape = (N, feat, T) to transform them into
    shape (N, feat*T) by means of horizontal stacking
    '''
    N, feat, T = features.shape
    new_features = np.zeros((N, feat * T))
    for i in range(N):
        new_features[i, :] = np.hstack(features[i, :, :])
    return new_features




