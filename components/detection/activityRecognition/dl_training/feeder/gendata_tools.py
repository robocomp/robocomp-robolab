import numpy as np

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

def horizontal_flip(X, valid_frame_num):
    '''
    flip horizontally all data relative to torso, assumes that data is centered at torso
    in_shape = N, C, T, V
    out_shape = in_shape
    '''
    N = X.shape[0]
    X_flipped = np.copy(X)
    for i in range(N):
        for t in range(valid_frame_num[i]):
            X_flipped[i, 0, t, :] *= -1
    return X_flipped


def cut_samples_len(samples, valid_frame_num, new_length, overlap):
    '''
    takes in data and returns sum((valid_frame_num - overlap)//new_length) samples
    in_shape = (N, C, T, V)
    out_shape = (N_new, C, length, V)
    returns cut X data and list which keeps information about in how many samples was each samples cut
    '''
    N, C, _, V = samples.shape
    new_samples_num = [(x - overlap) // (new_length - overlap) for x in valid_frame_num]
    print('num of samples after cutting')
    print(new_samples_num)
    print(f'total samples: {sum(new_samples_num)}')
    new_N = sum(new_samples_num)
    cut_samples = np.zeros((new_N, C, new_length, V))
    count = 0
    for i in range(N):
        # curr shape (C, T, V)
        curr = samples[i, :, :, :]
        frame_counter = 0
        for j in range(new_samples_num[i]):
            cut_samples[count + j, :, :, :] = curr[:, frame_counter : frame_counter + new_length, :]
            frame_counter += new_length - overlap
        count += new_samples_num[i]

    return cut_samples, new_samples_num

def samples_from_cut_sequence(X, new_samples_num, window_size):
    '''
    assumes sample is cut already
    in_shape = (N_cut, C, T_cut, V)
    size is how many frames should remain in the final sample
    if length is 70, size is 32, function will return 2 unique sequences 
    out_shape = (N_cut*T_cut//size, C, T_sampled, V)
    '''
    N, C, T, V = X.shape
    # all T are equal now, as we cut all samples in same length sequences
    freq = T//window_size
    idx = []
    for i in range(freq):
        if (i + 1) % 2 == 0:
            continue
        idx.append([x + i for x in range(window_size*freq) if x%freq == 0])
    
    X_reduced = np.zeros((N*len(idx), C, window_size, V))
    for i in range(N):
        for j in range(len(idx)):
            for k, ind in enumerate(idx[j]):
                X_reduced[i*len(idx) + j, :, k, :] = X[i, :, ind, :]
    new_samples_num = [x * len(idx) for x in new_samples_num]
    print('num samples after sampling')
    print(new_samples_num)
    print(f'total samples: {sum(new_samples_num)}')
    return X_reduced, new_samples_num

def names_labels_for_cut_samples(names, labels, new_samples_num):
    '''
    after getting more samples through cutting and sampling we need to adjust the labels list accordingly
    '''
    new_labels = [None]*sum(new_samples_num)
    new_names = [None]*sum(new_samples_num)
    count = 0
    for i in range(len(labels)):
        new_labels[count : count + new_samples_num[i]] = [labels[i]]*new_samples_num[i]
        new_names[count: count + new_samples_num[i]] = [names[i]]*new_samples_num[i]
        count += new_samples_num[i]
    return new_names, new_labels



