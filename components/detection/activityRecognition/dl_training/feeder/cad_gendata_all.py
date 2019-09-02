import numpy as np
import argparse
import os
import sys
from numpy.lib.format import open_memmap
import pickle
import glob
from gendata_tools import *
import sys
# we will reuse the cad_read_skeleton file from SVM training here
sys.path.append('../../SVM_hand_crafted/feeder')
from cad_read_skeleton import read_xyz

_subjects = 4
_max_body = 1
_num_joint = 15
# for original data
_max_frame = 2000
_toolbar_width = 30
# length for cut sequnces
_cut_frames = 100
_overlap = 50
# hyperparam of the NN, should agree with params.json of NN training if used
_window_size = 32


# labels in the dataset are strings, we need to convert them to numbers
activities = {
    'talking on the phone' : 0,
    'writing on whiteboard': 1,
    'drinking water': 2,
    'rinsing mouth with water' : 3, 
    'brushing teeth' : 4,
    'wearing contact lenses' : 5,
    'talking on couch' : 6,
    'relaxing on couch' : 7,
    'cooking (chopping)' : 8,
    'cooking (stirring)' : 9,
    'opening pill container' : 10,
    'working on computer' : 11,
}


def print_toolbar(rate, annotation=''):
    # setup toolbar
    sys.stdout.write("{}[".format(annotation))
    for i in range(_toolbar_width):
        if i * 1.0 / _toolbar_width > rate:
            sys.stdout.write(' ')
        else:
            sys.stdout.write('-')
        sys.stdout.flush()
    sys.stdout.write(']\r')


def end_toolbar():
    sys.stdout.write("\n")



def gendata(data_path,
            out_path,
            training_subjects,
            part,
            ignored_sample_path=None
            ):

    # no label or video for four samples and we do not count still and random samples
    ignored_samples = ['0512164333', '0510171947', '0511125626', '0512160049',
     '0512172825', '0512174930', '0510160858', '0510165514', '0511121410', '0511125200', '0512150222', '0512154740']
    
    sample_name = []
    sample_label = []
    sample_data = []
    valid_frame_num = []

    os.makedirs(out_path, exist_ok=True)
    print(data_path)
    print(out_path)

    for s in range(_subjects):

        subject = '/data{0}'.format(s + 1)
        data_path_s = data_path + subject

        for filename in glob.glob(data_path_s + '/0*.txt'):
            base = os.path.basename(filename)
            sample_path = subject + '/' + base
            sample_id = base.split('.')[0]

            if sample_id in ignored_samples:
                continue

            action_label = -1 
            with open('../../support_operations/total_labels.txt', 'r') as f:
                num_lines = len(f.readlines())
                f.seek(0)
                for i in range(num_lines):
                    line = f.readline() 
                    cont = line.split(',')
                    if sample_id == cont[0]:
                        action_label = activities[cont[1]]
                if action_label == -1:
                    raise ValueError('cannot find activity label for a sample')

            istraining = (s + 1 in training_subjects)

            # depending in training or validation, issample will be true or false for this particular file
            if part == 'train':
                issample = istraining
            elif part == 'val':
                issample = not (istraining)
            else:
                raise ValueError()

            # if issample is true, append the name to the sample_name list and its label to sample_label
            if issample:
                sample_name.append(sample_path)
                sample_label.append(action_label)
                # xyz, frame, joint
                xyz_data = read_xyz((data_path + sample_path), max_body=_max_body, num_joint=_num_joint)
                sample_data.append(xyz_data)
                valid_frame_num.append(xyz_data.shape[1])
    # put data into numpy array
    data = np.zeros((len(sample_data), 3, _max_frame, _num_joint))
    for i in range(len(sample_data)):
        data[i, :, : valid_frame_num[i], :] = sample_data[i]
    # change to meters
    data = data/1000
    print('original frame num')
    print(valid_frame_num)
    print(f'total samples: {len(valid_frame_num)}')

    # below code is for augmentation by flipping which increases the number of training samples
    # do not flip on this level, flipping will be done randomly during training in the feeder
    # if part == 'train':
    #     flipped_data = horizontal_flip(data, valid_frame_num)
    #     total_data = np.zeros((data.shape[0]*2, 3, _max_frame, _num_joint))
    #     total_data[: data.shape[0], :, :, :] = data
    #     total_data[data.shape[0]:, :, :, :] = flipped_data
    #     data = total_data
    #     valid_frame_num = valid_frame_num * 2
    #     sample_name = sample_name*2
    #     sample_label = sample_label*2
    samples, new_samples_num = cut_samples_len(data, valid_frame_num, _cut_frames, _overlap)
    samples, new_samples_num = samples_from_cut_sequence(samples, new_samples_num, _window_size)
    names, labels = names_labels_for_cut_samples(samples, sample_label, new_samples_num)
    valid_frame_num = [_window_size] * len(names)
    save_data(part, out_path, labels, names, samples, valid_frame_num)
    print(f'training on: {training_subjects}')
    print(part)
    print(f'data shape: {samples.shape}')
    print(f'labels len: {len(labels)}')
    
def save_data(part, out_path, sample_label, sample_name, sample_data, valid_frame_num):

    with open('{}/{}_label.pkl'.format(out_path, part), 'wb') as f:
        pickle.dump((sample_name, list(sample_label)), f)

    # model will be pre-trained on NTU RGB-D which was trained with 2 possible skeletons, in cad we only have 1 body at a time, but want to have the
    # same data dimensions
    fp = open_memmap(
        '{}/{}_data.npy'.format(out_path, part),
        dtype='float32',
        mode='w+',
        shape=(len(sample_label), 3, _window_size, _num_joint, 2))

    # num of frames of every sample stored here, every sample has equal length now
    fl = open_memmap(
        '{}/{}_num_frame.npy'.format(out_path, part),
        dtype='int',
        mode='w+',
        shape=(len(sample_label),))

    for i, s in enumerate(sample_name):
        print_toolbar(i * 1.0 / len(sample_label),
                      '({:>5}/{:<5}) Processing {:<5} data: '.format(
                          i + 1, len(sample_name), part))
        fp[i, :, :, :, 0] = sample_data[i, :, :, :]

        fl[i] = _cut_frames # num_frame
    end_toolbar()



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='CAD-60 Data Converter.')
    parser.add_argument(
        '--data_path', default='../../../cad60dataset')
    parser.add_argument('--out_folder', default='../data0/CAD-60/all')

    part = ['train', 'val']

    subjects = [1, 2, 3, 4]

    arg = parser.parse_args()

    # we will create separate folder for each fold, where 1 subject is validation and the others are in the training set
    for s in subjects:
        training_subjects = [x for x in subjects if x != s]
        out_path = os.path.join(arg.out_folder, str(s))
        if not os.path.exists(out_path):
            os.makedirs(out_path)
        for p in part:    
            gendata(
                arg.data_path,
                out_path,
                training_subjects,
                part=p)
