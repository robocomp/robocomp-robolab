# code credits https://github.com/huguyuehuhu/HCN-pytorch

import numpy as np
import argparse
import os
import sys
from ntu_read_skeleton import read_xyz
from numpy.lib.format import open_memmap
import pickle

# ntu
training_subjects = [
    1, 2, 4, 5, 8, 9, 13, 14, 15, 16, 17, 18, 19, 25, 27, 28, 31, 34, 35, 38
]
# ntu
training_cameras = [2, 3]
max_body = 2
# ntu
num_joint = 25
# ntu joints that match cad-60 joints
# head, neck, torso, left shoulder, left elbow, right shoulder, right elbow, left hip, left knee, right hip, right knee, left hand, right hand, left foot, right foot 
cad_joints = [3, 2, 1, 4, 5, 8, 9, 12, 13, 16, 17, 7, 11, 15, 19]
# used joints
used_joints = 15
max_frame = 125
toolbar_width = 30


def print_toolbar(rate, annotation=''):
    # setup toolbar
    sys.stdout.write("{}[".format(annotation))
    for i in range(toolbar_width):
        if i * 1.0 / toolbar_width > rate:
            sys.stdout.write(' ')
        else:
            sys.stdout.write('-')
        sys.stdout.flush()
    sys.stdout.write(']\r')


def end_toolbar():
    sys.stdout.write("\n")

def cut_data(data, rem_joints):
    # clips the data to the max_frame or leaves original length if it's shorter
    C, T, V, M = data.shape
    new_data = np.zeros((C, min(max_frame, T), len(rem_joints), M))
    for i in range(len(rem_joints)):
        new_data[:, :, i, :] = data[:, : min(max_frame, T), rem_joints[i], :]
    return new_data    


def gendata(data_path,
            out_path,
            ignored_sample_path=None,
            benchmark='xview',
            part='eval'):

    if ignored_sample_path != None:
        with open(ignored_sample_path, 'r') as f:
            ignored_samples = [
                line.strip() + '.skeleton' for line in f.readlines()
            ]
    else:
        ignored_samples = []

    sample_name = []
    sample_label = []
    for filename in os.listdir(data_path):
        if filename in ignored_samples:
            continue
        action_class = int(
            filename[filename.find('A') + 1:filename.find('A') + 4])
        subject_id = int(
            filename[filename.find('P') + 1:filename.find('P') + 4])
        camera_id = int(
            filename[filename.find('C') + 1:filename.find('C') + 4])

        if benchmark == 'xview':
            istraining = (camera_id in training_cameras)
        elif benchmark == 'xsub':
            istraining = (subject_id in training_subjects)
        else:
            raise ValueError()

        if part == 'train':
            issample = istraining
        elif part == 'val':
            issample = not (istraining)
        else:
            raise ValueError()

        if issample:
            sample_name.append(filename)
            sample_label.append(action_class - 1)

    with open('{}/{}_label.pkl'.format(out_path, part), 'wb') as f:
        pickle.dump((sample_name, list(sample_label)), f)

    fp = open_memmap(
        '{}/{}_data.npy'.format(out_path, part),
        dtype='float32',
        mode='w+',
        shape=(len(sample_label), 3, max_frame, used_joints, max_body))

    fl = open_memmap(
        '{}/{}_num_frame.npy'.format(out_path, part),
        dtype='int',
        mode='w+',
        shape=(len(sample_label),))

    for i, s in enumerate(sample_name):
        print_toolbar(i * 1.0 / len(sample_label),
                      '({:>5}/{:<5}) Processing {:>5}-{:<5} data: '.format(
                          i + 1, len(sample_name), benchmark, part))
        data = read_xyz(
            os.path.join(data_path, s), max_body=max_body, num_joint=num_joint)
        common_data = cut_data(data, cad_joints)
        fp[i, :, 0:common_data.shape[1], :, :] = common_data
        fl[i] = common_data.shape[1]
    end_toolbar()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='NTU-RGB-D Data Converter.')
    parser.add_argument(
        '--data_path', default='../../../ntudataset/nturgb+d_skeletons')
    parser.add_argument(
        '--ignored_sample_path',
        default='../resource/NTU-RGB-D/samples_with_missing_skeletons.txt')
    parser.add_argument('--out_folder', default='../data0/NTU-RGB-D')

    benchmark = ['xsub']
    part = ['train', 'val']
    arg = parser.parse_args()

    for b in benchmark:
        for p in part:
            out_path = os.path.join(arg.out_folder, b)
            if not os.path.exists(out_path):
                os.makedirs(out_path)
            gendata(
                arg.data_path,
                out_path,
                arg.ignored_sample_path,
                benchmark=b,
                part=p)
