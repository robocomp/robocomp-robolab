import numpy as np
import argparse
import os
import sys
from cad_read_skeleton import read_xyz
from numpy.lib.format import open_memmap
import pickle
import glob

subjects = 4
max_body = 1
num_joint = 15
max_frame = 2000
toolbar_width = 30

# all of them are in the training set, since we will be doing LOOCV
training_subjects = [1, 2, 3, 4]

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
    for i in range(toolbar_width):
        if i * 1.0 / toolbar_width > rate:
            sys.stdout.write(' ')
        else:
            sys.stdout.write('-')
        sys.stdout.flush()
    sys.stdout.write(']\r')


def end_toolbar():
    sys.stdout.write("\n")



def gendata(data_path,
            out_path,
            separated,
            part,
            env,
            ignored_sample_path=None,
            ):

    # no label or video for four samples and we do not count still and random samples
    ignored_samples = ['0512164333', '0510171947', '0511125626', '0512160049',
     '0512172825', '0512174930', '0510160858', '0510165514', '0511121410', '0511125200', '0512150222', '0512154740']
    
    sample_name = []
    sample_label = []
    sample_data = []

    if separated:
        data_path = os.path.join(data_path, e)
    out_path = os.path.join(out_path, e)

    if not os.path.exists(out_path):
        os.makedirs(out_path)
    print(data_path)
    print(out_path)

    for s in range(subjects):

        subject = '/data{0}'.format(s + 1)
        data_path_s = data_path + subject

        for filename in glob.glob(data_path_s + '/0*.txt'):
            base = os.path.basename(filename)
            sample_path = subject + '/' + base
            sample_id = base.split('.')[0]

            if sample_id in ignored_samples:
                continue

            action_label = -1 
            with open('../support_operations/total_labels.txt', 'r') as f:
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
                sample_data.append(read_xyz((data_path + sample_path), max_body=max_body, num_joint=num_joint))

    with open('{}/{}_label.pkl'.format(out_path, part), 'wb') as f:
        pickle.dump((sample_name, list(sample_label)), f)

    # https://docs.scipy.org/doc/numpy/reference/generated/numpy.memmap.html
    # https://blog.csdn.net/u014630431/article/details/72844501
    print(env)
    print(len(sample_name))
    # in fp the data itself is stored
    fp = open_memmap(
        '{}/{}_data.npy'.format(out_path, part),
        dtype='float32',
        mode='w+',
        shape=(len(sample_label), 3, max_frame, num_joint))

    # num of frames of every sample stored here
    fl = open_memmap(
        '{}/{}_num_frame.npy'.format(out_path, part),
        dtype='int',
        mode='w+',
        shape=(len(sample_label),))

    for i, s in enumerate(sample_name):
        print_toolbar(i * 1.0 / len(sample_label),
                      '({:>5}/{:<5}) Processing {:<5} data: '.format(
                          i + 1, len(sample_name), part))
        fp[i, :, 0:sample_data[i].shape[1], :] = sample_data[i]

        fl[i] = sample_data[i].shape[1] # num_frame
    end_toolbar()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='CAD-60 Data Converter.')
    # parser.add_argument(
        # '--data_path', default='../../cad60_separated')
    parser.add_argument('--data_path', default='../../cad60dataset')
    parser.add_argument('--out_folder', default='../data0/CAD-60')
    parser.add_argument('--envs', default='all', help='all or separated')

    # everything is train data, because we will be doing cross-validation, can extend this for train-test split simply by adding 'test' to the list
    part = ['train']

    arg = parser.parse_args()

    separated = False

    if arg.envs == 'separated':
        environments = ['bathroom', 'bedroom', 'kitchen', 'livingroom', 'office']
        separated = True;
    elif arg.envs == 'all':
        environments = ['all']
    else:
        raise ValueError('unsupported option, choose either all or separated')

    for p in part:
        for e in environments:
            out_path = arg.out_folder
            if not os.path.exists(out_path):
                os.makedirs(out_path)
            gendata(
                arg.data_path,
                out_path,
                separated, 
                part=p,
                env=e)
