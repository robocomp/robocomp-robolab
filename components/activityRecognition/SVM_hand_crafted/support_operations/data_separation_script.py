import numpy as np
import os
from glob import glob
from shutil import copyfile
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--dataset_dir', default='../../cad60dataset/', help="directory where original data exists")
parser.add_argument('--separated_dataset_dir', default='../../cad60_separated/', help="directory where original data exists")


if __name__ == '__main__':

    args = parser.parse_args()

    cad60_path = args.dataset_dir
    cad60_separated_path = args.separated_dataset_dir

    data_folders = sorted([x for x in os.listdir(cad60_path) if 'data' in x and 'zip' not in x])

    labels_path = list(map(lambda x: os.path.join(cad60_path, x, 'activityLabel.txt'), data_folders))

    # write all labels to one textfile and copy it into cad60_separated_path
    with open(os.path.join(cad60_separated_path, 'total_labels.txt'), 'w') as out:
        for i in range(len(labels_path)):
            with open(labels_path[i], 'r') as f:
                for line in f:
                    if 'END' not in line:
                        out.write(line)
    copyfile('total_labels.txt', os.path.join(cad60_separated_path, 'total_labels.txt'))

    # labels related to each environment
    bathroom = ['rinsing mouth with water', 'brushing teeth', 'wearing contact lenses']
    bedroom = ['talking on the phone', 'drinking water', 'opening pill container']
    kitchen = ['cooking (chopping)', 'cooking (stirring)', 'drinking water', 'opening pill container']
    livingroom = ['talking on the phone', 'drinking water', 'talking on couch', 'relaxing on couch']
    office = ['talking on the phone', 'writing on whiteboard', 'drinking water', 'working on computer']

    envs = [bathroom, bedroom, kitchen, livingroom, office]

    samples_ba = []
    samples_be = []
    samples_ki = []
    samples_li = []
    samples_of = []
    samples_all_envs = [samples_ba, samples_be, samples_ki, samples_li, samples_of]

    with open('total_labels.txt', 'r') as f:
        total_samples = f.read().split('\n')
    total_samples = total_samples[:-1]

    # dividing all samples names in different lists
    for s in total_samples:
        name = s.split(',')[0]
        label = s.split(',')[1]
        for i in range(len(envs)):
            if label in envs[i]:
                samples_all_envs[i].append(name)

    env_names = ['bathroom', 'bedroom', 'kitchen', 'livingroom', 'office']

    # distribute the files from cad60_path to cad60_separated_path
    for i in range(len(env_names)):
        target_path = os.path.join(cad60_separated_path, env_names[i])
        os.makedirs(target_path, exist_ok=True)
        for d in data_folders:
            subject = d
            target_subject_path = os.path.join(target_path, d)
            os.makedirs(target_subject_path, exist_ok=True)
            source_dir = os.path.join(cad60_path, d)
            files_in_dir = glob(os.path.join(source_dir, '*.txt'))
            for f in range(len(files_in_dir)):
                name = files_in_dir[f].split('/')[-1].split('.')[0]
                if name in samples_all_envs[i]:
                    copyfile(files_in_dir[f], os.path.join(target_subject_path, (name + '.txt')))