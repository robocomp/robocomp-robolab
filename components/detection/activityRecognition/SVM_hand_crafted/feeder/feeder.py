# sys
import pickle

import numpy as np
import os

class Dataset():
    """ Dataset for skeleton-based action recognition
    Arguments:
        data_path: the path to '.npy' data, the shape of data should be (N, C, T, V)
        label_path: the path to label
        center: If true, center at torso
    """

    def __init__(self,
                 data_path,
                 label_path,
                 num_frame_path,
                 mmap = False,
                 max_body = 1
                 ):
        self.data_path = data_path
        self.label_path = label_path
        self.num_frame_path = num_frame_path
        self.mmap = mmap
        self.max_body = max_body

        self.load_data()
        
    def load_data(self):
        # data: N C V T 

        # load label
        if '.pkl' in self.label_path:   
            with open(self.label_path, 'rb') as f:
                self.sample_name, self.label = pickle.load(f)

        # load data
        if self.mmap == True:
            self.data = np.load(self.data_path,mmap_mode='r')
        else:
            self.data = np.load(self.data_path,mmap_mode=None) 

        # load num of valid frame length
        self.valid_frame_num = np.load(self.num_frame_path)

        # actions per subjects
        self.actions_num = len(self.label)/4

        # N - sample, C - xyz, T - frame, V - joint
        if self.max_body == 1 :
            self.N, self.C, self.T, self.V = self.data.shape
        else :
            raise NotImplementedError('multiperson not implemented')




if __name__ == '__main__':

    import numpy as np

    # testing, works for separated environments. No need to run this, just if want to get more info about data
    base_path = "../data0/CAD-60"
    environments = ['bathroom', 'bedroom', 'kitchen', 'livingroom', 'office']
    data_file = "train_data.npy"
    label_file = "train_label.pkl"
    num_frame_file = "train_num_frame.npy"
    
    for env in environments:

        print("Environment: " +  env)

        data_path = os.path.join(base_path, env, data_file)
        label_path = os.path.join(base_path, env, label_file)
        num_frame_path = os.path.join(base_path, env, num_frame_file)

        dataset = Dataset(data_path, label_path, num_frame_path,
                         #center=False
                         )

        print('Labels distribution: ')
        oneh_vector = np.zeros(12)
        bins = np.bincount(dataset.label)
        for i in range(len(bins)):
            oneh_vector[i] = bins[i] 
        print(oneh_vector)

        # only print for small datasets
        for i, j in zip(dataset.sample_name, dataset.label):
            print(str(i) + ' | ' + str(j))

        print('Num of frames in samples: ')
        print(dataset.valid_frame_num)

        print('Samples statistics: ')
        print(str(dataset.N) + ' samples')
        print(str(dataset.C) + ' coords')
        print(str(np.mean(dataset.valid_frame_num)) +  ' average frames') 
        print(str(dataset.V) + ' joints')

        print ('------------------')












