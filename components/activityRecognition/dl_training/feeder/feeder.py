# code credits https://github.com/huguyuehuhu/HCN-pytorch

# sys
import pickle

# torch
import torch
from torch.autograd import Variable
from torchvision import transforms
import numpy as np
import torch.nn.functional as F

import sys
sys.path.insert(0, "../utils")
import utils

class Feeder(torch.utils.data.Dataset):
    """ Feeder for skeleton-based action recognition
    Arguments:
        data_path: the path to '.npy' data, the shape of data should be (N, C, T, V, M)
        label_path: the path to label
        window_size: The length of the output sequence
    """

    def __init__(self,
                 data_path,
                 label_path,
                 num_frame_path,
                 window_size=-1,
                 origin_transfer=False,
                 crop_resize=False,
                 random_flip=False,
                 p_interval=1,
                 mmap=False,
                 ):
        self.data_path = data_path
        self.label_path = label_path
        self.num_frame_path = num_frame_path
        self.window_size = window_size
        self.origin_transfer = origin_transfer
        self.crop_resize = crop_resize
        self.random_flip = random_flip
        self.p_interval = p_interval
        self.mmap = mmap

        self.load_data()
        self.coordinate_transfer()

    def load_data(self):

        # load label
        if '.pkl' in self.label_path:
            try:
                with open(self.label_path) as f:
                    self.sample_name, self.label = pickle.load(f)
            except:
                # for pickle file from python2
                with open(self.label_path, 'rb') as f:
                    self.sample_name, self.label = pickle.load(
                        f, encoding='latin1')
        # old label format
        elif '.npy' in self.label_path:
            self.label = list(np.load(self.label_path))
            self.sample_name = [str(i) for i in range(len(self.label))]
        else:
            raise ValueError()

        # load data
        if self.mmap == True:
            self.data = np.load(self.data_path,mmap_mode='r')
        else:
            self.data = np.load(self.data_path,mmap_mode=None) 

        # load num of valid frame length
        self.valid_frame_num = np.load(self.num_frame_path)


        # N - sample, C - xyz, T - frame, V - joint, M - body 
        self.N, self.C, self.T, self.V, self.M = self.data.shape

    def coordinate_transfer(self):
        data_numpy = self.data

        if self.origin_transfer == 2:
            #  take joints 3 (torso in CAD)  of each person, as origins of each person
            origin = data_numpy[:, :, :, 2, :]
            data_numpy = data_numpy - origin[:, :, :, None, :]
        elif self.origin_transfer == 0:
            #  take joints 3 (torso in CAD)  of first person, as origins of each person
            origin = data_numpy[:, :, :, 2, 0]
            data_numpy = data_numpy - origin[:, :, :, None, None]
        elif self.origin_transfer == 1:
            #  take joints 3 (torso in CAD)  of second person, as origins of each person
            origin = data_numpy[:, :, :, 2, 1]
            data_numpy = data_numpy - origin[:, :, :, None, None]
        else:
            pass

        self.data = data_numpy


    def __len__(self):
        return len(self.label)

    def __iter__(self):
        return self

    def __getitem__(self, index):
        # get data
        # input: C, T, V, M
        data_numpy = self.data[index]
        # if self.mmap = True, the loaded data_numpy is read-only, and torch.utils.data.DataLoader could load type 'numpy.core.memmap.memmap'
        if self.mmap:
            data_numpy = np.array(data_numpy) # convert numpy.core.memmap.memmap to numpy

        label = self.label[index]
        valid_frame_num = self.valid_frame_num[index]

        # processing
        if self.crop_resize:
            data_numpy = self.valid_crop_resize(data_numpy, valid_frame_num, self.window_size, p_interval=self.p_interval)
        if self.random_flip:
            data_numpy = self.do_random_flip(data_numpy)
        return data_numpy, label

    def top_k(self, score, top_k):
        rank = score.argsort()
        hit_top_k = [l in rank[i, -top_k:] for i, l in enumerate(self.label)]
        return sum(hit_top_k) * 1.0 / len(hit_top_k)

    def crop(self, data, tw, th):
        _, w, h, _ = data.shape
        x1 = int(round((w - tw) / 2.))
        y1 = int(round((h - th) / 2.))
        return data[:, x1:x1 + tw, y1:y1 + th, :]

    def do_random_flip(self, data):
        draw = np.random.randint(low=1, high=100)
        if draw < 50:
            data[0, :, :, :] *= -1
        return data

    def valid_crop_resize(self, data_numpy, valid_frame_num, window, p_interval):
        # input: C,T,V,M
        C, T, V, M = data_numpy.shape

        if 'NTU' in self.data_path:
            begin = 0
            end = valid_frame_num
            valid_size = end - begin

            #crop
            if len(p_interval) == 1:
                p = p_interval[0]
                bias = int((1-p) * valid_size/2)
                data = data_numpy[:, begin+bias:end-bias, :, :]# center_crop
                cropped_length = data.shape[1]
            else:
                p = np.random.rand(1)*(p_interval[1]-p_interval[0])+p_interval[0]
                cropped_length = np.minimum(np.maximum(int(np.floor(valid_size*p)),64),valid_size)# constraint cropped_length lower bound as 64
                bias = np.random.randint(0,valid_size-cropped_length+1)
                data = data_numpy[:,begin+bias:begin+bias+cropped_length, :, :]

            # resize
            data = torch.tensor(data,dtype=torch.float)
            data = data.permute(0, 2, 3, 1).contiguous().view(C * V * M, cropped_length)
            data = data[None, :, :, None]
            data = F.upsample(data, size=(window, 1), mode='bilinear',align_corners=False).squeeze(dim=3).squeeze(dim=0) # could perform both up sample and down sample
            data = data.contiguous().view(C, V, M, window).permute(0, 3, 1, 2).contiguous().numpy()

        else:
            step = data_numpy.shape[1]//window
            count = 0
            
            data = np.zeros((C, window, V, M), dtype=np.float32)
            for i in range(valid_frame_num - 1):
                if i%step == 0 and count < window:
                    data[:, count, :, :] = data_numpy[:, i, :, :]
                    count +=1

        return data


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import numpy as np

    # testing
    data_path = "../data0/NTU-RGB-D/xview/val_data.npy"
    label_path = "../data0/NTU-RGB-D/xview/val_label.pkl"
    num_frame_path = "../data0/NTU-RGB-D/xview/val_num_frame.npy"
    

    dataset = Feeder(data_path, label_path, num_frame_path,
                     window_size=100,
                     origin_transfer=False
                     )
    print(np.bincount(dataset.label))

    # only print for small datasets
    # for i, j in zip(dataset.sample_name, dataset.label):
    #     print(str(i) + ' | ' + str(j))

    print(dataset.valid_frame_num)

    print(f"{dataset.N} samples")
    print(f"{dataset.C} coords")
    print(f"{dataset.T} frames") 
    print(f"{dataset.V} joints")
    print(f"{dataset.M} bodies")











