'''
Takes the saved detected bounding boxes for the obstacles.

Parses them and obtains the location of the center of the 
bounding boxes for each detected obstacle in each image
and saves as npy for the next modules of the pipeline to consume.
'''

import numpy as np
import math
import os
import argparse

#Parsing arguments
parser = argparse.ArgumentParser()
parser.add_argument('--folder', type=str, default='./imgs')
args = parser.parse_args()
# print("File:",args.folder)
folder_name = args.folder.split('/')[-1]

# Initialising paths and variable to load data
(_, _, filenames) = next(os.walk(args.folder))
filenames.sort()
images = filenames
#images = ['out_stiched_working.jpg']
names = []
bboxes_paths = []
exps = next(os.walk('./runs/detect/'))[1]
exps.sort()
folder = exps[-1]
base_path = './runs/detect/'+folder+'/labels'
print(folder)
for image in images:
    names.append(image.split('.')[0])

for name in names:
    bboxes_paths.append(os.path.join(base_path,str(name+'.txt')))

# Computing the location of the center of the 
# bounding boxes for each detected obstacle in each image
# and saving as npy for the next modules
count = 0
for path in bboxes_paths:
    print(path)
    if os.path.isfile(path):
        with open(path, 'r') as f:
            bboxes = []
            for line in f:
                vals = line.split()
                temp = vals[1:]
                coords = list(map(int, temp))
                bboxes.append(coords)
        image_boxes = np.array(bboxes)
        np.save('./boxes/'+str(names[count])+'.npy',image_boxes)
    count+=1
