'''
Takes the saved detected bounding boxes for the obstacles 
and the depth maps produced. 

Uses the image info/default values like width, height and fov
to comput the intrinsic matrix.

Uses the intirinsic matrix and metric depth values to predict the 
3D Locations.
'''

import numpy as np
import math
import os
from Kmat import intrinsic
import argparse

#Parsing arguments

parser = argparse.ArgumentParser()
parser.add_argument('--folder', type=str, default='./imgs')
parser.add_argument('--width', type=str, default='1920')
parser.add_argument('--height', type=str, default='1080')

args = parser.parse_args()

# Initialising paths and variable to load data

folder_name = args.folder.split('/')[-1]
(_, _, filenames) = next(os.walk('./boxes'))
filenames.sort()
images = filenames
#images = ['out_stiched_working.png']
names = []
bboxes_paths = []
depth_paths = []
box_path = './boxes/'
depth_path = "./out_"+folder_name+"/"
# print(depth_path)

for image in images:
    names.append(image.split('.')[0])

for name in names:
    bboxes_paths.append(os.path.join(box_path,str(name+'.npy')))

for name in names:
    depth_paths.append(os.path.join(depth_path,str('out_'+name+'.npy')))

# Computing the 3D locations using the loaded data and the intrinsic
# projection matrix and saving the final results.
count = 0
for path in bboxes_paths:
    
    # print(path)
    coords_bbox = np.load(path)
    # print(coords_bbox)
    # print(((coords_bbox[:,0]+coords_bbox[:,2])/2).astype(int))
    depth_map = np.load(depth_paths[count])
    center_x = ((coords_bbox[:,0]+coords_bbox[:,2])/2).astype(int).reshape((coords_bbox.shape[0],1))
    center_y = ((coords_bbox[:,1]+coords_bbox[:,3])/2).astype(int).reshape((coords_bbox.shape[0],1))
    # K = intrinsic(ImageSizeX=3840,ImageSizeY=1920)

    # Computing the intrinsic projection matrix and its inverse
    K = intrinsic(ImageSizeX=int(args.width),ImageSizeY=int(args.height))
    K_inv = K.inverse()

    final_3d = []
    for i in range(coords_bbox.shape[0]):
        coord_2d = np.array([[center_x[i]],[center_y[i]],[1]])
        coord_3d = np.matmul(K_inv,coord_2d)*depth_map[center_x[i],center_y[i]]
        final_3d.append(coord_3d)
    final_3d = np.array(final_3d)
    np.save('./results/'+str(names[count])+'_3d.npy',final_3d)
    print("The predicted 3d locations for "+str(names[count])+" are:\n", final_3d)
    count+=1
    
