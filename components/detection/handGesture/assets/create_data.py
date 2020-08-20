import cv2
from src.hand_tracker import Hand_Detector
from tqdm import tqdm
import os
import numpy as np

folder = "../ASLBig/asl_alphabet_train/asl_alphabet_train"

cnt = 0
final_data = np.array([])
labels = np.array([])

for alphabet in tqdm(os.listdir(folder)):
	# print(alphabet)
	for image_filename in tqdm(os.listdir(folder + '/' + alphabet)):
	    img_file = cv2.imread(folder + '/' + alphabet + '/' + image_filename)
	    # print(img_file)
	    if img_file is not None:
	        image = cv2.cvtColor(img_file, cv2.COLOR_BGR2RGB)
	        bbox, points = Hand_Detector(image)
	        if(points is not None):
	            cnt+=1
	            points = np.array(points)
	            # points = np.append(points,['C'])
	            labels = np.append(labels, alphabet)
	            final_data = np.append(final_data,points)
	            # print(points.flatten())

# print(final_data)
with open('points.npy', 'wb') as f:
    np.save(f, final_data)


with open('labels.npy', 'wb') as f2:
    np.save(f2, labels)