#### Python file which detects the faces in the given image and then returns the aligned image and bounding box coordinates

import sys
import argparse
import time
import os
import cv2
import numpy as np
import test_img

def getNewCoords(x,y,rectangles):
	bbUpperLeftX = rectangles[0]
	bbUpperLeftY = rectangles[1]
	bbLowerRightX = rectangles[2]
	bbLowerRightY = rectangles[3]

	sizeX = bbLowerRightX - bbUpperLeftX
	sizeY =  bbLowerRightY - bbUpperLeftY

	sizeMax = max(sizeX, sizeY)

	centerX = (bbLowerRightX + bbUpperLeftX)/2
	centerY = (bbLowerRightY + bbUpperLeftY)/2

	offsetX = (centerX-sizeMax/2)*256/sizeMax
	offsetY = (centerY-sizeMax/2)*256/sizeMax

	x = x * 256/sizeMax - offsetX 
	y = y * 256/sizeMax - offsetY
	return (x,y)

#### Face alignment to make face straight in all the images
def alignment(I,points):
	desired_left_eye = [0.35, 0.35]
	desired_face_width = 256
	desired_face_height = 256

	dY = points[3] - points[1]
	dX = points[2] - points[0]
	angle = (np.arctan(dY/dX))
	angle = np.degrees(angle) 

	eyes_center = ((points[0] + points[2])/2,(points[1] + points[3])/2)

	### Rotation Matrix
	R = cv2.getRotationMatrix2D(eyes_center,angle,1)
	tX = desired_face_width * 0.5
	tY = desired_face_height * desired_left_eye[1]

	R[0,2] = R[0,2] + tX - eyes_center[0]
	R[1,2] = R[1,2] + tY - eyes_center[1]

	W,H = desired_face_width, desired_face_height
	output = cv2.warpAffine(I,R,(W,H),flags = cv2.INTER_CUBIC)
	return output

def draw_bounding_box(img, gpu_memory):
	rectangles, points = test_img.bounding_box_face(img,gpu_memory)
	points = np.transpose(points)

	#### Increasing the size of bounding box by 10%
	width = rectangles[:,2] - rectangles[:,0]
	height = rectangles[:,3] - rectangles[:,1]
	
	rectangles[:,0] = rectangles[:,0] - width * 0.10
	rectangles[:,1] = rectangles[:,1] - height * 0.10
	rectangles[:,2] = rectangles[:,2] + width * 0.10
	rectangles[:,3] = rectangles[:,3] + height * 0.10

	#### Cropping the images for the given bounding boxes and aligns them
	face_data = []
	img_original = img.copy()
	for i, rectangle in enumerate (rectangles):
		I = img_original[int(rectangle[1]):int(rectangle[3]), int(rectangle[0]):int(rectangle[2])]
		if (I.shape[0] == 0 or I.shape[1] == 0):
			continue
		I = cv2.resize(I,(256,256))
		point = points[i]
		point[0], point[1] = getNewCoords(point[0], point[1],rectangles[i])
		point[2], point[3] = getNewCoords(point[2], point[3],rectangles[i])
		I_align = alignment(I, [point[0], point[1],point[2], point[3]])

		face_data.append([int(rectangle[0]), int(rectangle[1]), int(rectangle[2] - rectangle[0]), int(rectangle[3] - rectangle[1]), I_align ])
	face_data = np.array(face_data)
	return face_data

