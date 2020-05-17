#
# Copyright (C) 2019 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time
sys.path.append(os.path.join(os.getcwd(),"assets","src"))
import numpy as np
import cv2
import wx
from wx.lib import buttons
from PIL import Image
from PySide import QtGui, QtCore
from genericworker import *
import face_detect as detector
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
from threading import Thread
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import tensorflow as tf
from datetime import datetime

face_cascade = cv2.CascadeClassifier('assets/haarcascade_frontalface_default.xml')

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		files = os.listdir('.')
		if ('captured_images' in files and os.path.isdir('./captured_images')):
			pass
		else:
			folder_path = os.path.join(os.getcwd(),'captured_images')
			os.system('mkdir %s'%(folder_path))
		self.frames_stored = 0
		self.max_frames = 10
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)
		self.save_path = './'
		self.face_detect_gpu_memory = 0.4
		#### Defining the GUI 
		self.app = wx.App()
		self.window = wx.Frame(None, -1, title = "Face Recognition Options", size=(640,480),
							style=wx.DEFAULT_FRAME_STYLE | wx.NO_FULL_REPAINT_ON_RESIZE)

		self.window.panel = wx.Panel(self.window)

		### Defining buttons in the gui
		sizer2 = wx.BoxSizer(wx.VERTICAL)
		buttons = []
		buttons.append(wx.Button(self.window.panel, -1, "Recognize faces in the images"))
		buttons.append(wx.Button(self.window.panel, -1, "Add image to the database"))
		buttons.append(wx.Button(self.window.panel, -1, "Delete an existing label"))   
		buttons.append(wx.Button(self.window.panel, -1, "Add label from camera feed"))   

		self.window.text_ctrl = wx.TextCtrl(self.window.panel, value = "Enter the label to add or delete ...")

		### Adding events to the gui
		buttons[0].Bind(wx.EVT_BUTTON, self.FaceRecog_Image)
		buttons[1].Bind(wx.EVT_BUTTON, self.Add_Image)
		buttons[2].Bind(wx.EVT_BUTTON, self.Delete_specific_label)
		buttons[3].Bind(wx.EVT_BUTTON, self.Add_camera_feed)

		self.window.text_ctrl.Bind(wx.EVT_SET_FOCUS, self.toggle1)
		self.window.text_ctrl.Bind(wx.EVT_KILL_FOCUS, self.toggle2)

		### Adding everything to the panel
		for i in range(0, len(buttons)):
			sizer2.Add(buttons[i], 1, wx.EXPAND, wx.CENTER)
		sizer2.Add(self.window.text_ctrl, 0, wx.ALL | wx.EXPAND, 5)        

		self.window.panel.SetSizer(sizer2)        
		self.window.Show()
		self.window.Centre()

		self.window.Show(True)
		self.app.MainLoop()

	def setParams(self, params):
		return True

	### Defining function to toggle text display for the text box
	def toggle1(self, event):
		self.window.text_ctrl.SetValue("")
		event.Skip()

	def toggle2(self, event):
		if self.window.text_ctrl.GetValue() == "":
			self.window.text_ctrl.SetValue("Enter the label to add or delete ...")
		event.Skip() 

	### Defining function to perform the FaceRecogntion if the image is given as input
	def FaceRecog_Image(self, event):
		dlg = wx.FileDialog(self.window, "Open image file...", os.getcwd()+"/image",
							style=wx.FD_OPEN,
							wildcard = "Image files (*.png;*.jpeg;*.jpg)|*.png;*.jpeg;*.jpg")
		if dlg.ShowModal() == wx.ID_OK:
			self.window.filename = dlg.GetPath()
			img = cv2.imread(self.window.filename)
			aspect_ratio = float(img.shape[0])/float(img.shape[1])
			window_height = 600
			window_width = window_height/aspect_ratio
			img = cv2.resize(img, (int(window_width), int(window_height)))
			faces = self.face_detection(img, flag = 1)

			for idx, face in enumerate(faces):
				x = faces[idx,0]
				y = faces[idx,1]
				w = faces[idx,2]
				h = faces[idx,3]
				im = TImage()
				im.height = int(faces[idx,4].shape[0])
				im.width = int(faces[idx,4].shape[1])
				im.depth = int(faces[idx,4].shape[2])
				im.image = faces[idx,4]
				FaceName = self.faceidentification_proxy.getFaceLabels(im)
				cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0),2)
				cv2.putText(img, FaceName, (x,y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255) ,1 , cv2.LINE_AA)
			img_path = '%sFace.png'%(self.save_path)	
			cv2.imwrite(img_path, img)
			msg = "Image with the bounding box saved at %s"%(img_path)
			wx.MessageBox(msg)
		dlg.Destroy()

	### Defining the function to add image to an existing label
	def Add_Image(self, event):
		if self.window.text_ctrl.GetValue() == "Enter the label to add or delete ...":
			wx.MessageBox("Enter the existing label for which image is to be added in the text box below")
		else:
			self.window.label_add = self.window.text_ctrl.GetValue()
			self.window.label_add = self.window.label_add.lower()
			dlg = wx.FileDialog(self.window, "Open image file...", os.getcwd()+"/image",
								style=wx.FD_OPEN,
								wildcard = "Image files (*.png;*.jpeg;*.jpg)|*.png;*.jpeg;*.jpg")
			if dlg.ShowModal() == wx.ID_OK:
				self.window.filename = dlg.GetPath()

				#### Detecting Face and then storing it in the database
				img = cv2.imread(self.window.filename)
				faces = self.face_detection(img, flag = 1)
				if (faces.shape[0] == 0):
					wx.MessageBox("No face found in the given image")
				elif (faces.shape[0] > 1):
					wx.MessageBox("Multiple people found in the image")

				else:
					#### Adding image to the database
					im = TImage()
					im.height = int(faces[0,4].shape[0])
					im.width = int(faces[0,4].shape[1])
					im.depth = int(faces[0,4].shape[2])
					im.image = faces[0,4]
					status = self.faceidentification_proxy.addNewFace(im,self.window.label_add)
					if (status):
						msg = "%s added to the database"%(self.window.label_add)
					else:
						msg = "Given person is stored with a different name. Hence the given encoding is not stored."
					wx.MessageBox(msg)
			dlg.Destroy()

	### Function to delete a specific label
	def Delete_specific_label(self, event):
		if self.window.text_ctrl.GetValue() == "Enter the label to add or delete ...":
			wx.MessageBox("Enter the label you want to delete in the text box below")
		else:
			self.window.label_delete = self.window.text_ctrl.GetValue()
			self.window.label_delete = self.window.label_delete.lower()
			self.faceidentification_proxy.deleteLabel(self.window.label_delete)
			print ("Deleting the label : " + self.window.label_delete)
			wx.MessageBox("Label deleted successfully")	

	### Function to add camera directly from the camera feed
	def Add_camera_feed(self,event):
		### Capture and store the people for the max_frames
		if (self.frames_stored == 0 | self.frames_stored < self.max_frames):
			if self.window.text_ctrl.GetValue() == "Enter the label to add or delete ...":
				wx.MessageBox("Enter the label for which image is to be added in the text box below")
			else:
				self.window.label_add_captured = self.window.text_ctrl.GetValue()
				self.window.label_add_captured = self.window.label_add_captured.lower()
				for idx, face in enumerate(self.faces_detected):
					now = datetime.now().time().strftime("%H_%M_%S_%f")
					img_name = './captured_images/%s_%d_%d.jpg'%(now, self.frames_stored, idx)
					cv2.imwrite(img_name, self.faces_detected[idx,4])
				self.frames_stored = self.frames_stored + 1
		#### Select the image and add it to the database
		elif (self.frames_stored == self.max_frames):
			folder_path = os.path.join(os.getcwd(),'captured_images')
			dlg = wx.FileDialog(self.window, "Open image file...", folder_path+"/image",
				style=wx.FD_OPEN,
				wildcard = "Image files (*.png;*.jpeg;*.jpg)|*.png;*.jpeg;*.jpg")
			if dlg.ShowModal() == wx.ID_OK:
				self.window.filename = dlg.GetPath()
				img = cv2.imread(self.window.filename)
				im = TImage()
				im.height = int(img.shape[0])
				im.width = int(img.shape[1])
				im.depth = int(img.shape[2])
				im.image = img
				status = self.faceidentification_proxy.addNewFace(im,self.window.label_add_captured)
				if (status):
					msg = "%s added to the database"%(self.window.label_add_captured)
				else:
					msg = "Given person is stored with a different name. Hence the given encoding is not stored."
				wx.MessageBox(msg)
			dlg.Destroy()
			self.frames_stored = 0
			os.system('rm -rf %s'%(folder_path))
			os.system('mkdir %s'%(folder_path))
		else:
			self.frames_stored = 0

	#### Function to detect multiple faces in an image 
	def face_detection(self, img, flag):
		#### Use Haar Cascade for face detection
		if (flag == 0):
			gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY )
			faces = face_cascade.detectMultiScale(gray)
			faces_data = []
			for (x,y,w,h) in faces:
				I_align = np.array(img[int(y):int(y + h), int(x):int(x + w)])
				faces_data.append([x,y,w,h, I_align])
			faces_data = np.array(faces_data)

		#### Use MTCNN for face detection
		elif (flag == 1):
			faces_data = detector.draw_bounding_box(img, self.face_detect_gpu_memory)
		return faces_data


	@QtCore.Slot()
	def compute(self):
		# print 'SpecificWorker.compute...'
		try:
			data = self.camerasimple_proxy.getImage()
			arr = np.fromstring(data.image, np.uint8)
			frame = np.reshape(arr, (data.height, data.width, data.depth))
			### Getting bounding boxes across the faces using Harr cascade implementation
			self.faces_detected = self.face_detection(frame, flag = 0)

			#### Finding name for each person for the bounding boxes
			for idx, face in enumerate(self.faces_detected):
				x = self.faces_detected[idx,0]
				y = self.faces_detected[idx,1]
				w = self.faces_detected[idx,2]
				h = self.faces_detected[idx,3]
				im = TImage()
				im.height = int(self.faces_detected[idx,4].shape[0])
				im.width = int(self.faces_detected[idx,4].shape[1])
				im.depth = int(self.faces_detected[idx,4].shape[2])
				im.image = self.faces_detected[idx,4]
				FaceName = self.faceidentification_proxy.getFaceLabels(im)

				cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0),2)
				cv2.putText(frame, FaceName, (x,y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1 , cv2.LINE_AA)

			cv2.imshow('Face', frame)	

			if (self.frames_stored != 0):
				self.Add_camera_feed('invoke_event')
		except Ice.Exception as e:
			traceback.print_exc()
			print(e)

		cv2.waitKey(1)

		return True
