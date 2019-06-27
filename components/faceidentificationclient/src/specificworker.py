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
import numpy as np
import cv2
import tensorflow as tf
import wx
from wx.lib import buttons
from PIL import Image
from PySide import QtGui, QtCore
from genericworker import *
from assets.src import face_detect as detector
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
from threading import Thread

face_cascade = cv2.CascadeClassifier('assets/haarcascade_frontalface_default.xml')

class MyFrame(wx.Frame):
	def __init__(self, parent):
		wx.Frame.__init__(self, parent, -1, "Face Recognition", size=(640,480),
							style=wx.DEFAULT_FRAME_STYLE | wx.NO_FULL_REPAINT_ON_RESIZE)
		panel = wx.Panel(self)     

		### Defining buttons in the gui
		sizer2 = wx.BoxSizer(wx.VERTICAL)
		buttons = []
		buttons.append(wx.Button(panel, -1, "Recognize faces in the images"))
		buttons.append(wx.Button(panel, -1, "Add image to the database"))
		buttons.append(wx.Button(panel, -1, "Delete an existing label"))   

		self.text_ctrl = wx.TextCtrl(panel, value = "Enter the label to add or delete ...")

		### Adding events to the gui
		buttons[0].Bind(wx.EVT_BUTTON, self.FaceRecog_Image)
		buttons[1].Bind(wx.EVT_BUTTON, self.Add_Image)
		buttons[2].Bind(wx.EVT_BUTTON, self.Delete_specific_label)
		self.text_ctrl.Bind(wx.EVT_SET_FOCUS, self.toggle1)
		self.text_ctrl.Bind(wx.EVT_KILL_FOCUS, self.toggle2)

		### Adding it to the panel
		for i in range(0, 3):
			sizer2.Add(buttons[i], 1, wx.EXPAND, wx.CENTER)
		sizer2.Add(self.text_ctrl, 0, wx.ALL | wx.EXPAND, 5)        

		panel.SetSizer(sizer2)        
		self.Show()
		self.Centre()

	### Defining function to toggle text display for the text box
	def toggle1(self, event):
		if self.text_ctrl.GetValue() == "Enter the label to add or delete ...":
			self.text_ctrl.SetValue("")
		event.Skip()

	def toggle2(self, event):
		if self.text_ctrl.GetValue() == "":
			self.text_ctrl.SetValue("Enter the label to add or delete ...")
		event.Skip()    

	### Defining function to perform the FaceRecogntion if the image is given as input
	def FaceRecog_Image(self, event):
		dlg = wx.FileDialog(self, "Open image file...", os.getcwd()+"/image",
							style=wx.FD_OPEN,
							wildcard = "Image files (*.png;*.jpeg;*.jpg)|*.png;*.jpeg;*.jpg")
		if dlg.ShowModal() == wx.ID_OK:
			self.filename = dlg.GetPath()
			print ('Image path is :' + self.filename)
			img = cv2.imread(self.filename)

			faces = face_detection(img, flag = 1)

			for idx, face in enumerate(faces):
				x = faces[idx,0]
				y = faces[idx,1]
				w = faces[idx,2]
				h = faces[idx,3]
				FaceName = "Unknown"
				cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0),2)
				cv2.putText(img, FaceName, (x,y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255) ,2 , cv2.LINE_AA)
			cv2.imwrite('Face.png', img)
		dlg.Destroy()

	### Defining the function to add image to an existing label
	def Add_Image(self, event):
		if self.text_ctrl.GetValue() == "Enter the label to add or delete ...":
			wx.MessageBox("Enter the existing label for which image is to be added in the text box below")
		else:
			self.label_add = self.text_ctrl.GetValue()
			print ("Adding image for the label : " + self.label_add)
			dlg = wx.FileDialog(self, "Open image file...", os.getcwd()+"/image",
								style=wx.FD_OPEN,
								wildcard = "Image files (*.png;*.jpeg;*.jpg)|*.png;*.jpeg;*.jpg")
			if dlg.ShowModal() == wx.ID_OK:
				self.filename = dlg.GetPath()
				print ('File path is:' + self.filename)
				#### Add the existing functionality and checks
			dlg.Destroy()

	### Defining the function to delete the database file
	def Delete_all_labels(self, event):
		### Add the functionality to delete the existing embedding file
		wx.MessageBox("All labels deleted")
		print ("All labels deleted successfully")

	### Defining the fucnction to delete a specific label
	def Delete_specific_label(self, event):
		if self.text_ctrl.GetValue() == "Enter the label to add or delete ...":
			wx.MessageBox("Enter the label you want to delete in the text box below")
		else:
			self.label_delete = self.text_ctrl.GetValue()
			print ("Deleting the label : " + self.label_delete)
			### Add the required Functionality
			wx.MessageBox("Label deleted successfully")		

def face_detection(img, flag):
	#### Use Haar Cascade for face detection
	if (flag == 0):
		gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY )
		faces = face_cascade.detectMultiScale(gray)
		faces_data = []
		for (x,y,w,h) in faces:
			I_align = img[int(x):int(x+w), int(y):int(y+h)]
			faces_data.append([x,y,w,h, I_align])
		faces_data = np.array(faces_data)

	#### Use MTCNN for face detection
	elif (flag == 1):
		faces_data = detector.draw_bounding_box(img)
	return faces_data

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)
		self.app = wx.App()
		self.frame = MyFrame(None)
		self.frame.Show(True)
		self.app.MainLoop()

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		# print 'SpecificWorker.compute...'
		try:
			data = self.camerasimple_proxy.getImage()
			arr = np.fromstring(data.image, np.uint8)
			frame = np.reshape(arr, (data.width, data.height, data.depth))

			### Getting bounding boxes across the faces using Harr cascade implementation
			faces = face_detection(frame, flag = 0)

			for idx, face in enumerate(faces):
				x = faces[idx,0]
				y = faces[idx,1]
				w = faces[idx,2]
				h = faces[idx,3]
				FaceName = "Unknown"
				cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0),2)
				cv2.putText(frame, FaceName, (x,y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255) ,2 , cv2.LINE_AA)

			cv2.imshow('Face', frame)	

		except Ice.Exception, e:
			traceback.print_exc()
			print e	

		cv2.waitKey(1)

		return True
