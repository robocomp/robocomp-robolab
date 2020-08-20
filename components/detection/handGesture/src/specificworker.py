#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
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

import numpy as np
import cv2
import random
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier
from genericworker import *

sys.path.append(os.path.join(os.getcwd(),"assets"))
# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

        ## ASL Alphabet Classes
        self.gesture_labels = []

        ## Trained model

        self.model = None

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')

        return True



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getHandGesture
    #
    def HandGesture_getHandGesture(self, handImg, keypoints):

        print('Called')
        arr = np.fromstring(handImg.image, np.uint8)
        frame = np.reshape(arr, (handImg.height, handImg.width, handImg.depth))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        print(frame)
        keypoints = np.array(keypoints)
        keypoints = np.reshape(keypoints,(1,-1))

        print(keypoints)
        gesture = self.model.predict(keypoints)
        # gesture = str()
        # gesture = "A"
        print(gesture[0])
        return str(gesture)

    #
    # setClasses
    #
    def HandGesture_setClasses(self, classes):
        #
        # implementCODE
        #
        self.gesture_labels = classes
        print('Classes Set, Now Training')

        with open('./assets/points.npy', 'rb') as f:
            scankeys = np.load(f)

        with open('./assets/labels.npy', 'rb') as f2:
            labels = np.load(f2)
        
        scankeys = np.reshape(scankeys, (-1,42))

        keys = scankeys
        # for k in scankeys:
        #   normkey = []
        #   for i in range(len(k)):
        #       normkey.append(k[i]-k[i%2])
        #   keys.append(normkey)


        # print(labels)
        # print(keys)

        train_idx = []
        test_idx = []

        lab_set = self.gesture_labels

        for ch in lab_set:
            all_idx = []
            for j in range(0,len(labels)):
                if(labels[j]==ch):
                    all_idx.append(j)

            random.shuffle(all_idx)
            sz = len(all_idx)

            train_sz = 0.75*sz
            test_sz = sz - train_sz

            k = 0

            while(k<train_sz):
                train_idx.append(all_idx[k])
                k+=1

            while(k<sz):
                test_idx.append(all_idx[k])
                k+=1


        train_keys, test_keys, train_labels, test_labels = [],[],[],[]


        for idx in train_idx:
            train_keys.append(keys[idx])
            train_labels.append(labels[idx])

        for idx in test_idx:
            test_keys.append(keys[idx])
            test_labels.append(labels[idx])

        # print(train_keys[0])
        print(len(train_keys))
        # # print(train_labels)
        print(len(train_labels))
        # # print(test_keys)
        print(len(test_keys))
        # # print(test_labels)
        print(len(test_labels))

        ## training linear SVM
        self.model = make_pipeline(StandardScaler(), SVC(gamma='auto'))
        print("Model pipeline created")
        self.model.fit(train_keys, train_labels)
        print('Model Trained')
        # print(self.model.score(test_keys,test_labels))


        # neigh = KNeighborsClassifier(n_neighbors=10)

        # neigh.fit(train_keys, train_labels)

        # print(neigh.score(test_keys,test_labels))
        return

    # ===================================================================
    # ===================================================================


