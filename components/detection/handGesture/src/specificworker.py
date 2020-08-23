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
from os import path
import random
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier
import pickle
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
        self.gesture_labels = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", 
                                "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "del", "space"]

        ## Trained model
        self.model = None

        ## Save model flag
        self.save_model = True
        self.model_name = "./assets/asl_model.pkl"
        print('Component Started')

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):

        return True



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getHandGesture
    #
    def HandGesture_getHandGesture(self, keypoints):
        #
        # implementCODE
        #
        keypoints = np.array(keypoints)
        keypoints = np.reshape(keypoints,(1,-1))
        gesture = self.model.predict(keypoints)
        print('Detected Gesture is:')
        print(gesture[0])
        return str(gesture[0])


    #
    # setClasses
    #
    def HandGesture_setClasses(self, classes):
        #
        # implementCODE
        #
        if(self.gesture_labels == classes and path.exists('./assets/gesture_model.pkl')):
            print('All ASL classes to be detected')
            print('Using pretrained model')
            pkl_filename = './assets/gesture_model.pkl'
            with open(pkl_filename, 'rb') as file:
                self.model = pickle.load(file)
        else:
            self.gesture_labels = classes
            print('Classes Set, Now Training')

            with open('./assets/points.npy', 'rb') as f:
                scankeys = np.load(f)

            with open('./assets/labels.npy', 'rb') as f2:
                labels = np.load(f2)
            
            scankeys = np.reshape(scankeys, (-1,42))

            keys = scankeys
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

            ## training linear SVM
            self.model = make_pipeline(StandardScaler(), SVC(gamma='auto'))
            print("Model pipeline created")
            self.model.fit(train_keys, train_labels)
            print('Model Trained')

            ## Uncomment for checking accuracy
            # print(self.model.score(test_keys,test_labels))

            ## Save model?
            if(self.save_model == True):
                print('save_model=True, Saving model')
                with open(self.model_name, 'wb') as file:
                    pickle.dump(self.model, file)
                print('Model Saved')

        return

    # ===================================================================
    # ===================================================================


