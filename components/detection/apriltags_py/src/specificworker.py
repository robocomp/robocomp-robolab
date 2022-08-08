#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import cv2
import numpy as np
import time
import pupil_apriltags
from scipy.spatial.transform import Rotation as R

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.detector = pupil_apriltags.Detector(families="tag36h11")
        self.tagsize = 0.5
        self.display = False
        self.image = ifaces.RoboCompCameraRGBDSimple.TImage()
        self.new_image = False
        self.tags = []
        self.cont = 0
        self.last_time = time.time()

        self.Period = 50
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        try:
            self.display = params["display"] == "true" or params["display"] == "True"
            self.family = params["family"]
            self.tagsize = float(params["tagsize"])

            self.detector = pupil_apriltags.Detector(families=self.family)

        except:
            print("Error reading config params")
        return True

    #with X right, Y down, and Z pointing out the lens, origin at the camera center


    @QtCore.Slot()
    def compute(self):
        if self.new_image:
            self.tags = []
            color = np.frombuffer(self.image.image, dtype=np.uint8)
            color = color.reshape((self.image.height, self.image.width, 3))
            camera_params = [self.image.focalx, self.image.focaly, color.shape[1]/2, color.shape[0]/2]

            detections = self.detector.detect(cv2.cvtColor(color, cv2.COLOR_RGB2GRAY),
                                              estimate_tag_pose=True,
                                              camera_params=camera_params,
                                              tag_size=self.tagsize)
            if len(detections) == 0:
                #print("Compute_april_tags: No tags detected")
                self.new_image = False
                return

            for tag in detections:
                rot = tag.pose_R
                r = R.from_matrix(rot)
                rot = np.multiply(r.as_euler('yxz'), -1).tolist()  # [r_x, r_y, r_z]
                rot[-1] *= -1
                trans = tag.pose_t

                rtag = ifaces.RoboCompAprilTags.Tag()
                rtag.id = tag.tag_id
                rtag.tx = -float(trans[0])
                rtag.ty = -float(trans[1])
                rtag.tz = float(trans[2])
                rtag.rx = rot[0]
                rtag.ry = rot[1]
                rtag.rz = rot[2]
                rtag.cx = tag.center[0]
                rtag.cy = tag.center[1]
                rtag.family = str(tag.tag_family)
                rtag.cor1x = tag.corners[0][0]
                rtag.cor1y = tag.corners[0][1]
                rtag.cor2x = tag.corners[1][0]
                rtag.cor2y = tag.corners[1][1]
                rtag.cor3x = tag.corners[2][0]
                rtag.cor3y = tag.corners[2][1]
                rtag.cor4x = tag.corners[3][0]
                rtag.cor4y = tag.corners[3][1]

                self.tags.append(rtag)

                if self.display:
                    self.draw_image(color, detections)
                    print(rtag)

            self.new_image = False

        # Hz
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
            print("Freq: ", self.cont, "Hz. Waiting for image in AprilTags.idsl")
            self.cont = 0
        else:
            self.cont += 1
        return True

    def draw_image(self, image, detections):
        for tag in detections:
            for idx in range(len(tag.corners)):
                cv2.line(image, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
            cv2.putText(image, "'" + str(tag.tag_id) + "'",
                       org = (tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                       fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
            cv2.rectangle(image, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        cv2.imshow("Tags", image)

    ################################################33

    def startup_check(self):
        print(f"Testing RoboCompAprilTagsServer.tag from ifaces.RoboCompAprilTagsServer")
        test = ifaces.RoboCompAprilTagsServer.tag()
        print(f"Testing RoboCompAprilTagsServer.Format from ifaces.RoboCompAprilTagsServer")
        test = ifaces.RoboCompAprilTagsServer.Format()
        print(f"Testing RoboCompAprilTagsServer.Image from ifaces.RoboCompAprilTagsServer")
        test = ifaces.RoboCompAprilTagsServer.Image()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Interface methods =================================
    # ===================================================================

    # IMPLEMENTATION of getAprilTags method from AprilTagsServer interface
    #
    def AprilTags_getAprilTags(self, image, tagsize, tagfamily):
        ret = ifaces.RoboCompAprilTags.TagsList()
        if self.new_image == False:
            self.image = image
            # wait for tags
            while not self.image:       # tags cannot be computed in this thread
                pass
            ret = self.tags
            print("Tags detected:")
            for r in ret:
                print(r.id)
            self.new_image = True

        return ret
    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompAprilTagsServer you can use this types:
    # RoboCompAprilTagsServer.tag
    # RoboCompAprilTagsServer.Format
    # RoboCompAprilTagsServer.Image

# CODIGO PARA CALCULAR LA DISTANCIA A PARTIR DE LA PROFUNDIDAD

   #index_x = int(tag.center[1]) // 2
            #index_y = int(tag.center[0]) // 2
            #pos_z = np.mean(depth[index_x - 10:index_x + 10, index_y - 10:index_y + 10])
            #pos_x = - ((tag.center[1] // 2 - color.shape[0] // 4) * pos_z) / focals[0]
            #pos_y = - ((tag.center[0] // 2 - color.shape[1] // 4) * pos_z) / focals[1]
            #pos = [pos_y, pos_x, pos_z]  # Acording to gripper reference frame
            # pos = [pos_x, pos_y, pos_z] # Acording to world reference frame

            #s_tags[tag.tag_id] = {"pos": pos, "rot": rot}
