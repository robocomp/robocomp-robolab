#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2018 by Esteban Martinena
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

from PySide import QtGui, QtCore

import imutils

from genericworker import *
from scipy import stats
import numpy as np
import cv2


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.debug = False
        self.flip = False
        self.fps = 0
        self.greenLower = (29, 86, 6)
        self.greenUpper = (64, 255, 255)
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.click_event)
        self.refPt = []
        self.lower_hsv = None
        self.upper_hsv = None
        self._state = "initial"
        self._colors_tracked = []
        self._last_ref_pixel_coord = None
        cv2.namedWindow('camshift')
        cv2.setMouseCallback('camshift', self.onmouse)
        self._last_selection = None
        self.drag_start = None
        self.show_backproj = False
        self._last_track_window = None
        # self.run()
        self.timer.start(self.Period)

    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self._last_track_window = None
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self._last_selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:
            self.drag_start = None
            self._last_track_window = (xmin, ymin, xmax - xmin, ymax - ymin)


        # It helps to increase the space between the depth wall and some inclination or frames

    def click_event(self, event, x, y, flags, param):

        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            self._last_ref_pixel_coord = (x, y)
            self._state = "new_color"
            print("Reference point %s" % (str(self._last_ref_pixel_coord)))

    def show_hist(self, hist):
        bin_count = hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)


    def setParams(self, params):
        if "debug" in params:
            if "true" in params["debug"].lower():
                self.debug = True

        if "flip" in params:
            if "true" in params["flip"].lower():
                self.flip = True
        return True

    @QtCore.Slot()
    def compute(self):
        self.last_time = time.time()
        try:
            # image = self.camerasimple_proxy.getImage()
            # frame = np.fromstring(image.image, dtype=np.uint8)
            # frame = frame.reshape(image.width, image.height, image.depth)

            color, depth, _, _ = self.rgbd_proxy.getData()
            frame = np.fromstring(color, dtype=np.uint8)
            frame = frame.reshape(480, 640, 3)

            depth = np.array(depth, dtype=np.float32)

            # depth = np.array(depth, dtype=np.uint8)
            # depth = depth.reshape(480, 640, 1)
            if self.flip:
                frame = cv2.flip(frame, 0)
                depth = np.fromstring(depth, dtype=np.float32)
                depth = depth.reshape(480, 640, 1)
                depth = cv2.flip(depth, 0)
                depth = depth.reshape(480 * 640)

            # depth = self.depth_normalization(depth).reshape(480, 640, 1).astype(np.uint8)
            # self.calculate_depth_threshold()
            # if self.depth_thresold:
            #     depth=depth[depth>self.depth_thresold]
            # cv2.imshow("depth normalized", depth)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # white_balanced = frame.copy()  # white_balance(frame)
            # white_balanced = white_balance(frame)
            # white_balanced = cv2.balanceWhite(frame, cv2.xphoto.)
            white_balanced = np.dstack([wb(channel, 0.05) for channel in cv2.split(frame)])
            self.frame = frame
            vis = self.frame.copy()
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            # A Range of useful tones (0-180) discarding unsaturated values (0-60) and too dark or too bright values)
            mask = cv2.inRange(hsv, np.array((0., 30., 100.)), np.array((180., 255., 255.)))
            cv2.imshow("Discarded areas mask", mask)

            if self._last_selection:
                x0, y0, x1, y1 = self._last_selection
                hsv_roi = hsv[y0:y1, x0:x1]
                mask_roi = mask[y0:y1, x0:x1]
                hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
                cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
                hist = hist.reshape(-1)

                self.show_hist(hist)

                vis_roi = vis[y0:y1, x0:x1]
                cv2.bitwise_not(vis_roi, vis_roi)
                vis[mask == 0] = 0


                print self._last_track_window
                if self._last_track_window and self._last_track_window[2] > 0 and self._last_track_window[3] > 0:
                    self._colors_tracked.append({"id":len(self._colors_tracked), "histogram": hist, "track_window": self._last_track_window})
                    self._last_track_window = None
                    self._last_selection = None

            for tracked_color in self._colors_tracked:
                if "track_window" in tracked_color:
                    print("Tracked color %d at %s "%(tracked_color["id"], str(tracked_color["track_window"])))
                    window = tracked_color["track_window"]
                    prob = cv2.calcBackProject([hsv], [0], tracked_color["histogram"], [0, 180], 1)
                    prob &= mask
                    term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
                    track_box, tracked_color["track_window"] = cv2.CamShift(prob, window, term_crit)

                    if self.show_backproj:
                        vis[:] = prob[..., np.newaxis]
                    try:
                        cv2.ellipse(vis, track_box, (0, 0, 255), 2)
                    except:
                        print(track_box)
                else:
                    print "No tracking window"

            cv2.imshow('camshift', vis)

            ch = cv2.waitKey(5)
            if ch == 27:
                return

            return


            # OLD VERSION CODE
			#
			#
            # if "new_color" in self._state:
            #     print "Seetting reference colors"
            #     # label_image = self.label_image(frame)
			#
            #     filled = white_balanced.copy()
            #     h, w, chn = frame.shape
            #     mask1 = np.zeros((h + 2, w + 2), np.uint8)
            #     # mask2 = np.zeros((h + 2, w + 2), np.uint8)
            #     # image_mask_image = np.zeros((h , w), np.uint8)
            #     if self._last_ref_pixel_coord is not None:
            #         floodflags = 4
            #         floodflags |= cv2.FLOODFILL_MASK_ONLY
            #         floodflags |= (255 << 8)
            #         rect = None
            #         _,_,_,rect = cv2.floodFill(filled, mask1, self._last_ref_pixel_coord, (0, 0, 255), (20, 20, 20), (20, 20, 20),
            #                      floodflags)
            #         print rect
            #         mask1 = mask1[1:h + 1, 1:w + 1]
			#
            #         img1_bg = cv2.bitwise_and(white_balanced, white_balanced, mask=mask1)
            #         cv2.imshow("flood", img1_bg)
			#
            #         hsv_frame = cv2.cvtColor(white_balanced, cv2.COLOR_RGB2HSV)
			#
            #         # get the pixels of hsv_frame that are 255 in the generated mask
            #         masked_pixels = hsv_frame[np.where(mask1 == 255)]
			#
            #         # img_avg = np.mean(img_mask, axis=0)
            #         # inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
            #         # 		Scalar(180, 256, MAX(_vmin, _vmax)), mask);
            #         # get the min hue and saturation hsv value
            #         min_hsv = np.min(masked_pixels, axis=0)
            #         min_hsv[1] = 0
            #         min_hsv[2] = 0
            #         # min_hsv[2] = 128
			#
            #         # get the max hue and saturation hsv value
            #         max_hsv = np.max(masked_pixels, axis=0)
            #         max_hsv[1] = 255
            #         max_hsv[2] = 255
            #         # max_hsv[2] = 128
			#
            #         color_sample = min_hsv * np.ones((20, 20, 3), np.uint8)
            #         color_sample2 = max_hsv * np.ones((20, 20, 3), np.uint8)
            #         color_sample = cv2.cvtColor(color_sample, cv2.COLOR_HSV2RGB)
            #         color_sample2 = cv2.cvtColor(color_sample2, cv2.COLOR_HSV2RGB)
            #         cv2.imshow("color_sample", color_sample)
            #         cv2.imshow("color_sample2", color_sample2)
			#
            #         self._colors_tracked.append({"hsv_ref_values": (min_hsv, max_hsv), "region":rect})
            #         self._state = "tracking"
            #         self._last_ref_pixel_coord = None
            # if "tracking" in self._state:
            #     print "Tracking %d colors" % (len(self._colors_tracked))
            #     for id, color_tracked in enumerate(self._colors_tracked):
            #         color_tracked_frame, mask = self.get_color_mask(white_balanced, color_tracked["hsv_ref_values"][0],
            #                                                         color_tracked["hsv_ref_values"][1], None)
            #         cv2.imshow("color_mask"+str(id), mask)
            #     cv2.imshow("DEBUG: HandDetection: Specificworker: color_filter", color_tracked_frame)
			#
            # if self.debug:
            #     depth_to_show = self.depth_normalization(depth).reshape(480, 640, 1).astype(np.uint8)
            #     frame_to_show_wb = cv2.cvtColor(white_balanced, cv2.COLOR_BGR2RGB)
            #     frame_to_show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: depth readed ", depth_to_show)
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: color", frame_to_show)
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: color_wb", frame_to_show_wb)
			#
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: color_filter", mask)
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: mask", image_mask_image)
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: mask", image_mask_image)
            #     # cv2.imshow("DEBUG: HandDetection: Specificworker: filled", filled)
            #     cv2.imshow("image", white_balanced)
            #     # cv2.imshow("mask", mask1)



        except Ice.Exception, e:
            traceback.print_exc()
            print e
            return False

        return True

    def calculate_fps(self):
        self.fps = 1.0 / (time.time() - self.last_time)

    def depth_normalization(self, depth):
        # depth_min = np.min(depth)
        # depth_max = np.max(depth)
        # if depth_max != depth_min and depth_max > 0:
        #     depth = np.interp(depth, [depth_min, depth_max], [0.0, 255.0], right=255, left=0)
        depth = cv2.normalize(depth, depth,0,255,cv2.NORM_MINMAX)
        return depth

    def calculate_depth_threshold(self, depth):
        depth = np.array(depth)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        # Create 3 clusters of point (0, intermediate, far)
        ret, label, center = cv2.kmeans(depth.astype(np.float32), 3, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        clusters = [depth[label.ravel() == 0], depth[label.ravel() == 1], depth[label.ravel() == 2]]
        # The wall or table probably is the bigger one of the clusters
        index = np.argmax(map(len, clusters))
        # print np.median(clusters[index])
        # print np.mean(clusters[index])
        # Use the most repeated value as the depth to wich the wall or table can be.
        # TODO: ENV_DEPENDECE: 70 is the amount of space for the frame of the tv or for inclination from the camera plane
        self.depth_thresold = stats.mode(clusters[index])[0][0] - 70

    def label_image(self, frame):
        new_frame = frame.copy()
        new_frame = cv2.threshold(new_frame, 127, 255, cv2.THRESH_BINARY)[1]  # ensure binary
        ret, labels = cv2.connectedComponents(new_frame)

        # Map component labels to hue val
        label_hue = np.uint8(179 * labels / np.max(labels))
        blank_ch = 255 * np.ones_like(label_hue)
        labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

        # cvt to BGR for display
        labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

        # set bg label to black
        labeled_img[label_hue == 0] = 0

        return labeled_img

    def get_color_mask(self, frame, hsv_min, hsv_max, region=None):
        new_frame = frame.copy()
        mask = np.zeros((480, 640), np.uint8)
        if hsv_min is None or hsv_max is None:
            print("No lower or upper hsv. return without color mask")
            return new_frame, mask
        new_frame = frame.copy()
        blurred = cv2.GaussianBlur(new_frame, (11, 11), 0)
        cv2.imshow("blurred", blurred)
        blurred_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        if region:
            final_hsv = np.zeros(new_frame.shape, np.uint8)
            final_hsv[region[1]:region[1]+region[3], region[0]:region[0] + region[2]] = blurred_hsv[region[1]:region[1]+region[3], region[0]:region[0] + region[2]]
            cv2.imshow("blurred", final_hsv)
        else:
            final_hsv=blurred_hsv




        # construct a mask for the color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        print("In Range: %s %s, region %s" % (str(hsv_min), str(hsv_max), str(region)))
        mask = cv2.inRange(final_hsv,hsv_min, hsv_max)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the new_frame,
                # then update the list of tracked points
                cv2.circle(new_frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(new_frame, center, 5, (0, 0, 255), -1)
        return new_frame, mask


def wb(channel, perc = 0.05):
    mi, ma = (np.percentile(channel, perc), np.percentile(channel,100.0-perc))
    channel = np.uint8(np.clip((channel-mi)*255.0/(ma-mi), 0, 255))
    return channel

def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2RGB)
    # cv2.imshow("white balanced", result)
    return result
