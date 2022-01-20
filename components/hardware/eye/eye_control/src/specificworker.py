
#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import QRandomGenerator
from meld.vc import _null
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import cv2, traceback, json
import matplotlib.pyplot as plt
import math
import time
import os
import pickle
from collections import deque
import random
import json
import matplotlib.animation as animation

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

QChartView = QtCharts.QChartView
QChart = QtCharts.QChart
QValueAxis = QtCharts.QValueAxis
QLineSeries = QtCharts.QLineSeries

MIN_VELOCITY = 0.05
TOLERANCE = 0.01

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.motor = self.jointmotorsimple_proxy.getMotorState("")
        self.ui.horizontalSlider_pos.setSliderPosition(self.motor.pos)
        self.current_max_speed = 0.0

        QObject.connect(self.ui.horizontalSlider_pos, SIGNAL('valueChanged(int)'), self.slot_change_pos)
        QObject.connect(self.ui.horizontalSlider_max_speed, SIGNAL('valueChanged(int)'), self.slot_change_max_speed)
        QObject.connect(self.ui.pushButton_center, SIGNAL('clicked()'), self.slot_center)
        QObject.connect(self.ui.pushButton, SIGNAL('clicked()'), self.slot_track)


        self.m_x = 0
        self.chart = QChart()
        self.track_error = QLineSeries()
        green_pen = QPen(Qt.blue)
        green_pen.setWidth(2)
        self.track_error.setPen(green_pen)
        self.axisX = QValueAxis()
        self.axisY = QValueAxis()
        self.axisX.setTickCount(0.1)
        self.axisX.setRange(0, 1000)
        self.axisY.setRange(-5, 10)
        self.chart.addSeries(self.track_error)
        self.chart.addAxis(self.axisX, Qt.AlignBottom)
        self.chart.addAxis(self.axisY, Qt.AlignLeft)
        self.track_error.attachAxis(self.axisX)
        self.track_error.attachAxis(self.axisY)
        self.chart.legend().hide()
        self.chartView = QChartView(self.chart, self.ui.frame_error)
        self.chartView.resize(self.ui.frame_error.size())
        self.chartView.show()
        self.last_descriptors = None

        # Person position plot
        # plt.ion()
        # self.w = 1
        # self.h = 1
        # self.z = 1
        # self.d = 70
        # self.size = 500
        # plt.figure()
        # # self.ax = fig.add_subplot(projection='3d')
        # plt.axis([-2, 2, 0, 3])

        # Error data printing

        plt.ion()
        self.visible = 120
        self.d_camera_position_error = deque(np.zeros(self.visible), self.visible)
        self.dx = deque(np.zeros(self.visible), self.visible)
        self.data_length = np.linspace(0, 121, num=120)

        self.fig = plt.figure(figsize=(8, 3))
        self.ah1 = self.fig.add_subplot()
        plt.margins(x=0.001)
        self.ah1.set_ylabel("Error representation", fontsize=14)
        self.camera_position_error, = self.ah1.plot(self.dx, self.d_camera_position_error, color='green', label="Closing (x10)", linewidth=1.0)
        self.ah1.legend(loc="upper right", fontsize=12, fancybox=True, framealpha=0.5)
        self.x_data = 0

        # Dictionary for error saving

        self.error_dict = {
            "camera_position_error" : [],
            "person_lost_times" : [],
            "error_acumulator" : 0
        }

        # Vector for stored people
        self.act_people = []
        self.past_frame_people = []
        self.known_people = []
        self.main_person = None

        # Last position info
        self.last_pos = {
            "x" : 0,
            "y" : 0,
            "z" : 0,
            "angle" : 0
        }

        self.predicted_position = np.array([0, 0])
        self.leader_person = None

        # Person distance filters

        self.person_x_filter = [0, 0]
        self.person_y_filter = [0, 0]
        self.person_z_filter = [0, 0]
        self.person_angle_filter = [0, 0, 0, 0, 0, 0, 0, 0]
        self.person_vector_avg = {"x" : 0,
                                  "y" : 0,
                                  "z" : 0,
                                  "angle" : 0
                                  }

        # Filtering person speed

        self.person_speed_filter = [0, 0, 0, 0]
        self.person_speed_avg = 0

        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

        with open('human_pose.json', 'r') as f:
            self.human_pose = json.load(f)

        self.track = False
        self.error_ant = 0
        self.rad_old = 0

        # Set interesting parts key
        self.faceList = ["2", "3"]
        self.hipList = ["12", "13"]
        self.chestList = ["6", "7"]

        # Person distance filter
        self.distance_coefficients=[0,0,0]
        self.distance_avg = 0
        self.distance_old = 0

        # Rotational speed filter
        self.rotational_speed_coefficients=[0,0,0]
        self.rotational_speed_avg=0
        self.last_motor_pos = 0

        # Linear speed filter
        self.lineal_speed_coefficients=[0,0,0]
        self.lineal_speed_avg = 0

        self.distance_limit = 1.2

        self.k1 = 1.1
        self.k2 = 0.8
        self.k3 = 0.9
        self.k4 = 1
        self.k5 = 1
        self.k6 = 2
        self.k7 = 5

        self.photo_number = 0
        self.photo_image = None

        self.Period_camera = 40
        self.Period_base = 500
        if startup_check:
            self.startup_check()
        else:
            self.timer_camera.timeout.connect(self.compute_camera)
            self.timer_base.timeout.connect(self.compute_base)
            self.timer_camera.start(self.Period_camera)
            self.timer_base.start(self.Period_base)

    def __del__(self):
        pass

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute_base(self):
        self.tracker_base()

    def compute_camera(self):
        #obtenemos datos
        color, people_data = self.obtencion_datos()

        image = np.frombuffer(color.image, np.uint8).reshape(color.width, color.height, color.depth)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # cv2.imwrite("Captures/images/image_" + str(self.photo_number) + ".png", image)
        # self.image_data_to_file(people_data)
        # self.photo_number += 1

        # # Draw body parts on image
        image, person_pos_data = self.proceso_esqueleto(image, people_data)
        #
        print(person_pos_data)

        puntoMedioX = -1
        if self.known_people == []:
            self.known_people = person_pos_data

        else:
            for person in person_pos_data:
                id_similar = self.distance_comparison(person)
                # If person is found, actualize data
                if id_similar != None:
                    print("found")
                    self.act_pos_info(person, id_similar)
                    if person["x_pixel"] != -1:
                        puntoMedioX = person["x_pixel"]
                        print(puntoMedioX)
                        # print(puntoMedioX)

        # for person in people_data.peoplelist:
        #
        #     if person_coords != None:
        #         self.act_people.append(person_coords)
        # # print(self.act_people)
        #
        # if self.past_frame_people != []:
        #     for person in self.act_people:
        #         id_similar = self.distance_comparison(person)

                # # When new people is found, directly is entered to the known people vector
                # elif len(self.past_frame_people) < len(self.act_people):
                #     person["id"] = len(self.known_people)
                #     self.known_people.append(person)
                # else:
                #     pass



            # for i in self.known_people:
            #     print(i["frames"])
                # if
                # if coords != None and len(self.people) < len(people_data.peoplelist):
                #     self.people.append(coords)
            #         identified_ids.append()
            # for i in range(len(self.people)):
            #     if self.people[i]["id"] in identified_ids:
            #         print("actualised")
            #     else:
            #         self.people.pop(i)
            # print(identified_ids)

            # self.test_similitud(people_data)

        # self.past_frame_people = self.act_people

        #Hay persona y modo track activadoself
        print(puntoMedioX)
        if puntoMedioX > -1 and self.track:
            print("enter")
            self.tracker_camera(color, puntoMedioX)

        #Actualización de ventana Eye Control
        self.refesco_ventana(color, image)
        if(len(self.error_dict["camera_position_error"]) != 0):
            self.draw_error_data()

    def obtencion_datos(self):
        try:
            self.motor = self.jointmotorsimple_proxy.getMotorState("")
        except:
            print("Ice error communicating with JointMotorSimple interface")

        try:
            color = self.camerargbdsimple_proxy.getImage("")
        except:
            print("Ice error communicating with CameraRGBDSimple interface")

        try:
            people_data = self.humancamerabody_proxy.newPeopleData()
            # print(list(people_data.peoplelist[0].joints.keys()))
        except:
            traceback.print_exc()
            print("Ice error communicating with HumaCameraBody interface")
        return color, people_data

    def distance_comparison(self, coords):
        less_difference = 999
        for known_person in self.known_people:
            # Calculating position difference
            difference = math.sqrt((coords["position"][0] - known_person["position"][0])**2 + (coords["position"][1] - known_person["position"][1])**2 + (coords["position"][2] - known_person["position"][2])**2)
            # print("Difference: ", difference)
            if difference < less_difference:
                nearer_person = known_person
                less_difference = difference
        # print("Less difference: ", less_difference)
        if less_difference < 0.5:
            return nearer_person["id"]
        else:
            return None

    def get_coords(self, person):

        x_pos = []
        z_pos = []
        y_pos = []

        x_pixel_value = 0

        z_axis_rotation_matrix = np.array([[math.cos(self.motor.pos), -math.sin(self.motor.pos), 0],
                                               [math.sin(self.motor.pos), math.cos(self.motor.pos), 0],
                                               [0, 0, 1]])

        x_axis_rotation_matrix = np.array([[1, 0, 0],
                                           [0, math.cos(18 * math.pi / 180), -math.sin(18 * math.pi / 180)],
                                           [0, math.sin(18 * math.pi / 180), math.cos(18 * math.pi / 180)]])

        to_robot_reference_matrix = np.array([[1, 0, 0],
                                              [0, 0, 1],
                                              [0, 1, 0]])

        traslation_1_array = np.array([0, -0.06, -0.12])
        traslation_2_array = np.array([0, -0.04, -1.55])

        for joint in list(person.joints.keys()):
            if joint in self.chestList or joint in self.hipList or joint in self.faceList:
                if person.joints[joint].z != 0 and person.joints[joint].x != 0 and person.joints[joint].y != 0:
                    z_pos.append(person.joints[joint].z)
                    x_pos.append(person.joints[joint].x)
                    y_pos.append(person.joints[joint].y)
        if x_pos != [] and y_pos != [] and z_pos != []:
            x_z_y = np.array([sum(x_pos)/len(x_pos), sum(y_pos)/len(y_pos), sum(z_pos)/len(z_pos)])
            x_y_z = to_robot_reference_matrix.dot(x_z_y)
            converted_coords_1 = z_axis_rotation_matrix.dot(x_y_z)
            translated_1 = converted_coords_1 + traslation_1_array
            converted_coords_2 = x_axis_rotation_matrix.dot(translated_1)
            translated_2 = np.around(converted_coords_2 + traslation_2_array,2)

            person_data = {
                "id": len(self.act_people),
                "frames" : 1,
                "position": translated_2
            }

            # Printing 3D diagram

            # plt.scatter([x_y_z[0], translated_1[0], translated_2[0]],
            #             [x_y_z[1], translated_1[1], translated_2[1]],
            #            c=['r', 'b', 'g'], marker='+')
            #
            # plt.axis([-2, 2, 0, 3])
            # plt.pause(0.05)
            # plt.clf()

            return person_data

    def act_pos_info(self, pos_data, id):
        for person in self.known_people:
            if id == person["id"]:
                person["frames"] += 1
                person["speed"] = (pos_data["position"] - person["position"]) / (self.Period_camera/1000)
                person["last_position"] = person["position"]
                person["position"] = pos_data["position"]
                person["predicted_position"] = person["position"] * (self.Period_camera/1000)
                if "predicted_position" in person:
                    person["predicted_position_error"] = abs(person["predicted_position"] - person["position"])


    def test_similitud(self, people_data):
        for person in people_data.peoplelist:
            # Getting descriptors
            person_descriptors = self.get_descriptors(person)
            # print(person_descriptors)
            # Comparing with known people ones
            if len(self.people) > 1:
                for reference_person in self.people:
                    if(self.mean_descriptors_comparison(person_descriptors, reference_person["descriptors"])) > 0.85:
                        print("Same person")
                        self.act_descriptors(reference_person, person_descriptors)
                    else:
                        print("New person")
                        new_person = {}
                        new_person["id"] = person.id
                        new_person["descriptors"] = person_descriptors
                        self.people.append(new_person)
            else:
                print("First person")
                new_person = {}
                new_person["id"] = person.id
                new_person["descriptors"] = person_descriptors
                self.people.append(new_person)
        print(self.people)
        #
        # print(self.people)
        #     for stored_person in self.people:
        #         if stored_person["id"] == person.id:
        #             for key_point in list(person.joints.keys()):
        #
        #                 # Individual joint
        #                 if person.joints[key_point].floatdesclist:
        #
        #                     # If joint has descriptor, appends it normalized
        #                     stored_person["descriptors"][key_point]["vectors"].append(person.joints[key_point].floatdesclist[
        #                                                                 0] / np.linalg.norm(
        #                         person.joints[key_point].floatdesclist[0]))
        #
        #                     # If descriptor vector lenght is more than 10, calculate mean vector with forgetting factor
        #                     if len(stored_person["descriptors"][key_point]["vectors"]) > 10:
        #                         stored_person["descriptors"][key_point]["vectors"].pop[0]
        #                     average_vector = []
        #                     for i in range(0,14):
        #                         average_value = []
        #                         for j in range(0,len(stored_person["descriptors"][key_point]["vectors"])):
        #                             average_value.append(stored_person["descriptors"][key_point]["vectors"][j][i])
        #                         average_vector.append(sum(average_value)/len(average_value))
        #                     stored_person["dot_products"][key_point] = np.dot(average_vector, stored_person["descriptors"][key_point]["mean"])
        #                     stored_person["descriptors"][key_point]["mean"] = average_vector
        #
        #         else:
        #             person_dict = {"id": person.id,
        #                            "descriptors": {},
        #                            "dot_products": {}
        #                            }
        #
        #             for key_point in list(person.joints.keys()):
        #                 if person.joints[key_point].floatdesclist:
        #                     person_dict["descriptors"][key_point] = []
        #                     person_dict["dot_products"][key_point] = []
        #                     person_dict["descriptors"][key_point].append(person.joints[key_point].floatdesclist[0] / np.linalg.norm(person.joints[key_point].floatdesclist[0]))
        #
        #                     if key_point in list(self.last_descriptors.keys()):
        #                         descriptors_dot_products[key_point] = np.dot(self.last_descriptors[key_point], descriptors[key_point])
        #                         dot_product_acum.append(descriptors_dot_products[key_point])
        #
        #             if len(list(descriptors_dot_products.keys())) > 0:
        #                 descriptors_dot_products["promedio"] = sum(dot_product_acum)/len(dot_product_acum)
        #
        # if descriptors_dot_products != {}:
        #     print("dot_products: ", descriptors_dot_products)
        #
        # # Comparing with people in people list
        # if self.people > 0:
        #     for person in self.people:
        #         for key_point in list(self.last_descriptors.keys()):

####################################################################################################

        # for person in people_data.peoplelist:
        #     person_dict = {"id" : person.id,
        #                    "descriptors" : {}
        #                    }
        #     for key_point in list(person.joints.keys()):
        #         try:
        #             person_dict["descriptors"][key_point] = person.joints[key_point].floatdesclist[
        #                                                              0] / np.linalg.norm(
        #                 person.joints[key_point].floatdesclist[0])
        #         except:
        #             print("No se ha podido adquirir info")
        #     self.people.append(person_dict)
        # print("Vector gente: ", self.people)
        # print("Longitud vector gente: ", len(self.people))
        #
        # if(len(self.people) > 1):
        #     reference_person = self.people[0]
        #     reference_descriptors = list(reference_person["descriptors"].keys())
        #     for person in self.people:
        #         dot_product_vector = []
        #         if person["id"] != reference_person["id"]:
        #             for key_point in list(person["descriptors"].keys()):
        #                 if key_point in reference_descriptors:
        #                     dot_product_vector.append(np.dot(self.people[0]["descriptors"][key_point], person["descriptors"][key_point]))
        #             if len(dot_product_vector) > 0:
        #                 comparison_value = sum(dot_product_vector)/len(dot_product_vector)
        #                 print("Similitud entre personas: ", comparison_value)
        #
        # elif (len(self.people) == 1):
        #     reference_person = self.people[0]
        #     if self.last_descriptors != None:
        #         dot_product_vector = []
        #         for key_point in list(reference_person["descriptors"].keys()):
        #             if key_point in list(self.last_descriptors.keys()):
        #                 dot_product_vector.append(
        #                     np.dot(reference_person["descriptors"][key_point], self.last_descriptors[key_point]))
        #         if len(dot_product_vector) > 0:
        #             comparison_value = round(sum(dot_product_vector) / len(dot_product_vector),2)
        #             print("Similitud entre la misma persona: ", comparison_value)
        #     self.last_descriptors = reference_person["descriptors"]
        # self.people = []

    ########## Testing functions ##########

    def image_data_to_structure(self, data):
        infile = open("Captures/data/"+data, 'rb')
        new_structure = pickle.load(infile, encoding='bytes')
        return new_structure

    def image_data_to_file(self, data):
        filename = "Captures/data/image_data_"+str(self.photo_number)
        outfile = open(filename, 'wb')
        pickle.dump(data, outfile)
        outfile.close()

    def mean_descriptors_comparison(self, person_descriptors_0, person_descriptors_1):
        dot_product_vector = []
        person_0_descriptor_keys = list(person_descriptors_0.keys())
        for key_point in list(person_descriptors_1.keys()):
            if key_point in person_0_descriptor_keys:
                dot_product_vector.append(np.dot(person_descriptors_0[key_point]["mean"], person_descriptors_1[key_point]["mean"]))
        if len(dot_product_vector) > 0:
            return sum(dot_product_vector)/len(dot_product_vector)

    def act_descriptors(self, person, new_descriptors):
        descriptors = {}
        new_descriptors_list = list(new_descriptors.keys())

        for key_point in list(person["descriptors"].keys()):
            # Individual joint
            if key_point in new_descriptors_list:
                # Actualize data
                person["descriptors"][key_point]["vectors"].append(new_descriptors[key_point]["mean"][0] / np.linalg.norm(new_descriptors[key_point]["mean"][0]))
            # If new values not contain existing joints, insert the last one ponderated
            else:
                person["descriptors"][key_point]["vectors"].append(
                    person["descriptors"][key_point]["vectors"][-1] * (math.exp((10 - j) / 15) - 0.9))
            average_vector = []
            for i in range(0, 14):
                average_value = []
                for j in range(0, len(person["descriptors"][key_point]["vectors"])):
                    average_value.append(person["descriptors"][key_point]["vectors"][j][i] * (math.exp(j/15)-0.9))
                # Cuidao con el orden de meter valores
                average_vector.append(sum(average_value)/len(average_value))
            person.joints[key_point]["mean"] = average_vector

    def get_descriptors(self, person):
        descriptors = {}
        for key_point in list(person.joints.keys()):
            # Individual joint
            individual_joint = {"vectors":[],
                                "mean" : []}
            if person.joints[key_point].floatdesclist:
                # Getting normalized descriptors
                individual_joint["vectors"].append(person.joints[key_point].floatdesclist[0] / np.linalg.norm(person.joints[key_point].floatdesclist[0]))
                individual_joint["mean"] = person.joints[key_point].floatdesclist[0] / np.linalg.norm(
                    person.joints[key_point].floatdesclist[0])
                descriptors[key_point] = individual_joint
        return descriptors

    def get_pos_esqueleto(self, person, list):

        # Calculating middle points for eyes and hips

        puntoMedioX = (person.joints[list[0]].i + person.joints[list[1]].i) / 2.0
        puntoMedioY = (person.joints[list[0]].j + person.joints[list[1]].j) / 2.0

        z0 = person.joints[list[0]].z
        z1 = person.joints[list[1]].z

        if z0 != 0:
            if abs(z0 - z1) < 0.5:
                distance = (z0 + z1) / 2.0
                # print("puntos ", z0, z1, "dist", distance)
            else:
                # print("fallo poniendo punto ", z0)
                distance = z0
        else:
            distance = 0
        return puntoMedioX, puntoMedioY, distance

    def proceso_esqueleto(self, image, people_data):
        # Draw body parts on image

        for person in people_data.peoplelist:
            for name1, name2 in self.human_pose["skeleton"]:
                try:
                    # Printing bones
                    # if str(name1) in list(person.joints.keys()) and str(name2) in list(person.joints.keys()):
                    #    print(name1, name2)
                    joint1 = person.joints[str(name1)]
                    joint2 = person.joints[str(name2)]
                    # print(joint1.i, joint2.j)
                    cv2.line(image, (joint1.i, joint1.j), (joint2.i, joint2.j), (0, 255, 0), 2)
                except:
                    pass

        # compute bounding box
        faceList = ["2", "3"]
        hipList = ["12", "13"]
        chestList = ["6", "7"]

        people_pos_data = []

        # descriptors = {}
        # descriptors_dot_products = {}
        # dot_product_acum = []

        if len(people_data.peoplelist) > 0:
            for person in people_data.peoplelist:
                faceNameList = []
                hipNameList = []
                chestNameList = []

                # people_dict = {
                #     "id" : len(self.people),
                #     "descriptors" : {}
                # }

                for key_point in list(person.joints.keys()):
                    # if person.joints[key_point].floatdesclist:
                    #     descriptors[key_point] = person.joints[key_point].floatdesclist[0] / np.linalg.norm(person.joints[key_point].floatdesclist[0])
                    #     if key_point in list(self.last_descriptors.keys()):
                    #         descriptors_dot_products[key_point] = np.dot(self.last_descriptors[key_point], descriptors[key_point])
                    #         dot_product_acum.append(descriptors_dot_products[key_point])
                    if key_point in faceList:
                        faceNameList.append(key_point)
                    if key_point in hipList:
                        hipNameList.append(key_point)
                    if key_point in chestList:
                        chestNameList.append(key_point)

                # Create dictionarý to append some skeleton data
                distances = {}
                distance_list = []

                # print(faceNameList)
                # print(hipNameList)
                # print(chestNameList)

                puntoMedioX = -1

                if len(hipNameList) == 2:
                    distances["hips"] = (self.get_pos_esqueleto(person, hipNameList))
                    cv2.circle(image, (int(distances["hips"][0] - 10), int(distances["hips"][1] - 10)), 10, (0, 255, 150), 2)
                    puntoMedioX = distances["hips"][0]

                if len(chestNameList) == 2:
                    distances["chest"] = (self.get_pos_esqueleto(person, chestNameList))
                    cv2.circle(image, (int(distances["chest"][0] - 10), int(distances["chest"][1] - 10)), 10, (0, 150, 255), 2)
                    puntoMedioX = distances["chest"][0]

                if len(faceNameList) == 2:
                    distances["face"] = (self.get_pos_esqueleto(person, faceNameList))
                    cv2.circle(image, (int(distances["face"][0] - 10), int(distances["face"][1] - 10)), 10, (255, 0, 0), 2)
                    puntoMedioX = distances["face"][0]

                person_coords = self.get_coords(person)
                if person_coords != None:
                    if puntoMedioX > -1:
                        person_coords["x_pixel"] = puntoMedioX
                        people_pos_data.append(person_coords)
                    else:
                        person_coords["x_pixel"] = -1



                for dist in distances.values():
                    if dist[2] != 0:
                        distance_list.append(dist[2])
                if len(distance_list) > 0:
                    # self.soundrotation_proxy.personFound(True)
                    distance_average_local = sum(distance_list) / len(distance_list)
                    # print("Valor distancia promedio: ", distance_average_local)
                else:
                    distance_average_local = 0
                    try:
                        # self.soundrotation_proxy.personFound(False)
                        self.differentialrobot_proxy.setSpeedBase(0, 0)
                        # goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
                        # goal.maxSpeed = MIN_VELOCITY*5
                        # goal.position = 0
                        # self.jointmotorsimple_proxy.setPosition("", goal)
                    except:
                        pass
                        # print("Activa la base")

                # Filtering distances
                # self.distance_coefficients.append(distance_average_local)
                # self.distance_coefficients.pop(0)
                # self.distance_avg = sum(self.distance_coefficients) / len(self.distance_coefficients)
                # print("Distansia pecho promediá: ", self.distance_avg)
        return image, people_pos_data

    def draw_error_data(self):
        self.d_camera_position_error.extend([self.error_dict["camera_position_error"][-1]])
        self.dx.extend([self.x_data])
        self.camera_position_error.set_ydata(self.d_camera_position_error)
        self.camera_position_error.set_xdata(self.dx)

        # set axes
        self.ah1.set_ylim(-1, 1)
        self.ah1.set_xlim(self.x_data-self.visible, self.x_data)

        self.x_data += 1

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def tracker_camera(self, color, puntoMedioX):
        error = puntoMedioX - color.width / 2
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        der_error = -(error - self.error_ant)
        error_rads = np.arctan2(self.k1 * error + self.k2 * der_error, color.focalx)
        # goal_rad = self.motor.pos - error_rads - self.rotational_speed_avg * (self.Period_camera/1000)
        goal_rad = self.motor.pos - error_rads

        # Se mueve el sujeto?
        if self.rad_old > goal_rad + TOLERANCE or self.rad_old < goal_rad - TOLERANCE:
            print("camera moving")
            rad_seg = abs(((self.rad_old - goal_rad) / self.Period_camera) * 1000)  # rad/s
            # rad_seg = 9*error_rads
            # filtramos velicidad, ya que 0 es maxima
            if abs(error_rads) > 0.3:
                goal.maxSpeed = 0
            else:
                if abs(rad_seg) < MIN_VELOCITY:
                    goal.maxSpeed = MIN_VELOCITY
                else:
                    goal.maxSpeed = rad_seg
            goal.position = goal_rad
            # mandamos al motor el objetivo
            self.jointmotorsimple_proxy.setPosition("", goal)

            pos_error = self.motor.pos - self.last_motor_pos
            self.rotational_speed_coefficients.append(-self.k3 * self.motor.pos + self.k4 * pos_error)
            self.rotational_speed_coefficients.pop(0)
            self.rotational_speed_avg = sum(self.rotational_speed_coefficients) / len(
                self.rotational_speed_coefficients)
            # print("COEFICIENTES BASE: ", self.rotational_speed_coefficients)
            # print("VELOCIDAD DE LA BASE: ", self.rotational_speed_avg)
            self.last_motor_pos = self.motor.pos

            # print("DISTANCIA: ", self.distance_avg)
            # print("DIFERENCIA DISTANCIA: ", self.distance_avg - self.distance_limit)

            # print("Velocidad rotacional: ", self.rotational_speed_avg)
            # print("Valor gausiana rotacional: ", math.exp(5*-(self.rotational_speed_avg ** 2)))
            # print("Valor coeficiente distancia: ", (2 / (1 + math.exp(-(self.distance_avg - self.distance_limit)))) - 1)

            base_speed_lin = 700 * ((2 / (1 + math.exp(-(self.distance_avg - self.distance_limit) * self.k6))) - 1) * (math.exp(self.k7 * (self.rotational_speed_avg ** 2)))

            # Filtering lineal speeds
            self.lineal_speed_coefficients.append(base_speed_lin)
            self.lineal_speed_coefficients.pop(0)
            self.lineal_speed_avg = sum(self.lineal_speed_coefficients) / len(self.lineal_speed_coefficients)

            # print(goal_rad, self.rad_old, rad_seg)
            self.error_ant = error
            self.rad_old = goal_rad

            # print("ERROR CON RESPECTO AL CENTRO: ", error_rads)
            # print("DERIVADA DEL ERROR: ", der_error)
            # print("POSICIÓN DEL MOTOR: ", self.motor.pos)
            # print("VELOCIDAD DE CÁMARA: ", goal.maxSpeed)

            self.error_dict["camera_position_error"].append(self.motor.pos - goal_rad)

    def tracker_base(self):

        # print("VELOCIDAD ROTACIONAL DE LA BASE: ", self.rotational_speed_avg)
        # print("VELOCIDAD LINEAL DE LA BASE: ", self.lineal_speed_avg)
        try:
            # self.differentialrobot_proxy.setSpeedBase(self.lineal_speed_avg, self.rotational_speed_avg)
            self.differentialrobot_proxy.setSpeedBase(0, self.rotational_speed_avg)
        except:
            pass
        #     # print("Conecta la base")

    def refesco_ventana(self, color, image):
        qt_image = QImage(image, color.height, color.width, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qt_image).scaled(self.ui.label_image.width(), self.ui.label_image.height())
        self.ui.label_image.setPixmap(pix)
        # image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)

        self.ui.lcdNumber_pos.display(self.motor.pos)
        self.ui.lcdNumber_speed.display(self.motor.vel)
        self.ui.lcdNumber_temp.display(self.motor.temperature)
        self.ui.lcdNumber_max_speed.display(self.current_max_speed)
        if self.motor.isMoving:
            self.ui.radioButton_moving.setChecked(True)
        else:
            self.ui.radioButton_moving.setChecked(False)

        # self.graph_tick()

    @QtCore.Slot()
    def slot_change_pos(self, pos):   # comes in degrees -150 .. 150. Sent in radians -2.62 .. 2.62
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        goal.position = (2.62/150.0)*pos
        goal.maxSpeed = self.current_max_speed
        self.jointmotorsimple_proxy.setPosition("", goal)

    @QtCore.Slot()
    def slot_change_max_speed(self, max_speed):
        self.current_max_speed = max_speed*0.111/60*np.pi*2.0

    @QtCore.Slot()
    def slot_center(self):
        self.ui.horizontalSlider_pos.setSliderPosition(0)
        self.slot_change_pos(0)

    @QtCore.Slot()
    def slot_track(self):
        self.track = not self.track
        if not self.track:
            print("Guardando")
            with open("error_data.json", 'w+') as file:
                json.dump(self.error_dict, file, indent=4)
        print("state track", self.track)

    def graph_tick(self):
        m_y = QRandomGenerator.global_().bounded(5) - 2.5
        self.m_x += 20
        self.track_error.append(self.m_x, m_y)
        if self.m_x > 1000:
            self.chart.scroll(5, 0)


#####################################################################################################
    def startup_check(self):
        print(f"Testing RoboCompJointMotorSimple.MotorState from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorState()
        print(f"Testing RoboCompJointMotorSimple.MotorParams from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorParams()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalPosition from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalVelocity from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalVelocity()
        QTimer.singleShot(200, QApplication.instance().quit)

    ######################
    # From the RoboCompJointMotorSimple you can call this methods:
    # self.jointmotorsimple_proxy.getMotorParams(...)
    # self.jointmotorsimple_proxy.getMotorState(...)
    # self.jointmotorsimple_proxy.setPosition(...)
    # self.jointmotorsimple_proxy.setVelocity(...)
    # self.jointmotorsimple_proxy.setZeroPos(...)

    ######################
    # From the RoboCompJointMotorSimple you can use this types:
    # RoboCompJointMotorSimple.MotorState
    # RoboCompJointMotorSimple.MotorParams
    # RoboCompJointMotorSimple.MotorGoalPosition
    # RoboCompJointMotorSimple.MotorGoalVelocity


    #image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        #print(len(faces))
        # for (x, y, w, h) in faces:
        #     cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        #     error = x+(w/2.0) - color.width/2.0
        #     # # # error en pixels to radians
        #     goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        #     #goal.position = np.arctan2(error, color.focalx)
        #     goal.position = motor.pos + (error / 800)
        #     print(goal)
            #self.jointmotorsimple_proxy.setPosition("", goal)
        #cv2.imshow("", image)

        # try:
        #     skeleton = self.humancamerabody_proxy.newPeopleData()
        #     print(skeleton)
        # except:
        #     print("Ice error communicating with HumanCameraBody interface")