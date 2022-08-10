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


from rich.console import Console
from genericworker import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

import numpy as np
from numpy.random import choice as CColor # ChooseColor
import os
import time
import cv2
from colored import fg
import json
import trt_pose.coco
import trt_pose.models
import copy
import torch
import torch2trt
from torch2trt import TRTModule
import torchvision.transforms as transforms
import PIL.Image
from trt_pose.parse_objects import ParseObjects
sys.path.append('/usr/local/lib/python3.6/pyrealsense2')
import pyrealsense2.pyrealsense2 as rs
device = torch.device('cuda')

# Parametros
PMcR = [i for i in range(111, 231+1)] # Print Multicolor Range

# Colors
B = fg(250)
A = fg(14)
N = fg(208)
G = fg(244)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        #random_color = fg(CColor(PMcR))
        #print(random_color+"=======================  " +fg(15)+  "__init__"  +random_color+ "  =======================" + G)

        self.path_data = "/home/robo02/jetson_humanbody_data"

        self.model = 'densenet'
        self.IMPUT_WIDTH  = 640
        self.INPUT_HEIGHT = 480
        self.draw = True
        self.rotate = True

        # Vars para luego
        self.pipeline = None
        self.profile = None
        self.depth_profile = None
        self.MODEL_WIDTH  = None
        self.MODEL_HEIGHT = None
        self.mean = None
        self.std = None
        self.model_trt = None
        self.parse_objects = None
        self.peoplelist = None
        self.source_img = None
        self.skeleton_img = None
        self.human_pose = None
        self.skeleton_points = []
        self.skeleton_white = []

        # Tiempos
        self.t1 = None
        self.t2 = None
        self.t3 = None
        self.t4 = None

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True

    def compute(self):
        
        #random_color = fg(CColor(PMcR))
        #print(random_color+"=======================  " +fg(15)+  "Compute: Init"  +random_color+ "  =======================" + G)
        frames_num = 0
        
        # Inicializamos la red y los elementos necesarios para mas adelante
        self.initializePilar()

        random_color = fg(CColor(PMcR))
        print(random_color+"=======================  " +fg(15)+  "Compute: While"  +random_color+ "  =======================" + G)
        while True:
            init_time = time.time()
            # Obtenemos los frames, los procesamos y preparamos
            #self.t3 = time.time()
            self.peoplelist = self.computePilar()
            #self.t4 = time.time()

            
            #PP = RoboCompHumanCameraBody.PeopleData()
            #self.HumanCameraBody_newPeopleData(PP)

            #print(PP.__dict__)
            #exit()

            
            #print("total t: ", str(self.t4-self.t3), "  cam t: ", str(self.t2-self.t1), "  fps: ", str(round(1/(self.t4-self.t3), 1)))


            # prints de info estatica
            #sys.stdout.write("\r                                                            \r"+N+"Numero de Personas: " +A+ str(len(self.peoplelist)) +N+ "            " + "Frames:"+A+ str(frames_num) +B)
            #sys.stdout.flush()
            #frames_num += 1
            #time.sleep(0.05)
            end_time = time.time()
            print("////////////// tiempo de compute: ", end_time - init_time)
        return True

    def startup_check(self):
        pass


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getImage method from GiraffJetson interface
    #
    def GiraffJetson_getImage(self, drawSkeleton):

        if drawSkeleton:
            return json.dumps({"list": self.skeleton_img.tolist()})
        else:
            return json.dumps({"list": self.source_img.tolist()})
        


    #
    # IMPLEMENTATION of getSkeleton method from GiraffJetson interface
    #
    def GiraffJetson_getSkeleton(self):
    
        #
        # write your CODE here
        #
        return json.dumps(self.peoplelist)


    # Metodos de CameraRGBD_Simple
    def CameraRGBDSimple_getAll(self, camera):
        
        return None
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        ret = RoboCompCameraRGBDSimple.TDepth()
        #
        # write your CODE here
        #
        return self.depth_img.data
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getImage(self, camera):
        """
        Devuelve la imagen rgb que ve la camara.
        """
        print("////////////// GETTING IMAGE //////////////")
        self.t_init_compute = time.time()
        ret = RoboCompCameraRGBDSimple.TImage(
            compressed=True,
            cameraID=0,
            width=self.skeleton_img.shape[0],
            height=self.skeleton_img.shape[1],
            depth=3,
            focalx=0,
            focaly=0,
            alivetime=0,
            image=self.skeleton_img.tobytes()
        )

        # ret = RoboCompCameraRGBDSimple.TImage(
        #     cameraID=0,
        #     width=self.skeleton_white.shape[0],
        #     height=self.skeleton_white.shape[1],
        #     depth=3,
        #     focalx=0,
        #     focaly=0,
        #     alivetime=0,
        #     image=self.skeleton_white.tobytes()
        # )

        
        self.t_end_compute = time.time()
        print("////////////// tiempo de get image: ", self.t_end_compute - self.t_init_compute)
        return ret




    def HumanCameraBody_newPeopleData(self):
        """
        Modifica la estructura people y la rellena con los 
        datos de personas detectadas con la camara.
        """
        
        if self.peoplelist == None:
            return None

        people = RoboCompHumanCameraBody.PeopleData()

        # Guardamos la marca de tiempo
        people.timestamp = time.time()


        # Montamos la PeopleList (lista de Person)
        new_people_list = []
        for i, p in enumerate(self.peoplelist):
            # Generamos una instancia de Person por cada persona que haya detectado la red
            new_person = RoboCompHumanCameraBody.Person()
            
            new_person.id = i

            # Montamos el contenedor de joints
            TJoints = {}
            for key in p:
                dict_joint = p[key]
                key_point = RoboCompHumanCameraBody.KeyPoint()

                # Coordenadas en la imagen
                key_point.i = dict_joint[2][0]
                key_point.j = dict_joint[2][1]

                # Coordenadas del mundo
                key_point.x = dict_joint[3][0]
                key_point.y = dict_joint[3][1]
                key_point.z = dict_joint[3][2]
            
                # Añadimos al dict de joints de la persona
                TJoints[str(key)] = key_point
            
            # Adjuntamos el dict de joints a la persona en cuestion
            new_person.joints = TJoints

            # Añadimos la nueva persona a la lista de people
            new_people_list.append(new_person)
        
        # Adjuntamos la lista de personas a la estructura people
        people.peoplelist = new_people_list

        return people












    #########################################################################################
    #########################################################################################
    """                                     INITIALIZE                                    """
    #########################################################################################
    #########################################################################################

    def initializePilar(self):
        #random_color = fg(CColor(PMcR))
        #print(random_color+"=======================  " +fg(15)+  "initializePilar"  +random_color+ "  =======================" + G)
        # Cargamos el json de huesos y preparamos la red elegida para obtener el esqueleto
        with open('human_pose.json', 'r') as f:
            self.human_pose = json.load(f)

        topology = trt_pose.coco.coco_category_to_topology(self.human_pose)
        num_parts = len(self.human_pose['keypoints'])
        num_links = len(self.human_pose['skeleton'])

        if self.model == 'resnet':
            #print('model = densenet')
            MODEL_WEIGHTS = '../resnet18_baseline_att_224x224_A_epoch_249.pth'
            OPTIMIZED_MODEL = 'resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
            model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
            self.MODEL_WIDTH = 224.
            self.MODEL_HEIGHT = 224.
        elif self.model == 'densenet':
            #print('model = densenet')
            MODEL_WEIGHTS = '../densenet121_baseline_att_256x256_B_epoch_160.pth'
            OPTIMIZED_MODEL = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'
            model = trt_pose.models.densenet121_baseline_att(num_parts, 2 * num_links).cuda().eval()
            self.MODEL_WIDTH = 256.
            self.MODEL_HEIGHT = 256.


        

        # Preparamos un modelo optimizado si no existe. Si si existe, solo lo cargamos
        data = torch.zeros((1, 3, int(self.MODEL_HEIGHT), int(self.MODEL_WIDTH))).cuda()
        if os.path.exists(OPTIMIZED_MODEL) == False:
            #print("Preparando modelo optimizado. Espere...")
            model.load_state_dict(torch.load(MODEL_WEIGHTS))
            self.model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1<<25)
            torch.save(self.model_trt.state_dict(), OPTIMIZED_MODEL)

        self.model_trt = TRTModule()
        self.model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))



        # Popurri extraño quer aun no se sabe que hace :p
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()


        self.parse_objects = ParseObjects(topology)


        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.IMPUT_WIDTH, self.INPUT_HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.IMPUT_WIDTH, self.INPUT_HEIGHT, rs.format.z16, 30)
        if self.rotate:
            self.IMPUT_WIDTH, self.INPUT_HEIGHT = self.INPUT_HEIGHT, self.IMPUT_WIDTH

        try:
            self.pipeline.start(config)
        except:
            import tkinter as tk
            from tkinter import messagebox
            root = tk.Tk()
            root.withdraw()
            messagebox.showwarning('Atencion', '[!] La camara no esta conectada!')
            exit("[!] La camara no esta conectada!")
        self.profile = self.pipeline.get_active_profile()
        #print(self.profile)
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        #print(self.depth_profile)
        depth_intrinsics = self.depth_profile.get_intrinsics()
        #print(depth_intrinsics)



        depth_to_color_extrin =  self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.color))
        color_to_depth_extrin =  self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.depth))

        last_sent = time.time()


    def draw_arrow(self, image, i_w, i_h, ratio, xa, ya, xC, yC, xb, yb):
        tlx = int(ratio*(xa))+int(i_w/2)
        tly = int(i_h)-int(ratio*(ya))
        brx = int(ratio*(xb))+int(i_w/2)
        bry = int(i_h)-int(ratio*(yb))
        mx = int(ratio*(xC))+int(i_w/2)
        my = int(i_h)-int(ratio*(yC))
        cv2.line(image, (tlx,tly),( mx, my),(255,0,0),3)
        cv2.line(image, (mx, my),(brx,bry),(0,0,255),3)


    def preprocess(self, image):
        global device
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def execute(self, img, src):
        data = self.preprocess(img)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
        return counts, objects, peaks



    def get_keypoint(self, humans, hnum, peaks):
        kpoint = []
        human = humans[0][hnum]
        C = human.shape[0]
        for j in range(C):
            k = int(human[j])
            if k >= 0:
                peak = peaks[0][j][k]   # peak[1]:width, peak[0]:height
                peak = (j, float(peak[0]), float(peak[1]))
                kpoint.append(peak)
            else:
                peak = (j, None, None)
                kpoint.append(peak)
        return kpoint



    def cam_matrix_from_intrinsics(self, i):
        return np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])





    #########################################################################################
    #########################################################################################
    """                                       COMPUTE                                     """
    #########################################################################################
    #########################################################################################

    def computePilar(self):
        ##random_color = fg(CColor(PMcR))
        ##print(random_color+"=======================  " +fg(15)+  "computePilar"  +random_color+ "  =======================" + G)

        global last_sent
        data_to_publish = []
        #self.t1 = time.time()
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        # Rotamos
        color_image = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        depth_frame = frames.get_depth_frame()
        depth_data = np.asanyarray(depth_frame.get_data())
        # Rotamos
        depth_data = cv2.rotate(depth_data, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # self.skeleton_image = np.zeros([480,640,1],dtype=np.float32)
        

        self.depth_img = copy.deepcopy(depth_data)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        depth_to_color_extrin =  self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(self.profile.get_stream(rs.stream.color))
        color_to_depth_extrin =  self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(self.profile.get_stream(rs.stream.depth))

        #self.t2 = time.time()
        
        r_s = 0
        r_w = self.INPUT_HEIGHT
        c_s = 0
        c_w = self.IMPUT_WIDTH

        cropped = color_image[int(r_s):int(r_s+r_w), int(c_s):int(c_s+c_w)]
        img = cv2.resize(cropped, dsize=(int(self.MODEL_WIDTH), int(self.MODEL_HEIGHT)), interpolation=cv2.INTER_AREA)
        self.source_img = img # candidata para enviar al componente remoto
        counts, objects, peaks = self.execute(img, color_image)


        if self.draw:
            self.skeleton_img = copy.deepcopy(color_image)
            color = (0,255,0)
            for i in range(counts[0]):
                keypoints = self.get_keypoint(objects, i, peaks)
                for j in range(len(keypoints)):
                    if keypoints[j][1]:
                        x = keypoints[j][2]*c_w + c_s
                        y = keypoints[j][1]*r_w + r_s
                        if self.draw:
                            cv2.circle(self.skeleton_img, (int(x), int(y)), 1, color, 2)
                            cv2.putText(self.skeleton_img , "%d" % int(keypoints[j][0]), (int(x) + 5, int(y)),  cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
            

 


        def c2d(p):
            x,y = p
            depth_min = 0.1
            depth_max = 10.
            depth_point = rs.rs2_project_color_pixel_to_depth_pixel(depth_frame.get_data(), depth_scale, depth_min, depth_max, depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x,y])
            depth_point[0] = int(depth_point[0]+0.5)
            depth_point[1] = int(depth_point[1]+0.5)
            return depth_point

        def d2xyz(depth_point):
            depth_min = 0.01
            depth_max = 2.
            if np.any(depth_point == None):
                return False, [0,0,0]
            try:
                ret = True, rs.rs2_deproject_pixel_to_point(depth_intrin, depth_point, depth_scale*depth_data[depth_point[1], depth_point[0]])
                return ret
            except IndexError:
                return False, [0,0,0]


        keypoints_names = self.human_pose["keypoints"]
        for i in range(counts[0]):
            keypoints = self.get_keypoint(objects, i, peaks)
            kps = dict()
            for kp in range(len(keypoints)):
                centre = keypoints[kp]
                if centre[1] and centre[2]:
                    cx = centre[2]*c_w + c_s
                    cy = centre[1]*r_w + r_s
                    #cx = round(centre[2]* self.IMPUT_WIDTH) # + c_s
                    #cy = round(centre[1]* self.INPUT_HEIGHT)# + r_s
                    got3d, c = d2xyz(c2d([int(round(cx-3)),int(round(cy+2))]))
                    kps[kp] = [kp, keypoints_names[kp], [cx, cy], c + [float(got3d)] ]
                    if self.draw:
                        vector1 = np.array([c]).transpose()
                        K = self.cam_matrix_from_intrinsics(color_intrin)
                        vector2 = K @ vector1
                        vector2 /= vector2[2][0]
                        try:
                            cv2.circle(self.skeleton_img, (int(vector2[0][0])-1, int(vector2[1][0])-1), 1, (0,100,255), 2)
                        except:
                            pass
            data_to_publish.append(kps)

        


        return data_to_publish
