'''
Script to save the dataset/scene for getting 
dual RGB camera data.
The camera image camera data is saved in separate folders
for the cameras viewing a partially overlapping scene. 
'''

import glob
import os
import sys
import random
import time
import numpy as np
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import atexit

def destroy_sensors():
    '''
    Destroy sensors that were initiated
    '''
    camera1.destroy()
    camera2.destroy()

atexit.register(destroy_sensors)

sensor_width = 1920
sensor_height = 1080
sensor_fov = 90
sensor_tick = 0.5
uptime = 40
start_time = time.time()

while True and (time.time()-start_time < uptime):
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()
        ## setting up sensor/camera
        cam = blueprint_library.find('sensor.camera.rgb')
        cam.set_attribute('image_size_x', f'{sensor_width}')
        cam.set_attribute('image_size_y', f'{sensor_height}')
        cam.set_attribute('fov', f'{sensor_fov}')
        cam.set_attribute('sensor_tick', f'{sensor_tick}')
        
        ## specifying the sensors poses
        spawn_point1 = carla.Transform(carla.Location(x=-40, y=-5, z=9.4),carla.Rotation(yaw=-25, pitch=0, roll=0))
        spawn_point2 = carla.Transform(carla.Location(x=-40, y=-5, z=9.4),carla.Rotation(yaw=5, pitch=0, roll=0))

        ## initialising the cameras
        camera1 = world.try_spawn_actor(cam, spawn_point1, attach_to=None)
        camera2 = world.try_spawn_actor(cam, spawn_point2, attach_to=None)
        if camera1 is not None:
            print("[Camera-1 Initiated]")
        if camera2 is not None:
            print("[Camera-2 Initiated]")

        camera1.listen(lambda image: image.save_to_disk('./cam1/%03d.png' % image.frame_number))
        camera2.listen(lambda image: image.save_to_disk('./cam2/%03d.png' % image.frame_number))
         
    except:
        print("Failed! :(")
    
destroy_sensors()








