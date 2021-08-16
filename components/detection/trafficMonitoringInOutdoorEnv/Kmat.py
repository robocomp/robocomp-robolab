'''
Class for creating the intrinsic matrix K
used for obtaing the projected data. 
Takes as input the dimensions and FOV of the 
dataset and return the K and K_inverse.
'''
import numpy as np
import atexit
import math

class intrinsic:
    def __init__(self, ImageSizeX=1920, ImageSizeY=1080, CameraFOV=90):
        self.ImageSizeX = ImageSizeX
        self.ImageSizeY = ImageSizeY
        self.CameraFOV = CameraFOV
        self.Cu = ImageSizeX / 2
        self.Cv = ImageSizeY / 2
        self.f = ImageSizeX /(2 * math.tan(CameraFOV * math.pi / 360))
        self.K = np.array([[self.f, 0, self.Cu],[0, self.f, self.Cv],[0, 0, 1 ]])

    def inverse(self):
        K_inv = np.linalg.inv(self.K)
        return K_inv

if __name__ == '__main__':
    K = intrinsic()
    K_inv = K.inverse()
    print(K_inv)