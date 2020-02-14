import numpy as np
import cv2

mpii_edges = [[0, 1], [1, 2], [2, 6], [6, 3], [3, 4], [4, 5], 
              [10, 11], [11, 12], [12, 8], [8, 13], [13, 14], [14, 15], 
              [6, 8], [8, 9]]

def show_2d(img, points, c, edges):
  num_joints = points.shape[0]
  points = ((points.reshape(num_joints, -1))).astype(np.int32)
  for j in range(num_joints):
    cv2.circle(img, (points[j, 0], points[j, 1]), 3, c, -1)
  for e in edges:
    if points[e].min() > 0:
      cv2.line(img, (points[e[0], 0], points[e[0], 1]),
                    (points[e[1], 0], points[e[1], 1]), c, 2)
  return img

class Visualizer(object):
  def __init__(self, edges=mpii_edges):
    self.imgs = {}
    self.edges=edges
   
  def add_img(self, img, imgId = 'default'):
    self.imgs[imgId] = img.copy()

  def add_point_2d(self, point, c, imgId='default'):
    self.imgs[imgId] = show_2d(self.imgs[imgId], point, c, self.edges)
  
  def show_all_imgs(self, pause = False):
    for i, v in self.imgs.items():
      cv2.imshow('{}'.format(i), v)
    if pause:
      cv2.waitKey()
    
