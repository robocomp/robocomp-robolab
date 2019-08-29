import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), './pytorch-pose-hg-3d/src'))
import _init_paths
import torch
from models.msra_resnet import get_pose_net
from utils.image import get_affine_transform, transform_preds
from utils.eval import get_preds, get_preds_3d
import cv2
import numpy as np


class PoseEstimator:
  arch = "msra_50"
  num_layers = int(arch[arch.find('_') + 1:])
  heads = {'hm': 16, 'depth': 16}

  input_w = 256
  input_h = 256

  output_w = 64
  output_h = 64

  mean = np.array([0.485, 0.456, 0.406], np.float32).reshape(1, 1, 3)
  std = np.array([0.229, 0.224, 0.225], np.float32).reshape(1, 1, 3)

  def __init__(self, model_path=os.path.join(os.path.dirname(__file__), './pytorch-pose-hg-3d/models/fusion_3d_var.pth'), device='cuda:0'):
    self.device = device
    self.model = self.load_model(model_path, device)

  def load_model(self, model_path, device):
    model = get_pose_net(PoseEstimator.num_layers, PoseEstimator.heads)
    checkpoint = torch.load(
      model_path, map_location=lambda storage, loc: storage)
    if isinstance(checkpoint, dict):
      state_dict = checkpoint['state_dict']
    else:
      state_dict = checkpoint.state_dict()
    model.load_state_dict(state_dict, strict=False)
    model = model.to(device)
    model.eval()
    return model

  def estimate(self, image):
    if isinstance(image, str):
      image = cv2.imread(image)
    inp, c, s = self.processImage(image)
    inp = torch.from_numpy(inp).to(self.device)
    out = self.model(inp)[-1]
    pred = get_preds(out['hm'].detach().cpu().numpy())[0]
    pred = transform_preds(pred, c, s, (self.output_w, self.output_h))
    pred_3d = get_preds_3d(out['hm'].detach().cpu().numpy(),
                           out['depth'].detach().cpu().numpy())[0]
    return pred, pred_3d

  def processImage(self, image):
    s = max(image.shape[0], image.shape[1]) * 1.0
    c = np.array([image.shape[1] / 2., image.shape[0] / 2.], dtype=np.float32)
    trans_input = get_affine_transform(
        c, s, 0, [self.input_w, self.input_h])
    inp = cv2.warpAffine(image, trans_input, (self.input_w, self.input_h), flags=cv2.INTER_LINEAR)
    inp = (inp / 255. - self.mean) / self.std
    inp = inp.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)
    return inp, c, s


# estimator = PoseEstimator(device='cpu')
# image_path = "./pytorch-pose-hg-3d/my-images/images.jpg"
# poitnts2d, points = estimator.estimate(image_path)
# points.reshape(-1, 3)
# x, y, z = np.zeros((3, points.shape[0]))
# for j in range(points.shape[0]):
#   x[j] = points[j, 0].copy()
#   y[j] = points[j, 2].copy()
#   z[j] = - points[j, 1].copy()
# print(points)
# print(x)
# print(y)
# print(z)

# right_foot
# right_knee
# right_hip
# left_hip
# left_knee
# left_foot
# between_hips
# neck
# neck
# head
# right_hand
# right_elbow
# right_shoulder
# left_shoulder
# left_elbow
# left_hand


