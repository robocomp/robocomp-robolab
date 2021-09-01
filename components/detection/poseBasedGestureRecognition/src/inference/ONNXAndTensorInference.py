import torch
import numpy as np
from .custom_augmentation import InferenceTransformation
from .onnx_inference import PoseDetectionONNX
from .onnx_with_tensorrt import PoseDetectionONNXTensorRT
from .constant import *



class PoseBasedRecognitionONNXInference:
    def __init__(self, pretrained_weight, num_class = 100):
        self.model = PoseDetectionONNX(pretrained_weight)
        self.num_class = num_class

    def __call__(self, pose):
        # preprocess pose if needed

        # inference sequence of image
        perframe_logits = self.model(pose) # 1 X 55 X (num_samplex2)
        perframe_logits = perframe_logits.reshape((1, self.num_class))

        # get top 5
        predictions = np.max(perframe_logits, axis=0)[0]
        out_label = np.argsort(predictions)
        out_prob = np.sort(predictions)

        return [out_label[:5], out_prob[:5]]


class IPoseBasedRecognitionONNXTensorRTInference:
    def __init__(self, pretrained_weight, num_class = 100):
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.model = PoseDetectionONNXTensorRT(pretrained_weight, "./openlight.trt")
        self.num_class = num_class

    def __call__(self, pose):

        # inference sequence of image
        perframe_logits = self.model(pose) # 1 X 64 X 100
        perframe_logits = perframe_logits.reshape((1, self.num_class))

        # get top 5
        predictions = np.max(perframe_logits, axis = 0)[0]
        out_label = np.argsort(predictions)
        out_prob = np.sort(predictions)

        return [out_label[:5], out_prob[:5]]



