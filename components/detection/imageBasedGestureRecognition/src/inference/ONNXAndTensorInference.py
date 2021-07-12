import torch
import numpy as np
from .custom_augmentation import InferenceTransformation
from .onnx_inference import PoseDetectionONNX
from .onnx_with_tensorrt import PoseDetectionONNXTensorRT
from .constant import *


def preprocess_tensor(image):
    x_data = image.astype('f')
    x_data /= 255
    x_data -= 0.5
    x_data = x_data.transpose(3, 0, 1, 2)
    x_data = x_data[None,:,:,:,:] # 1 x 3 x n x h x w
    return x_data


class ImageBasedRecognitionONNXInference:
    def __init__(self, pretrained_weight):
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.model = PoseDetectionONNX(pretrained_weight)

    def __call__(self, images):
        batch_image = []
        # preprocess image
        height, width, _ = images[0].shape # get shape from an image.
        for img in images:
            batch_image.append(self.preprocess(img)[None,:,:,:])
        image = np.concatenate(batch_image, axis= 0)
        image = preprocess_tensor(image)

        # inference sequence of image
        perframe_logits = self.model(image) # 1 X 64 X 100
        perframe_logits = perframe_logits.reshape((1, len(images), self.num_class))

        # get top 5
        predictions = np.max(perframe_logits, axis=0)[0]
        out_label = np.argsort(predictions)
        out_prob = np.sort(predictions)

        return out_prob[:5],out_label[:5]


class ImageBasedRecognitionONNXTensorRTInference:
    def __init__(self, pretrained_weight, num_class = 100):
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.model = PoseDetectionONNXTensorRT(pretrained_weight, "./openlight.trt")
        self.num_class = num_class

    def __call__(self, images):
        batch_image = []
        # preprocess image
        height, width, _ = images[0].shape # get shape from an image.
        for img in images:
            batch_image.append(self.preprocess(img)[None,:,:,:])
        image = np.concatenate(batch_image, axis= 0)
        image = preprocess_tensor(image)

        # inference sequence of image
        perframe_logits = self.model(image) # 1 X 64 X 100
        perframe_logits = perframe_logits.reshape((1, len(images), self.num_class))

        # get top 5
        predictions = np.max(perframe_logits, axis = 0)[0]
        out_label = np.argsort(predictions)
        out_prob = np.sort(predictions)

        return out_prob[:5], out_label[:5]



