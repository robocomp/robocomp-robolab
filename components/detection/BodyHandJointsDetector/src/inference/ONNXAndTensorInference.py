import torch
from .custom_augmentation import InferenceTransformation
from .pose import Pose
from .utils_mediapipe import HandPoseDetector
from .constant import *
from .onnx_inference import PoseDetectionONNX
from .onnx_with_tensorrt import PoseDetectionONNXTensorRT


def preprocess_tensor(image):
    x_data = image.astype('f')
    x_data /= 255
    x_data -= 0.5
    x_data = x_data.transpose(2, 0, 1)
    return x_data


class BodyDetectorONNXInference:
    def __init__(self, pretrained_weight):
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.pose_parser = Pose(image_scale=0.125)
        self.hand_model = HandPoseDetector(static_model=True)
        self.model = PoseDetectionONNX(pretrained_weight)

    def __call__(self, origin_image):
        # preprocess image
        height, width, _ = origin_image.shape
        ratio_scale = height / FIX_SIZE
        add_width = (FIX_SIZE - int(FIX_SIZE * width / height)) // 2
        image = self.preprocess(origin_image)
        image = preprocess_tensor(image)

        # inference single image
        paf, heatmap = self.model(image)
        paf = paf.reshape((38, 40, 40))
        heatmap = heatmap.reshape((19, 40, 40))

        # parse paf and heatmap, and hand skeleton
        self.pose_parser.parser_pose(paf, heatmap)
        hand_images = self.pose_parser.get_hand_head_images(origin_image, ratio_scale, add_width)
        hand_skeleton = None
        if hand_images is not None:
            _, hand_skeleton = self.hand_model(hand_images)
        return self.pose_parser.postprocess_pose(hand_skeleton, ratio_scale, add_width)


class BodyDetectorONNXTensorRTInference:
    def __init__(self, pretrained_weight):
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.pose_parser = Pose(image_scale=0.125)
        self.hand_model = HandPoseDetector(static_model=True)
        self.model = PoseDetectionONNXTensorRT(pretrained_weight, "./openlight.trt")

    def __call__(self, origin_image):
        # preprocess image
        height, width, _ = origin_image.shape
        ratio_scale = height / FIX_SIZE
        add_width = (FIX_SIZE - int(FIX_SIZE * width / height)) // 2
        image = self.preprocess(origin_image)
        image = preprocess_tensor(image)

        # inference single image
        paf, heatmap = self.model(image)
        paf = paf.reshape((38, 40, 40))
        heatmap = heatmap.reshape((19, 40, 40))

        # parse paf and heatmap, and hand skeleton
        self.pose_parser.parser_pose(paf, heatmap)
        hand_images = self.pose_parser.get_hand_head_images(origin_image, ratio_scale, add_width)
        hand_skeleton = None
        if hand_images is not None:
            _, hand_skeleton = self.hand_model(hand_images)

        return self.pose_parser.postprocess_pose(hand_skeleton, ratio_scale, add_width)



