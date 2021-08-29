import torch
from .custom_augmentation import InferenceTransformation
from .pose import Pose
from .utils_mediapipe import HandPoseDetector
from .constant import *
from .onnx_inference import PoseDetectionONNX
from .onnx_with_tensorrt import PoseDetectionONNXTensorRT
import cv2
import numpy as np


def preprocess_tensor(image):
    x_data = image.astype('f')
    x_data /= 255
    x_data -= 0.5
    x_data = x_data.transpose(2, 0, 1)
    return x_data


class BodyDetectorONNXInference:
    """
    for single image only
    """
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
    """
    for single image only
    """
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


class BodyDetectorOpticalFlow:
    """
    support for batch of images
    """
    def __init__(self, inference_model, is_optical_flow=False):
        self.inference_model = inference_model
        self.is_optical_flow = is_optical_flow
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.pose_parser = Pose(image_scale=0.125)
        self.hand_model = HandPoseDetector(static_model=True)
        self.lk_params = dict(winSize=(100, 100),
                     maxLevel=10,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

    def __call__(self, videos, num_images):
        count = 0
        current_poses = None
        visible_poses = []
        for i in range(num_images):
            origin_image = videos[i]
            height, width, _ = origin_image.shape
            h_ratio = height / FIX_SIZE
            w_ratio = width / FIX_SIZE
            process_image = cv2.resize(origin_image, (FIX_SIZE, FIX_SIZE))

            if self.is_optical_flow and 0 < count < 4 and current_poses is not None:
                visible_mask = current_poses[:, 2] > 0
                visible_point = current_poses[visible_mask][:, 0:2]  # get only coord
                current_gray = cv2.cvtColor(process_image, cv2.COLOR_BGR2GRAY)
                visible_point = np.expand_dims(visible_point.astype(np.float32), axis=1)
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, current_gray, visible_point,
                                                       None, **self.lk_params)
                old_gray = current_gray.copy()
                visible_point[st == 1] = p1[st == 1]
                visible_point = np.squeeze(visible_point, axis=1)
                self.pose_parser.poses_list[0][np.where(visible_mask), :-1] = visible_point.copy()
            else:
                image = preprocess_tensor(process_image)

                # inference single image
                paf, heatmap = self.inference_model(image)
                paf = paf.reshape((38, 40, 40))
                heatmap = heatmap.reshape((19, 40, 40))

                # parse paf and heatmap, and hand skeleton
                self.pose_parser.parser_pose(paf, heatmap)


            hand_images, w_list, h_list = self.pose_parser.get_hand_head_images(process_image)
            hand_skeleton = None
            if hand_images is not None:
                _, hand_skeleton = self.hand_model(hand_images)
            parsed_pose, is_none = self.pose_parser.postprocess_pose(hand_skeleton, w_list, w_ratio, h_ratio, hand_images)
            if not is_none:
                current_poses = parsed_pose
                visible_poses.append(parsed_pose)
            else:
                current_poses = None

        return visible_poses


