import torch
from .custom_augmentation import InferenceTransformation
from .pose import Pose
from .utils_mediapipe import HandPoseDetector
from .constant import *
from .mini_model import *


def preprocess_tensor(image):
    x_data = image.astype('f')
    x_data /= 255
    x_data -= 0.5
    x_data = x_data.transpose(2, 0, 1)
    return x_data


class BodyDetectorInference:
    def __init__(self, pretrained_weight):
        self.preprocess = InferenceTransformation(FIX_SIZE, FIX_SIZE)
        self.pose_parser = Pose(image_scale=0.125)
        self.hand_model = HandPoseDetector(static_model=True)
        self.model = OpenPoseLightning()
        self.model.load_state_dict(torch.load(pretrained_weight))
        self.model.cuda()

    def __call__(self, origin_image):
        # preprocess image
        height, width, _ = origin_image.shape
        ratio_scale = height / FIX_SIZE
        add_width = (FIX_SIZE - int(FIX_SIZE * width / height)) // 2
        image = self.preprocess(origin_image)
        image = torch.Tensor(preprocess_tensor(image)).unsqueeze(0).cuda()

        # inference single image
        paf, heatmap = self.model(image)
        paf = paf.detach().cpu().numpy()[0]
        heatmap = heatmap.detach().cpu().numpy()[0]

        # parse paf and heatmap, and hand skeleton
        self.pose_parser.parser_pose(paf, heatmap)
        hand_images = self.pose_parser.get_hand_head_images(origin_image, ratio_scale, add_width)
        hand_skeleton = None
        if hand_images is not None:
            _, hand_skeleton = self.hand_model(hand_images)

        return self.pose_parser.poses_list, hand_skeleton



