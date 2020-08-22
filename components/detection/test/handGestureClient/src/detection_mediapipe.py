import os
import sys
import csv
import cv2
import numpy as np
import tensorflow as tf

from non_maximum_suppression import non_max_suppression_fast

sys.path.append(os.path.join(os.getcwd(),"assets"))

PALM_MODEL_PATH = os.path.join(os.getcwd(),"assets","mediapipe_models",
                                "palm_detection_without_custom_op.tflite")
ANCHORS_PATH = os.path.join(os.getcwd(),"assets","mediapipe_models",
                                "anchors.csv")

JOINT_MODEL_PATH = os.path.join(os.getcwd(),"assets","mediapipe_models",
                                "hand_keys.tflite")
box_shift = 0.2
box_enlarge = 1.3
print("Mediapipe models loaded")

interp_palm = tf.lite.Interpreter(PALM_MODEL_PATH)
interp_palm.allocate_tensors()

interp_joint = tf.lite.Interpreter(JOINT_MODEL_PATH)
interp_joint.allocate_tensors()

# reading the SSD anchors
with open(ANCHORS_PATH, "r") as csv_f:
    anchors = np.r_[
        [x for x in csv.reader(csv_f, quoting=csv.QUOTE_NONNUMERIC)]
    ]
# reading tflite model paramteres
output_details = interp_palm.get_output_details()
input_details = interp_palm.get_input_details()

in_idx = input_details[0]['index']
out_reg_idx = output_details[0]['index']
out_clf_idx = output_details[1]['index']

in_idx_joint = interp_joint.get_input_details()[0]['index']
out_idx_joint = interp_joint.get_output_details()[0]['index']
# 90° rotation matrix used to create the alignment trianlge
R90 = np.r_[[[0,1],[-1,0]]]

# trianlge target coordinates used to move the detected hand
# into the right position
target_triangle = np.float32([
                [128, 128],
                [128,   0],
                [  0, 128]
            ])
target_box = np.float32([
                [  0,   0, 1],
                [256,   0, 1],
                [256, 256, 1],
                [  0, 256, 1],
            ])

def get_triangle(kp0, kp2, dist=1):
    """get a triangle used to calculate Affine transformation matrix"""

    dir_v = kp2 - kp0
    dir_v /= np.linalg.norm(dir_v)

    dir_v_r = dir_v @ R90.T
    return np.float32([kp2, kp2+dir_v*dist, kp2 + dir_v_r*dist])


def im_normalize(img):
     return np.ascontiguousarray(
         2 * ((img / 255) - 0.5
    ).astype('float32'))

def sigm(x):
    return 1 / (1 + np.exp(-x) )

def pad1(x):
    return np.pad(x, ((0,0),(0,1)), constant_values=1, mode='constant')

def predict_joints(img_norm):
    interp_joint.set_tensor(
        in_idx_joint, img_norm.reshape(1,256,256,3))
    interp_joint.invoke()

    joints = interp_joint.get_tensor(out_idx_joint)
    # print(joints.shape)
    # print(joints)
    return joints.reshape(-1,2)

def detect_hand(img_norm):

    assert -1 <= img_norm.min() and img_norm.max() <= 1,\
    "img_norm should be in range [-1, 1]"
    assert img_norm.shape == (256, 256, 3),\
    "img_norm shape must be (256, 256, 3)"

    # predict hand location and 7 initial landmarks
    interp_palm.set_tensor(in_idx, img_norm[None])
    interp_palm.invoke()

    """
    out_reg shape is [number of anchors, 18]
    Second dimension 0 - 4 are bounding box offset, width and height: dx, dy, w ,h
    Second dimension 4 - 18 are 7 hand keypoint x and y coordinates: x1,y1,x2,y2,...x7,y7
    """
    out_reg = interp_palm.get_tensor(out_reg_idx)[0]
    """
    out_clf shape is [number of anchors]
    it is the classification score if there is a hand for each anchor box
    """
    out_clf = interp_palm.get_tensor(out_clf_idx)[0,:,0]

    # finding the best prediction
    probabilities = sigm(out_clf)
    detecion_mask = probabilities > 0.5
    candidate_detect = out_reg[detecion_mask]
    candidate_anchors = anchors[detecion_mask]
    probabilities = probabilities[detecion_mask]

    if candidate_detect.shape[0] == 0:
        return None, None

    # Pick the best bounding box with non maximum suppression
    # the boxes must be moved by the corresponding anchor first
    moved_candidate_detect = candidate_detect.copy()
    moved_candidate_detect[:, :2] = candidate_detect[:, :2] + (candidate_anchors[:, :2] * 256)
    box_ids = non_max_suppression_fast(moved_candidate_detect[:, :4], probabilities)

    # Pick the first detected hand. Could be adapted for multi hand recognition
    box_ids = box_ids[0]

    # bounding box offsets, width and height
    dx,dy,w,h = candidate_detect[box_ids, :4]
    center_wo_offst = candidate_anchors[box_ids,:2] * 256

    # 7 initial keypoints
    keypoints = center_wo_offst + candidate_detect[box_ids,4:].reshape(-1,2)
    side = max(w,h) * box_enlarge

    # now we need to move and rotate the detected hand for it to occupy a
    # 256x256 square
    # line from wrist keypoint to middle finger keypoint
    # should point straight up
    source = get_triangle(keypoints[0], keypoints[2], side)
    source -= (keypoints[0] - keypoints[2]) * box_shift

    return source, keypoints

def preprocess_img(img):
    # fit the image into a 256x256 square
    shape = np.r_[img.shape]
    pad = (shape.max() - shape[:2]).astype('uint32') // 2
    img_pad = np.pad(
        img,
        ((pad[0],pad[0]), (pad[1],pad[1]), (0,0)),
        mode='constant')
    img_small = cv2.resize(img_pad, (256, 256))
    img_small = np.ascontiguousarray(img_small)

    img_norm = im_normalize(img_small)
    return img_pad, img_norm, pad


def hand_detector(img):
    img_pad, img_norm, pad = preprocess_img(img)

    source, keypoints = detect_hand(img_norm)
    if source is None:
        return None, None, None

    # calculating transformation from img_pad coords
    # to img_landmark coords (cropped hand image)
    scale = max(img.shape) / 256
    Mtr = cv2.getAffineTransform(
        source * scale,
        target_triangle
    )

    img_landmark = cv2.warpAffine(
        im_normalize(img_pad), Mtr, (256,256)
    )

    joints = predict_joints(img_landmark)
    # adding the [0,0,1] row to make the matrix square
    Mtr = pad1(Mtr.T).T
    Mtr[2,:2] = 0

    Minv = np.linalg.inv(Mtr)

    # projecting keypoints back into original image coordinate space
    box_orig = (target_box @ Minv.T)[:,:2]
    box_orig -= pad[::-1]
    kp_orig = (pad1(joints) @ Minv.T)[:,:2]
    kp_orig -= pad[::-1]

    return box_orig, kp_orig, joints
