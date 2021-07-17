import numpy as np
import cv2
from .constant import *
try:
    from inference.pose_extractor import extract_poses
    print("using c++")
except:
    from inference.legacy_pose_extractor import extract_poses


def get_root_relative_poses(inference_results):
    paf_map, heatmap = inference_results

    upsample_ratio = 4
    found_poses = extract_poses(heatmap[0:-1], paf_map, upsample_ratio)[0]

    poses_2d = []
    num_kpt_panoptic = 19
    num_kpt = 18
    for pose_id in range(found_poses.shape[0]):
        if found_poses[pose_id, 3] == -1:  # skip pose if does not found neck
            continue
        pose_2d = np.ones(num_kpt_panoptic * 3 + 1, dtype=np.float32) * -1  # +1 for pose confidence
        for kpt_id in range(num_kpt):
            if found_poses[pose_id, kpt_id * 3] != -1:
                x_2d, y_2d = found_poses[pose_id, kpt_id * 3:kpt_id * 3 + 2]
                conf = found_poses[pose_id, kpt_id * 3 + 2]
                pose_2d[map_id_to_panoptic[kpt_id] * 3] = x_2d  # just repacking
                pose_2d[map_id_to_panoptic[kpt_id] * 3 + 1] = y_2d
                pose_2d[map_id_to_panoptic[kpt_id] * 3 + 2] = conf
        pose_2d[-1] = found_poses[pose_id, -1]
        poses_2d.append(pose_2d)

    return np.array(poses_2d)


previous_poses_2d = []


def parse_poses(inference_results):
    """
    parse inferece (paf, heatmap)
    :param inference_results:
    :param input_scale: scale of image and heatmap
    :return:list of pose = tuple of  (keypointsx3) + pose confidence)
    """
    poses_2d = get_root_relative_poses(inference_results)
    # return poses_2d
    poses_2d_scaled = []
    poses_prop = []
    for pose_2d in poses_2d:
        num_kpt = (pose_2d.shape[0] - 1) // 3
        pose_2d_scaled = np.ones((num_kpt, 3), dtype=np.float32) * -1  # +1 for pose confidence
        for kpt_id in range(num_kpt):
            if pose_2d[kpt_id * 3] != -1:
                pose_2d_scaled[kpt_id,0] = int(pose_2d[kpt_id * 3] / 0.5)
                pose_2d_scaled[kpt_id, 1] = int(pose_2d[kpt_id * 3 + 1] / 0.5)
                pose_2d_scaled[kpt_id , 2] = pose_2d[kpt_id * 3 + 2]
        poses_2d_scaled.append(pose_2d_scaled)
        poses_prop.append(pose_2d[-1])

    return poses_2d_scaled, poses_prop


class Pose:
    """
    store and parsing for drawing, optical flow
    """
    def __init__(self, image_scale,  base_on_prob = False):
        self.image_scale = image_scale
        self.poses_list = []
        self.poses_prob = []
        self.hand_window = []
        self.head_window = []

        # get best only
        self.base_on_prob = base_on_prob

    def parser_pose(self, paf, heatmap):
        # get pose list ([keypoints x 3], and prob)
        self.poses_list, self.poses_prob = parse_poses((paf, heatmap))
        self.hand_window = []
        self.head_window = []
        # get the best pose to index 0
        if len(self.poses_list) > 0:
            self.get_best_pose()

            # get the hand and head window 0
            self.get_hand_head_window()

    def postprocess_pose(self, hand_pose, ratio, pad):
        # normalize position of hand -> 0 -> 1
        left_hand = None
        right_hand = None
        if len(self.poses_list) > 0:
            best_body_pose = np.array(self.poses_list[0], np.float)[:,:2]



            # TODO: post process for join index here.
            if hand_pose is not None and hand_pose.multi_hand_landmarks:
                for i,hand in enumerate(hand_pose.multi_hand_landmarks):
                    if hand_pose.multi_handedness[i] == "Left":
                        left_hand = np.array([[pose.x, pose.y]for pose in hand.landmark], np.float)
                    elif hand_pose.multi_handedness[i] == "Right":
                        right_hand = np.array([[[pose.x, pose.y]for pose in hand.landmark]], np.float)
        else:
            best_body_pose = np.zeros((19,2))

        if left_hand is None:
            left_hand = np.zeros((21,2))
        if right_hand is None:
            right_hand = np.zeros((21,2))

        result = np.concatenate([best_body_pose, left_hand, right_hand], axis = 0)
        result[:,0] = result[:,0] - (pad+1)
        zero = result[:,0] < 0
        result[zero,0] = 0
        result = result * ratio

        return result

    def get_best_pose(self):
        # get only the biggest pose of image
        if self.base_on_prob:
            self.poses_list = [x for _, x in sorted(zip(self.poses_prob, self.poses_list))]
        else:
            max_size = 0
            max_index = 0
            for pose_id in range(len(self.poses_list)):
                max_x, min_x = np.max(self.poses_list[pose_id][:, 1]), np.min(self.poses_list[pose_id][:, 1])
                max_y, min_y = np.max(self.poses_list[pose_id][:, 0]), np.min(self.poses_list[pose_id][:, 0])
                temp_size = abs((max_x - min_x) * (max_y - min_y))
                if temp_size > max_size:
                    max_size = temp_size
                    max_index = pose_id
            self.poses_list[0], self.poses_list[max_index] = self.poses_list[max_index], self.poses_list[0]

    def get_hand_head_window(self):
        for pose_id in range(len(self.poses_list)):
            current_pose = self.poses_list[pose_id]
            max_x, min_x = np.max(current_pose[:, 1]), np.min(current_pose[:, 1])
            max_y, min_y = np.max(current_pose[:, 0]), np.min(current_pose[:, 0])
            size_hand = max((max_x - min_x), (max_y - min_y))
            size_hand = size_hand // 8

            was_found = current_pose[:, 2] > 0

            handtuple = []
            # get hand position
            if was_found[4] and was_found[5]:
                l_h = current_pose[5, 0:2] + (current_pose[5, 0:2] - current_pose[4, 0:2]) / 3
                handtuple.append(((
                                      int(l_h[0] - size_hand / 2),
                                      int(l_h[1] - size_hand / 2)
                                  ),
                                  (
                                      int(l_h[0] + size_hand / 2),
                                      int(l_h[1] + size_hand / 2)
                                  )
                                ))

            if was_found[10] and was_found[11]:
                r_h = current_pose[11, 0:2] + (current_pose[11, 0:2] - current_pose[10, 0:2]) / 3
                handtuple.append(((
                                      int(r_h[0] - size_hand / 2),
                                      int(r_h[1] - size_hand / 2)
                                  ),
                                  (
                                      int(r_h[0] + size_hand / 2),
                                      int(r_h[1] + size_hand / 2)
                                  )))
            self.hand_window.append(handtuple)

    def get_hand_head_images(self, origin_image, ratio, pad):
        hand_img = []
        w,h = 0,0
        if len(self.hand_window) >0:
            for window in self.hand_window[0]:
                temp_img = origin_image[int(window[0][1]*ratio):int(window[1][1]*ratio),
                                int(max(window[0][0]-pad,0)*ratio): int(max(window[1][0]-pad,0)*ratio)]

                if len(hand_img) ==0:
                    w,h, _ = temp_img.shape
                else:
                    w_t,h_t, _ = temp_img.shape
                    if w_t != w:
                        temp_img = temp_img[:w,:h]
                hand_img.append(temp_img)
        if len(hand_img) == 0:
            return None
        else:
            img_sum = cv2.hconcat(hand_img)
            return img_sum
