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
    found_poses[:,0:-1:3] /= upsample_ratio
    found_poses[:,1:-1:3] /= upsample_ratio

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
                pose_2d_scaled[kpt_id,0] = int(pose_2d[kpt_id * 3] / 0.125)
                pose_2d_scaled[kpt_id, 1] = int(pose_2d[kpt_id * 3 + 1] / 0.125)
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
        self.hand_window_origin = []
        self.hand_type = []
        self.head_window = []


        # get best only
        self.base_on_prob = base_on_prob

    def parser_pose(self, paf, heatmap):
        # get pose list ([keypoints x 3], and prob)
        self.poses_list, self.poses_prob = parse_poses((paf, heatmap))
        self.hand_window = []
        self.head_window = []
        self.hand_type = []
        # get the best pose to index 0
        if len(self.poses_list) > 0:
            self.get_best_pose()

            # get the hand and head window 0
            self.get_hand_head_window(1)

    def convert_hand_position(self, w_list, skeleton):
        min_w = np.min(skeleton[:,0])
        min_w_rect_img = min(w_list)
        index = int(min_w / min_w_rect_img)

        if index >= len(self.hand_window_origin):
            index = len(self.hand_window_origin) - 1

        skeleton[:,0] += self.hand_window_origin[index][1] - (index * min_w_rect_img)
        skeleton[:,1] += self.hand_window_origin[index][0]
        return skeleton, self.hand_type[index]


    def postprocess_pose(self, hand_pose, w_list, w_ratio, h_ratio, hand_imgs):
        # normalize position of hand -> 0 -> 1
        is_none_body = False
        left_hand = None
        right_hand = None
        if len(self.poses_list) > 0:
            best_body_pose = np.array(self.poses_list[0], np.float)[:,:2]



            # TODO: post process for join index here.
            if hand_pose is not None and hand_pose.multi_hand_landmarks:
                h , w, _ = hand_imgs.shape
                for i,hand in enumerate(hand_pose.multi_hand_landmarks):
                    position = np.array([[pose.x*w, pose.y*h]for pose in hand.landmark], np.float)
                    position, type = self.convert_hand_position(w_list, position)

                    if type == 'l':
                        left_hand = position
                    elif type == 'r':
                        right_hand = position

        else:
            best_body_pose = np.zeros((19,2))
            is_none_body = True

        if left_hand is None:
            left_hand = np.zeros((21,2))
        if right_hand is None:
            right_hand = np.zeros((21,2))

        result = np.concatenate([best_body_pose, left_hand, right_hand], axis = 0)
        zero = result[:,0] < 0
        result[zero,0] = 0
        result[:,0] = result[:,0] * w_ratio
        result[:,1] = result[:,1] * h_ratio

        return result, is_none_body

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

    def get_hand_head_window(self, num_pose):

        for pose_id in range(len(self.poses_list[:num_pose])):
            current_pose = self.poses_list[pose_id]

            was_found = current_pose[:, 2] > 0

            handtuple = []
            # get hand position
            if was_found[4] and was_found[5]:
                l_h = current_pose[5, 0:2] + (current_pose[5, 0:2] - current_pose[4, 0:2]) / 2
                handtuple.append(((
                                      int(l_h[0] - 60),
                                      int(l_h[1] - 60)
                                  ),
                                  (
                                      int(l_h[0] + 60),
                                      int(l_h[1] + 60)
                                  )
                                ))
                self.hand_type.append("l")

            if was_found[10] and was_found[11]:
                r_h = current_pose[11, 0:2] + (current_pose[11, 0:2] - current_pose[10, 0:2]) / 2
                handtuple.append(((
                                      int(r_h[0] - 60),
                                      int(r_h[1] - 60)
                                  ),
                                  (
                                      int(r_h[0] + 60),
                                      int(r_h[1] + 60)
                                  )))
                self.hand_type.append("r")
            if len(handtuple) > 0:
                self.hand_window.append(handtuple)

    def get_hand_head_images(self, origin_image):
        hand_img = []
        self.hand_window_origin = []
        w_list,h_list = [],[]
        if len(self.hand_window) >0:
            # just get from 1 body pose
            for window in self.hand_window[0]:
                temp_img = origin_image[int(window[0][1]):int(window[1][1]),
                                int(window[0][0]): int(window[1][0])]
                self.hand_window_origin.append((int(window[0][1]),int(window[0][0])))
                w,h, _ = temp_img.shape
                w_list.append(w)
                h_list.append(h)
                hand_img.append(temp_img)
        if len(hand_img) == 0:
            return None, None, None
        else:
            min_w = min(w_list)
            min_h = min(h_list)
            for i in range(len(hand_img)):
                hand_img[i] = hand_img[i][:min_w,:min_h]
            img_sum = cv2.hconcat(hand_img)

            return img_sum, w_list, h_list
