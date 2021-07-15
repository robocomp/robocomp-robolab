import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
mp_face_mesh = mp.solutions.face_mesh


class HandPoseDetector:
    def __init__(self, num_hands = 2, static_model = False):
        self.hands = mp_hands.Hands(static_image_mode = static_model,
                                    max_num_hands=num_hands,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)

    def __call__(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.hands.process(image)

        # draw on input image
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # return results.multi_hand_landmarks[0],results.multi_handedness[0]
        return image, results

    def __delete__(self):
        self.hands.close()
        print("release mem for hand processor")


class FaceMeshDetector:
    def __init__(self, num_hands = 1, static_model = False):
        self.faces = mp_face_mesh.FaceMesh(static_image_mode = static_model,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)
        self.drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

    def __call__(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.faces.process(image)

        # draw on input image
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_face_landmarks:
            face_landmarks = results.multi_face_landmarks[0]
            mp_drawing.draw_landmarks(
                image=image,
                landmark_list=face_landmarks,
                connections=mp_face_mesh.FACE_CONNECTIONS,
                landmark_drawing_spec=self.drawing_spec,
                connection_drawing_spec=self.drawing_spec)

        # return results.multi_face_landmarks
        return image, results.multi_face_landmarks

    def __delete__(self):
        self.faces.close()
        print("release mem for face processor")