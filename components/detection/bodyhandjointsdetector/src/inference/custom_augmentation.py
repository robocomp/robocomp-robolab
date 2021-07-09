from albumentations import Compose, PadIfNeeded, LongestMaxSize


class InferenceTransformation:
    def __init__(self, width, height):
        self.aug = Compose([
            LongestMaxSize(max_size=width if width > height else height),
            PadIfNeeded(min_height=height, min_width=width, border_mode=cv2.BORDER_CONSTANT)
        ])

    def __call__(self, image):
        transformed = self.aug(image=image)
        return transformed["image"]