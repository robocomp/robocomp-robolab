import onnxruntime
import numpy as np


class PoseDetectionONNX:
    def __init__(self, onnx_path):
        self.session_option = onnxruntime.SessionOptions()
        self.session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.ort_session = onnxruntime.InferenceSession(onnx_path, sess_options=self.session_option)
        self.ort_session.set_providers(['CPUExecutionProvider'])

        print("inference on %s"%onnxruntime.get_device())

    def __call__(self, input):
        # expand batch size
        input = np.expand_dims(input,axis = 0)
        ortvalue = onnxruntime.OrtValue.ortvalue_from_numpy(input, 'cpu', 0)

        # init input
        ort_inputs = {self.ort_session.get_inputs()[0].name: ortvalue}

        # inference
        output = self.ort_session.run(None, ort_inputs)

        return output