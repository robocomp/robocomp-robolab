import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time

import sys, os
sys.path.insert(1, os.path.join(sys.path[0], ".."))
import common

TRT_LOGGER = trt.Logger()


def get_engine(onnx_file_path, engine_file_path=""):
    '''
    Check engine, create if not
    :param onnx_file_path:
    :param engine_file_path:
    :return:
    '''
    """Attempts to load a serialized engine if available, otherwise builds a new TensorRT engine and saves it."""
    def build_engine():
        """Takes an ONNX file and creates a TensorRT engine to run inference with"""
        with trt.Builder(TRT_LOGGER) as builder, builder.create_network(common.EXPLICIT_BATCH) as network, builder.create_builder_config() as config, trt.OnnxParser(network, TRT_LOGGER) as parser:
            config.max_workspace_size = 1 << 20 # 256MiB
            builder.max_batch_size = 1
            # Parse model file
            if not os.path.exists(onnx_file_path):
                print('ONNX file {} not found, please run yolov3_to_onnx.py first to generate it.'.format(onnx_file_path))
                exit(0)
            print('Loading ONNX file from path {}...'.format(onnx_file_path))
            with open(onnx_file_path, 'rb') as model:
                print('Beginning ONNX file parsing')
                if not parser.parse(model.read()):
                    print ('ERROR: Failed to parse the ONNX file.')
                    for error in range(parser.num_errors):
                        print (parser.get_error(error))
                    return None
            # The actual yolov3.onnx is generated with batch size 64. Reshape input to batch size 1
            network.get_input(0).shape = [1, 55, 100]
            print('Completed parsing of ONNX file')
            print('Building an engine from file {}; this may take a while...'.format(onnx_file_path))
            engine = builder.build_engine(network, config)
            print("Completed creating Engine")
            with open(engine_file_path, "wb") as f:
                f.write(engine.serialize())
            return engine

    if os.path.exists(engine_file_path):
        # If a serialized engine exists, use it instead of building an engine.
        print("Reading engine from file {}".format(engine_file_path))
        with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            return runtime.deserialize_cuda_engine(f.read())
    else:
        return build_engine()


class PoseDetectionONNXTensorRT:
    def __init__(self, onnx_path, tensor_path):
        '''
        read onnx and create tensorrt engine.
        :param onnx_path:
        :param tensor_path:
        '''
        self.tensor_path = tensor_path
        self.onnx_path = onnx_path

        # create engine and context
        self.engine = get_engine(self.onnx_path, self.tensor_path)
        self.context = self.engine.create_execution_context()

        # prepare location for
        self.inputs, self.outputs, self.bindings, self.stream = \
            common.allocate_buffers(self.engine)

    def __call__(self, input):
        # init image to input location.
        np.copyto(self.inputs[0].host, input.ravel())

        # When infering on single image, we measure inference
        # time to output it to the user
        # inference_start_time = time.time()

        # Fetch output from the model
        output = common.do_inference_v2(self.context,
                                                  bindings=self.bindings,
                                                  inputs=self.inputs,
                                                  outputs=self.outputs,
                                                  stream=self.stream)

        # Output inference time
        # print("TensorRT inference time: {} ms".format(
        #     int(round((time.time() - inference_start_time) * 1000))))

        # And return results
        return output

    def close_tensorrt(self):
        pass