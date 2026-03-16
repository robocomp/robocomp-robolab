#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
import cv2
import numpy as np
import subprocess
import platform
import os
from genericworker import *
import interfaces as ifaces

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.configData = configData

        # Streaming variables
        self.streaming_enabled = False
        self.stream_pipeline = None
        self.frameL = None
        self.frame_count = 0

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')
        self.stop_streaming()
        if hasattr(self, 'capL') and self.capL.isOpened():
            self.capL.release()
        cv2.destroyAllWindows()

    def detect_platform(self):
        """Detects if we are on Jetson or PC with NVIDIA GPU"""
        # Check if it's Jetson
        if os.path.exists('/etc/nv_tegra_release'):
            return 'jetson'

        # Check if NVIDIA GPU is available
        try:
            result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)
            if result.returncode == 0:
                return 'nvidia'
        except FileNotFoundError:
            pass

        return 'cpu'

    def build_streaming_pipeline(self, width, height, fps, rtmp_url):
        """Builds GStreamer pipeline according to platform"""
        platform_type = self.detect_platform()
        print(f"Platform detected: {platform_type}")

        if platform_type == 'jetson':
            # Pipeline for Jetson Orin/Xavier with nvv4l2h264enc
            pipeline = (
                f"fdsrc fd=0 ! "
                f"rawvideoparse format=bgr width={width} height={height} framerate={fps}/1 ! "
                f"queue max-size-buffers=1 leaky=downstream ! "
                f"videoconvert ! video/x-raw,format=I420 ! "
                f"nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
                f"nvv4l2h264enc preset-level=1 control-rate=1 bitrate=5000000 "
                f"iframeinterval=30 insert-sps-pps=true maxperf-enable=true ! "
                f"h264parse ! "
                f"flvmux streamable=true ! "
                f"rtmpsink location={rtmp_url} sync=false"
            )
        elif platform_type == 'nvidia':
            # Pipeline for PC with NVIDIA GPU using nvenc
            pipeline = (
                f"fdsrc fd=0 ! "
                f"rawvideoparse format=bgr width={width} height={height} framerate={fps}/1 ! "
                f"queue max-size-buffers=1 leaky=downstream ! "
                f"videoconvert ! video/x-raw,format=I420 ! "
                f"nvh264enc preset=low-latency-hq rc-mode=cbr bitrate=5000 "
                f"gop-size=30 ! "
                f"h264parse ! "
                f"flvmux streamable=true ! "
                f"rtmpsink location={rtmp_url} sync=false"
            )
        else:
            # CPU fallback pipeline with x264enc
            pipeline = (
                f"fdsrc fd=0 ! "
                f"rawvideoparse format=bgr width={width} height={height} framerate={fps}/1 ! "
                f"queue max-size-buffers=1 leaky=downstream ! "
                f"videoconvert n-threads=4 ! video/x-raw,format=I420 ! "
                f"x264enc tune=zerolatency speed-preset=ultrafast bitrate=5000 "
                f"key-int-max=30 ! "
                f"h264parse ! "
                f"flvmux streamable=true ! "
                f"rtmpsink location={rtmp_url} sync=false"
            )

        return pipeline

    def init_streaming(self, width, height, fps):
        """Initializes streaming pipeline to MediaMTX using subprocess"""
        stream_host = self.configData.get("Stream", {}).get("Host", "localhost")
        stream_port = int(self.configData.get("Stream", {}).get("Port", 1935))
        stream_path = self.configData.get("Stream", {}).get("Path", "camera")

        rtmp_url = f"rtmp://{stream_host}:{stream_port}/{stream_path}"
        print(f"Initializing streaming to: {rtmp_url}")

        pipeline_str = self.build_streaming_pipeline(width, height, fps, rtmp_url)
        print(f"GStreamer pipeline: {pipeline_str}")

        try:
            # Start GStreamer process directly
            gst_command = f"gst-launch-1.0 -q {pipeline_str}"

            self.stream_pipeline = subprocess.Popen(
                gst_command,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Save dimensions and fps for frame sending
            self.stream_width = width
            self.stream_height = height
            self.stream_fps = fps

            print(f"✓ Streaming pipeline initialized successfully (PID: {self.stream_pipeline.pid})")
            self.streaming_enabled = True
            return True

        except Exception as e:
            print(f"✗ Error initializing streaming: {e}")
            return False

    def stop_streaming(self):
        """Stops streaming pipeline"""
        if self.stream_pipeline is not None:
            try:
                self.stream_pipeline.terminate()
                self.stream_pipeline.wait(timeout=2)
            except:
                self.stream_pipeline.kill()
            self.stream_pipeline = None
            self.streaming_enabled = False
            print("Streaming stopped")

    def setParams(self, params):
        # Get camera device from configuration
        camera_device = self.configData.get("Camera", {}).get("Device", "/dev/video2")
        width = int(self.configData.get("Camera", {}).get("Width", 640))
        height = int(self.configData.get("Camera", {}).get("Height", 480))
        fps = int(self.configData.get("Camera", {}).get("FPS", 30))

        # Display configuration
        self.display_enabled = int(self.configData.get("Display", {}).get("Enabled", 1)) == 1

        # Streaming configuration
        streaming_config = int(self.configData.get("Stream", {}).get("Enabled", 0))

        print(f"Opening camera: {camera_device}")

        # Use V4L2 backend for better performance on Linux
        self.capL = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)

        if not self.capL.isOpened():
            print(f"Error: Could not open camera {camera_device}")
            return False

        # Real-time optimizations
        # Use MJPEG for faster decoding (less CPU load)
        self.capL.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Set resolution
        self.capL.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capL.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Maximize FPS
        self.capL.set(cv2.CAP_PROP_FPS, fps)

        # Minimize internal buffer (reduces latency)
        self.capL.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Get actual values
        actual_width = int(self.capL.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capL.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.capL.get(cv2.CAP_PROP_FPS))

        print(f"Resolution: {actual_width}x{actual_height}")
        print(f"FPS configured: {actual_fps}")

        # Initialize streaming if enabled
        if streaming_config == 1:
            self.init_streaming(actual_width, actual_height, actual_fps if actual_fps > 0 else fps)

        return True


    @QtCore.Slot()
    def compute(self):
        # Use grab() + retrieve() which is more efficient than read() for real-time
        if self.capL.grab():
            retL, self.frameL = self.capL.retrieve()
            if retL:
                self.frame_count += 1

                # Send frame to streaming if enabled
                if self.streaming_enabled and self.stream_pipeline is not None:
                    try:
                        # Check if process is still active
                        if self.stream_pipeline.poll() is not None:
                            print("✗ Streaming pipeline stopped, restarting...")
                            self.stop_streaming()
                            self.init_streaming(self.stream_width, self.stream_height, self.stream_fps)
                        else:
                            # Write raw frame to stdin of GStreamer process
                            self.stream_pipeline.stdin.write(self.frameL.tobytes())
                            self.stream_pipeline.stdin.flush()
                    except Exception as e:
                        if self.frame_count % 100 == 0:  # Only print every 100 frames
                            print(f"Error sending frame: {e}")

                # Show image with OpenCV if enabled
                if self.display_enabled:
                    cv2.imshow('CameraSimple', self.frameL)
                    cv2.waitKey(1)
        else:
            print("No frame could be grabbed")
        return True

    def startup_check(self):
        print(f"Testing RoboCompCameraSimple.TImage from ifaces.RoboCompCameraSimple")
        test = ifaces.RoboCompCameraSimple.TImage()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getImage
    #
    def CameraSimple_getImage(self):
        #
        #implementCODE
        #
        im = TImage()
        im.image = self.frameL
        im.height, im.width, im.depth = self.frameL.shape
        return im

    # ===================================================================
    # ===================================================================


