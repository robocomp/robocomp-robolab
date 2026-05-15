#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by YOUR NAME HERE
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

import os
import sys
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

from PySide6 import QtCore
from PySide6.QtCore import QTimer, QByteArray
from PySide6.QtGui import QVector3D, QColor, QImage, QPixmap
from PySide6.QtWidgets import QApplication, QHBoxLayout, QWidget, QLabel
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DRender import Qt3DRender
from PySide6.Qt3DExtras import Qt3DExtras
from genericworker import *
import interfaces as ifaces
try:
    from .webots_style_camera_controller import WebotsStyleCameraController
except ImportError:
    from webots_style_camera_controller import WebotsStyleCameraController
import cv2
import numpy as np
import queue
import threading
import time
import torch
from ultralytics import YOLO
import os

# Workaround for OpenCV Qt font warning
cv2_qt_font_dir = os.path.join(os.path.dirname(cv2.__file__), 'qt', 'fonts')
os.makedirs(cv2_qt_font_dir, exist_ok=True)

sys.path.append('/opt/robocomp/lib')
class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.Display = configData["Display"] if "Display" in configData else "True"
        self.display_enabled = str(self.Display).lower() in ["true", "1", "yes", "on"]
        self.use_proxy_thread = str(configData.get("ProxyThread", "False")).lower() in ["true", "1", "yes", "on"]
        self.proxy_poll_period = max(float(self.Period) / 1000.0, 0.005)
        self.proxy_min_sleep = 0.001
        self.proxy_max_sleep = 0.050
        self.proxy_wait_ms = max(1, int(self.proxy_poll_period * 1000.0))

        component_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

        seg_engine_path = os.path.join(component_root, 'yolo26m-seg.engine')
        seg_pt_path = os.path.join(component_root, 'yolo26m-seg.pt')
        if os.path.exists(seg_engine_path):
            seg_model_path = seg_engine_path
        elif os.path.exists(seg_pt_path):
            seg_model_path = seg_pt_path
        else:
            seg_model_path = 'yolo26m-seg.pt'
            print(f"No local segmentation model found in {component_root}. Ultraytics will download {seg_model_path} if available.")

        pose_engine_path = os.path.join(component_root, 'yolo26m-pose.engine')
        pose_pt_path = os.path.join(component_root, 'yolo26m-pose.pt')
        if os.path.exists(pose_engine_path):
            pose_model_path = pose_engine_path
        elif os.path.exists(pose_pt_path):
            pose_model_path = pose_pt_path
        else:
            pose_model_path = 'yolo26m-pose.pt'
            print(f"No local pose model found in {component_root}. Ultraytics will download {pose_model_path} if available.")

        print(f"Loading YOLO model from {seg_model_path}...")
        self.yolo_model = YOLO(seg_model_path)

        print(f"Loading YOLO pose model from {pose_model_path}...")
        self.yolo_pose_model = YOLO(pose_model_path)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {self.device}")
        print("YOLO loaded.")

        # Memory reuse optimization: pre-allocate reusable arrays
        self._reusable_seg_mask = None
        self._last_img_size = None
        
        # Cache empty point cloud to avoid repeated creation
        self._cached_empty_cloud = ifaces.RoboCompImageSegmentation.PointCloud()
        self._cached_empty_cloud.X = []
        self._cached_empty_cloud.Y = []
        self._cached_empty_cloud.Z = []
        self._cached_empty_cloud.numberPoints = 0
        self._cached_empty_cloud.timestamp = 0

        # FPS tracking variables
        self.fps_frames = 0
        self.fps_last_time = time.time()
        
        # State variables for serving to clients
        self.last_image_request_time = 0.0
        self.last_objects_request_time = 0.0
        self.image_request_hold_seconds = 1.0
        self.objects_request_hold_seconds = 1.0

        # Latest published snapshot for ICE consumers
        initial_timestamp = int(time.time() * 1000)
        self._state_lock = threading.Lock()
        self._shutting_down = False
        self._latest_snapshot = (
            [],
            ifaces.RoboCompImageSegmentation.TImage(),
            ifaces.RoboCompImageSegmentation.TDepth(),
            initial_timestamp,
        )

        # Background proxy reader queue (decouples network latency from compute loop)
        self._proxy_queue = queue.Queue(maxsize=1)
        self._proxy_last_rgbd = None
        self._proxy_thread_stop = threading.Event()
        self._proxy_thread = None

        # Display toggle from config
        if self.display_enabled:
            self.setup_qt3d()
        else:
            self.hide()
        
        if startup_check:
            self.startup_check()
        else:
            if self.use_proxy_thread:
                self._start_proxy_thread()
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def initialize(self):
        component_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

        seg_model_path = self._choose_model_path(component_root, 'yolo26m-seg')
        pose_model_path = self._choose_model_path(component_root, 'yolo26m-pose')

        print(f"Loading YOLO model from {seg_model_path}...")
        self.yolo_model = YOLO(seg_model_path)

        print(f"Loading YOLO pose model from {pose_model_path}...")
        self.yolo_pose_model = YOLO(pose_model_path)

        # GPU optimization: detect CUDA availability (device specified during inference for TensorRT)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {self.device}")
        print("YOLO loaded.")

    def __del__(self):
        """Destructor"""
        self._shutdown()

    def closeEvent(self, event):
        self._shutdown()
        super(SpecificWorker, self).closeEvent(event)

    def _shutdown(self):
        # Check if _state_lock was initialized (handles case where __init__ failed early)
        if not hasattr(self, '_state_lock'):
            return
        
        with self._state_lock:
            if self._shutting_down:
                return
            self._shutting_down = True

        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()
        if hasattr(self, '_proxy_thread'):
            self._stop_proxy_thread()

    @QtCore.Slot()
    def compute(self):

        print("------------------------------------------------")
        time_start = time.time()

        # Return early if initialization failed or we're shutting down
        if not hasattr(self, '_state_lock'):
            return
        
        with self._state_lock:
            if self._shutting_down:
                return
        self._update_fps()

        try:
            if self.use_proxy_thread:
                rgbd = self._get_latest_rgbd()
                if rgbd is None:
                    # Avoid racing the proxy thread with a parallel direct proxy call.
                    return
                img_struct = rgbd.image
                depth_struct = rgbd.depth
            else:
                depth = self.camerargbdsimple_proxy.getDepthAsync("camera")
                image = self.camerargbdsimple_proxy.getImageAsync("camera")
                rgbd = ifaces.RoboCompCameraRGBDSimple.TRGBD()
                depth_struct = depth.result()
                img_struct = image.result()

            print(f"Proxy fetch time: {(time.time() - time_start) * 1000:.2f} ms")

            img = np.frombuffer(img_struct.image, dtype=np.uint8)

            img = img.reshape(img_struct.height, img_struct.width, img_struct.depth)

            # GPU optimization: specify device for inference (works for both PyTorch and TensorRT models)
            results = self.yolo_model(img, device=self.device, verbose=False)

            print(f"YOLO segmentation inference time: {(time.time() - time_start) * 1000:.2f} ms")

            results_pose = self.yolo_pose_model(img, device=self.device, verbose=False)

            print(f"YOLO pose inference time: {(time.time() - time_start) * 1000:.2f} ms")

            annotated_img = img.copy()

            depth_np, fx, fy, cx, cy = self._get_depth_intrinsics(depth_struct)

            annotated_img, seg_mask, segmented_objects = self._process_segmentation(
                results,
                annotated_img,
                depth_np,
                fx,
                fy,
                cx,
                cy,
                img_struct.height,
                img_struct.width,
            )

            print(f"Segmentation processing time: {(time.time() - time_start) * 1000:.2f} ms")

            # Process pose estimation results
            pose_objects = self._process_pose_segmentation(
                results_pose,
                annotated_img,
                depth_np,
                fx,
                fy,
                cx,
                cy,
                img_struct.height,
                img_struct.width,
            )

            self._modify_person_objects(segmented_objects, pose_objects, depth_np, fx, fy, cx, cy, annotated_img)

            print(f"Pose processing time: {(time.time() - time_start) * 1000:.2f} ms")

            # self._update_qt3d_from_mask(seg_mask, depth_np, fx, fy, cx, cy, img)

            image_out = self._build_output_image(img_struct, img.tobytes())
            depth_out = self._build_output_depth(depth_struct)

            with self._state_lock:
                if self._shutting_down:
                    return
                self._latest_snapshot = (segmented_objects, image_out, depth_out, int(time.time() * 1000))

            # Display the annotated image with both segmentation and pose overlays
            self._update_2d_display(annotated_img)

            print(f"Total compute loop time: {(time.time() - time_start) * 1000:.2f} ms")
            
        except Exception as e:
            print(f"Error reading image: {e}")

    #######################################################################################
    def _start_proxy_thread(self):
        if self._proxy_thread is not None:
            return

        self._proxy_thread_stop.clear()
        self._proxy_thread = threading.Thread(target=self._proxy_loop, daemon=True)
        self._proxy_thread.start()

    def _stop_proxy_thread(self):
        self._proxy_thread_stop.set()
        if self._proxy_thread is not None and self._proxy_thread.is_alive():
            self._proxy_thread.join(timeout=0.5)
        self._proxy_thread = None

    def _update_proxy_wait_from_period(self, rgbd):
        period_ms = getattr(rgbd.image, "period", None)
        if period_ms is None:
            return

        period_ms = int(period_ms)
        if self.proxy_wait_ms > (period_ms + 2):
            self.proxy_wait_ms -= 1
        elif self.proxy_wait_ms < (period_ms - 2):
            self.proxy_wait_ms += 1

        min_ms = max(1, int(self.proxy_min_sleep * 1000.0))
        max_ms = max(min_ms, int(self.proxy_max_sleep * 1000.0))
        self.proxy_wait_ms = max(min_ms, min(self.proxy_wait_ms, max_ms))

    def _proxy_loop(self):
        while not self._proxy_thread_stop.is_set():
            try:
                depth = self.camerargbdsimple_proxy.getDepthAsync("camera")
                image = self.camerargbdsimple_proxy.getImageAsync("camera")
                rgbd = ifaces.RoboCompCameraRGBDSimple.TRGBD()
                rgbd.depth = depth.result()
                rgbd.image = image.result()

                self._proxy_queue.put(rgbd, timeout=0.1)
                self._update_proxy_wait_from_period(rgbd)
                
                self._proxy_thread_stop.wait(self.proxy_wait_ms / 1000.0)
            except queue.Full:
                self._proxy_thread_stop.wait(self.proxy_wait_ms / 1000.0)
            except Exception as e:
                print(f"Proxy thread error: {e}")
                self._proxy_thread_stop.wait(0.01)

    def _get_latest_rgbd(self):
        try:
            latest = self._proxy_queue.get(timeout=0.1)
            self._proxy_last_rgbd = latest
        except queue.Empty:
            latest = self._proxy_last_rgbd

        return latest

    ###############################################################################

    def _update_fps(self):
        self.fps_frames += 1
        current_time = time.time()
        if (current_time - self.fps_last_time) >= 3.0:
            fps = self.fps_frames / (current_time - self.fps_last_time)
            print(f"Current FPS: {fps:.2f}")
            self.fps_frames = 0
            self.fps_last_time = current_time

    def _build_output_image(self, img_struct, image_bytes):
        img_out = ifaces.RoboCompImageSegmentation.TImage()
        img_out.compressed = img_struct.compressed
        img_out.cameraID = img_struct.cameraID
        img_out.width = img_struct.width
        img_out.height = img_struct.height
        img_out.depth = img_struct.depth
        img_out.focalx = img_struct.focalx
        img_out.focaly = img_struct.focaly
        img_out.alivetime = img_struct.alivetime
        img_out.period = img_struct.period
        img_out.image = image_bytes
        return img_out

    def _build_output_depth(self, depth_struct):
        depth_out = ifaces.RoboCompImageSegmentation.TDepth()
        depth_out.compressed = depth_struct.compressed
        depth_out.cameraID = depth_struct.cameraID
        depth_out.width = depth_struct.width
        depth_out.height = depth_struct.height
        depth_out.focalx = depth_struct.focalx
        depth_out.focaly = depth_struct.focaly
        depth_out.alivetime = depth_struct.alivetime
        depth_out.period = depth_struct.period
        depth_out.depthFactor = depth_struct.depthFactor
        depth_out.depth = depth_struct.depth
        return depth_out

    def _get_depth_intrinsics(self, depth_struct):
        depth_np = np.frombuffer(depth_struct.depth, dtype=np.float32).reshape(
            (depth_struct.height, depth_struct.width)
        )
        fx = depth_struct.focalx if depth_struct.focalx > 0 else 585.756
        fy = depth_struct.focaly if depth_struct.focaly > 0 else 585.756
        cx = depth_struct.width / 2.0
        cy = depth_struct.height / 2.0
        return depth_np, fx, fy, cx, cy

    def _project_depth_points(self, u, v, depth_np, fx, fy, cx, cy):
        if len(u) == 0:
            return None, None, None, None, None
        
        h, w = depth_np.shape
        u = np.clip(u, 0, w - 1).astype(int)
        v = np.clip(v, 0, h - 1).astype(int)

        z = depth_np[v, u]
        valid = (z > 0) & np.isfinite(z)
        if not np.any(valid):
            return None, None, None, None, None

        z_val = z[valid]
        u_val = u[valid]
        v_val = v[valid]
        x_cam = (u_val - cx) * z_val / fx
        y_cam = (cy - v_val) * z_val / fy
        z_cam = z_val

        x_robocomp = x_cam
        y_robocomp = z_cam
        z_robocomp = y_cam
        return x_robocomp, y_robocomp, z_robocomp, u_val, v_val
    
    def _build_segmented_object(self, contour_i32, label_name, conf, depth_np, fx, fy, cx, cy):
        seg_obj = ifaces.RoboCompImageSegmentation.SegmentedObject()
        seg_obj.label = label_name
        seg_obj.score = float(conf)

        # Build imagePolygon more efficiently - avoid reshape if already correct shape
        if contour_i32.ndim == 2 and contour_i32.shape[1] == 2:
            contour_points = contour_i32
        else:
            contour_points = contour_i32.reshape(-1, 2)
        
        polygon = ifaces.RoboCompImageSegmentation.Polygon()
        polygon.U = contour_points[:, 0].tolist()
        polygon.V = contour_points[:, 1].tolist()
        polygon.numberPoints = len(contour_points)
        polygon.timestamp = 0
        seg_obj.imagePolygon = polygon

        # Early bounds check with cached empty cloud
        x0, y0, width, height = cv2.boundingRect(contour_i32)
        if width <= 0 or height <= 0:
            seg_obj.points3D = self._cached_empty_cloud
            return seg_obj

        # Optimize mask creation - use existing contour_points, avoid reshape
        local_mask = np.zeros((height, width), dtype=np.uint8)
        shifted_contour = contour_points - np.array([x0, y0], dtype=np.int32)
        cv2.fillPoly(local_mask, [shifted_contour], 1)

        local_v, local_u = np.where(local_mask > 0)
        if len(local_u) == 0:
            seg_obj.points3D = self._cached_empty_cloud
            return seg_obj
            
        obj_u = local_u + x0
        obj_v = local_v + y0

        # Batch depth projection with early validation
        x_val, y_val, z_val, _, _ = self._project_depth_points(obj_u, obj_v, depth_np, fx, fy, cx, cy)
        if z_val is None or len(z_val) == 0:
            seg_obj.points3D = self._cached_empty_cloud
            return seg_obj

        # Vectorized filtering - more efficient with numpy arrays
        z_arr = np.asarray(z_val, dtype=np.float32)
        x_arr = np.asarray(x_val, dtype=np.float32)
        y_arr = np.asarray(y_val, dtype=np.float32)

        # Combined validity check for better performance
        # Only filter by depth (y_arr > 0) and finite values - don't filter camera Y coordinate
        valid = (y_arr > 0) & np.isfinite(y_arr) & np.isfinite(x_arr) & np.isfinite(z_arr)
        if not np.any(valid):
            seg_obj.points3D = self._cached_empty_cloud
            return seg_obj
            
        # Filter arrays in-place for memory efficiency
        x_arr = x_arr[valid]
        y_arr = y_arr[valid]
        z_arr = z_arr[valid]

        point_cloud = ifaces.RoboCompImageSegmentation.PointCloud()
        point_cloud.X = x_arr.tolist()
        point_cloud.Y = y_arr.tolist()
        point_cloud.Z = z_arr.tolist()
        point_cloud.numberPoints = len(z_arr)
        point_cloud.timestamp = 0
        seg_obj.points3D = point_cloud

        return seg_obj


    def _process_segmentation(self, results, annotated_img, depth_np, fx, fy, cx, cy, img_height, img_width):
        # Memory reuse optimization: reuse seg_mask if size hasn't changed
        if self._reusable_seg_mask is None or self._last_img_size != (img_height, img_width):
            self._reusable_seg_mask = np.zeros((img_height, img_width), dtype=np.uint8)
            self._last_img_size = (img_height, img_width)
        else:
            self._reusable_seg_mask.fill(0)  # Reset instead of creating new
        
        seg_mask = self._reusable_seg_mask if self.display_enabled else None
        segmented_objects = []
        now = time.time()
        draw_overlay = self.display_enabled
        need_objects = (now - self.last_objects_request_time) <= self.objects_request_hold_seconds
        need_seg_mask = self.display_enabled

        result = results[0]

        if not (hasattr(result, 'masks') and result.masks is not None):
            return annotated_img, seg_mask, segmented_objects

        contours = result.masks.xy
        # GPU optimization: keep tensors on target device, convert to CPU only when needed
        classes_gpu = result.boxes.cls.to(self.device)
        confs_gpu = result.boxes.conf.to(self.device)
        names = self.yolo_model.names

        accepted_classes = ["bottle", "person"]
        # Pre-filter accepted classes to avoid processing unwanted objects
        accepted_indices = []
        for i, cls_gpu in enumerate(classes_gpu):
            cls = cls_gpu.item() if hasattr(cls_gpu, 'item') else cls_gpu
            class_name = names[int(cls)] if isinstance(names, (list, tuple)) else names.get(int(cls), str(cls))
            if class_name in accepted_classes:
                accepted_indices.append(i)
        
        if not accepted_indices:
            return annotated_img, seg_mask, segmented_objects

        for idx in accepted_indices:
            contour = contours[idx]
            cls_gpu = classes_gpu[idx]
            conf_gpu = confs_gpu[idx]
            
            # Convert individual values to CPU only when needed for numpy operations
            cls = cls_gpu.item() if hasattr(cls_gpu, 'item') else cls_gpu
            conf = conf_gpu.item() if hasattr(conf_gpu, 'item') else conf_gpu
            
            class_id = int(cls)
            class_name = names[class_id] if isinstance(names, (list, tuple)) else names.get(class_id, str(class_id))
            
            contour_i32 = np.asarray(contour, dtype=np.int32)
            if contour_i32.size == 0:
                continue

            contour_poly = contour_i32.reshape(-1, 1, 2)
            if draw_overlay:
                cv2.polylines(annotated_img, [contour_poly], isClosed=True, color=(0, 255, 0), thickness=2)
            if need_seg_mask:
                cv2.fillPoly(seg_mask, [contour_poly], 1)

            if draw_overlay:
                cv2.putText(
                    annotated_img,
                    f"{class_name} {float(conf):.2f}",
                    tuple(contour_i32[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

            if need_objects:
                segmented_objects.append(
                    self._build_segmented_object(contour_poly, class_name, conf, depth_np, fx, fy, cx, cy)
                )

        return annotated_img, seg_mask, segmented_objects

    def _process_pose_segmentation(self, results, annotated_img, depth_np, fx, fy, cx, cy, img_height, img_width):
        """
        Process YOLO pose model output to extract keypoints and object IDs.
        
        Returns:
            pose_objects: List of dictionaries containing:
                - id: Object identifier (index)
                - class_id: YOLO class ID
                - class_name: Class name
                - confidence: Detection confidence score
                - joints: List of joint dictionaries with keys:
                    - name/id: Joint identifier
                    - x: X coordinate in image
                    - y: Y coordinate in image
                    - confidence: Keypoint confidence
                    - x_world: 3D X coordinate (if depth available)
                    - y_world: 3D Y coordinate (if depth available)
                    - z_world: 3D Z coordinate (if depth available)
        """
        pose_objects = []
        now = time.time()
        need_objects = (now - self.last_objects_request_time) <= self.objects_request_hold_seconds
        
        result = results[0]
        
        # Check if pose data is available
        if not (hasattr(result, 'keypoints') and result.keypoints is not None):
            return pose_objects
        
        # Extract data efficiently
        boxes = result.boxes
        keypoints_data = result.keypoints
        names = self.yolo_model.names
        
        num_poses = len(boxes)
        if num_poses == 0:
            return pose_objects
        
        # GPU optimization: keep tensors on target device, convert to CPU only when needed
        classes_gpu = boxes.cls.to(self.device)
        confs_gpu = boxes.conf.to(self.device)
        
        # Extract keypoint coordinates and confidences
        keypoints_xy = keypoints_data.xy  # Shape: (num_poses, num_joints, 2)
        keypoints_conf = keypoints_data.conf if hasattr(keypoints_data, 'conf') else None  # Shape: (num_poses, num_joints)
        
        # Pre-filter for accepted classes if needed (optional - process all for now)
        for obj_idx in range(num_poses):
            try:
                # Convert GPU tensors to CPU values only when needed
                cls = classes_gpu[obj_idx].item() if hasattr(classes_gpu[obj_idx], 'item') else classes_gpu[obj_idx]
                conf = confs_gpu[obj_idx].item() if hasattr(confs_gpu[obj_idx], 'item') else confs_gpu[obj_idx]
                
                class_id = int(cls)
                class_name = names[class_id] if isinstance(names, (list, tuple)) else names.get(class_id, str(class_id))
                
                # Extract keypoints for this pose
                keypoints = keypoints_xy[obj_idx]  # Shape: (num_joints, 2)
                kp_confs = keypoints_conf[obj_idx] if keypoints_conf is not None else np.ones(len(keypoints))
                
                # Build joints list with efficient vectorized operations
                joints = []
                valid_joints = 0
                
                for joint_idx, (kp_xy, kp_conf) in enumerate(zip(keypoints, kp_confs)):
                    kp_conf_val = kp_conf.item() if hasattr(kp_conf, 'item') else kp_conf
                    
                    joint_dict = {
                        'id': joint_idx,
                        'name': f'joint_{joint_idx}',
                        'x': float(kp_xy[0]),
                        'y': float(kp_xy[1]),
                        'confidence': float(kp_conf_val)
                    }
                    
                    # Add 3D coordinates if depth is available and joint confidence is high
                    if depth_np is not None and kp_conf_val > 0.3:  # Confidence threshold
                        u, v = int(kp_xy[0]), int(kp_xy[1])
                        
                        # Validate pixel coordinates
                        h, w = depth_np.shape
                        if 0 <= u < w and 0 <= v < h:
                            z = depth_np[v, u]
                            
                            if z > 0 and np.isfinite(z):
                                # Project to 3D coordinates
                                x_cam = (u - cx) * z / fx
                                y_cam = (cy - v) * z / fy
                                
                                # Convert to RoboComp coordinate system
                                joint_dict['x_world'] = float(x_cam)
                                joint_dict['y_world'] = float(z)
                                joint_dict['z_world'] = float(y_cam)
                                valid_joints += 1
                    
                    joints.append(joint_dict)
                
                # Only include pose if we have at least some valid joints or always include (configurable)
                if joints:  # Include all poses with keypoints
                    pose_obj = {
                        'id': obj_idx,
                        'class_id': class_id,
                        'class_name': class_name,
                        'confidence': float(conf),
                        'joints': joints,
                        'valid_joints_3d': valid_joints
                    }
                    pose_objects.append(pose_obj)
                    
                    # Draw pose on image if display is enabled
                    if self.display_enabled:
                        self._draw_pose_on_image(annotated_img, joints, obj_idx)
            
            except Exception as e:
                print(f"Error processing pose {obj_idx}: {e}")
                continue
        
        return pose_objects
    
    def _modify_person_objects(self, segmented_objects, pose_objects, depth_np, fx, fy, cx, cy, annotated_img):
        """
        Modify segmented_objects list to update 'person' objects with pose information.
        
        For each pose object, find the corresponding segmented object (if any) based on class and proximity,
        and update the segmented object's point cloud to have only the center point calculated from keypoints.
        Also draw the computed center point on the annotated image.
        """
        h, w = depth_np.shape
        for pose in pose_objects:
            if pose['class_name'] == 'person':
                # Find the closest segmented object with label 'person'
                closest_obj = None
                closest_distance = float('inf')
                
                # Calculate the center using only chest-related joints.
                # COCO-style pose ordering: 5=left_shoulder, 6=right_shoulder, 11=left_hip, 12=right_hip.
                chest_joint_ids = {5, 6, 11, 12}
                chest_keypoints = [joint for joint in pose['joints'] if joint['confidence'] > 0.3 and joint['id'] in chest_joint_ids]
                
                if not chest_keypoints:
                    chest_keypoints = [joint for joint in pose['joints'] if joint['confidence'] > 0.3]

                keypoints_u = [joint['x'] for joint in chest_keypoints]
                keypoints_v = [joint['y'] for joint in chest_keypoints]

                if len(keypoints_u) >= 2:
                    min_u = min(keypoints_u)
                    max_u = max(keypoints_u)
                    min_v = min(keypoints_v)
                    max_v = max(keypoints_v)
                    pose_centroid_u = (min_u + max_u) / 2
                    pose_centroid_v = (min_v + max_v) / 2

                    u_center = int(round(pose_centroid_u))
                    v_center = int(round(pose_centroid_v))
                    if 0 <= u_center < w and 0 <= v_center < h:
                        cv2.circle(annotated_img, (u_center, v_center), 6, (0, 0, 255), -1)
                        cv2.putText(
                            annotated_img,
                            'center',
                            (u_center + 8, v_center - 8),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA,
                        )

                    for seg_obj in segmented_objects:
                        if seg_obj.label == 'person':
                            # Calculate distance between pose centroid and segmented object polygon centroid
                            if seg_obj.imagePolygon.numberPoints > 0:
                                poly_u = np.array(seg_obj.imagePolygon.U)
                                poly_v = np.array(seg_obj.imagePolygon.V)
                                centroid_u = np.mean(poly_u)
                                centroid_v = np.mean(poly_v)
                                
                                distance = np.sqrt((centroid_u - pose_centroid_u)**2 + (centroid_v - pose_centroid_v)**2)
                                if distance < closest_distance:
                                    closest_obj = seg_obj
                                    closest_distance = distance
                    
                    # If a closest object is found, update its point cloud with the center point
                    if closest_obj is not None:
                        u = int(pose_centroid_u)
                        v = int(pose_centroid_v)
                        if 0 <= u < w and 0 <= v < h:
                            z = depth_np[v, u]
                            if z > 0 and np.isfinite(z):
                                x_cam = (u - cx) * z / fx
                                y_cam = (cy - v) * z / fy
                                # Convert to RoboComp coordinate system
                                x_robocomp = x_cam
                                y_robocomp = z
                                z_robocomp = y_cam
                                
                                point_cloud = ifaces.RoboCompImageSegmentation.PointCloud()
                                point_cloud.X = [x_robocomp]
                                point_cloud.Y = [y_robocomp]
                                point_cloud.Z = [z_robocomp]
                                point_cloud.numberPoints = 1
                                point_cloud.timestamp = 0
                                closest_obj.points3D = point_cloud

                                # Draw the computed center point on the annotated image
                                cv2.circle(
                                    annotated_img,
                                    (u, v),
                                    6,
                                    (0, 0, 255),
                                    -1,
                                )
                                cv2.putText(
                                    annotated_img,
                                    'center',
                                    (u + 8, v - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (0, 0, 255),
                                    1,
                                    cv2.LINE_AA,
                                )

    


    def _draw_pose_on_image(self, img, joints, pose_id):
        """
        Draw pose keypoints and skeleton on image (optimized).
        
        Args:
            img: Image to draw on
            joints: List of joint dictionaries with x, y coordinates
            pose_id: ID of the pose for color variation
        """
        # Use different colors for different poses
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
        color = colors[pose_id % len(colors)]
        
        # Draw keypoints
        for joint in joints:
            x, y = int(joint['x']), int(joint['y'])
            conf = joint['confidence']
            
            # Only draw confident keypoints
            if conf > 0.3:
                radius = max(3, int(conf * 5))
                cv2.circle(img, (x, y), radius, color, -1)
                
                # Optional: draw keypoint ID
                cv2.putText(
                    img,
                    str(joint['id']),
                    (x + 5, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    color,
                    1
                )
        
        # Optional: Draw skeleton connections (common pose estimation connections)
        # Standard connections for 17-point COCO format
        skeleton_connections = [
            (0, 1), (0, 2), (1, 3), (2, 4),
            (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
            (5, 11), (6, 12), (11, 12), (11, 13), (13, 15),
            (12, 14), (14, 16)
        ]
        
        for joint_id_1, joint_id_2 in skeleton_connections:
            if joint_id_1 < len(joints) and joint_id_2 < len(joints):
                joint_1 = joints[joint_id_1]
                joint_2 = joints[joint_id_2]
                
                # Only draw if both joints are confident
                if joint_1['confidence'] > 0.3 and joint_2['confidence'] > 0.3:
                    pt1 = (int(joint_1['x']), int(joint_1['y']))
                    pt2 = (int(joint_2['x']), int(joint_2['y']))
                    cv2.line(img, pt1, pt2, color, 2)

    def _update_qt3d_from_mask(self, seg_mask, depth_np, fx, fy, cx, cy, img):
        if not self.display_enabled:
            return

        v, u = np.where(seg_mask > 0)
        x_robocomp, y_robocomp, z_robocomp, u_val, v_val = self._project_depth_points(u, v, depth_np, fx, fy, cx, cy)

        if y_robocomp is None:
            self.pos_attr.setCount(0)
            self.color_attr.setCount(0)
            return

        x_qt = -x_robocomp
        y_qt = z_robocomp
        z_qt = y_robocomp
        pts_array = np.column_stack((x_qt, y_qt, z_qt)).astype(np.float32)
        self.vertex_buffer.setData(QByteArray(pts_array.tobytes()))
        self.pos_attr.setCount(len(z_qt))

        point_colors = (img[v_val, u_val] / 255.0).astype(np.float32)
        self.color_buffer.setData(QByteArray(point_colors.tobytes()))
        self.color_attr.setCount(len(z_qt))

    def _update_2d_display(self, annotated_img):
        if not self.display_enabled:
            return

        # Convert BGR (OpenCV format) to RGB for proper display
        display_img = cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB)
        
        h, w, ch = display_img.shape
        bytes_per_line = ch * w
        # Force a deep copy so Qt does not keep a dangling pointer to numpy-owned memory.
        q_img = QImage(display_img.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
        self.image_label.setPixmap(QPixmap.fromImage(q_img))


    def setup_qt3d(self):
        # Set up a layout if the UI didn't create one
        if not self.layout():
            self.setLayout(QHBoxLayout())
            
        # 1. 2D Image Label
        self.image_label = QLabel()
        self.image_label.setMinimumSize(400, 300)
        self.layout().addWidget(self.image_label)
            
        # 2. Qt3D Window Setup
        self.view_3d = Qt3DExtras.Qt3DWindow()
        self.view_3d.defaultFrameGraph().setClearColor(QColor("black"))
        self.container = QWidget.createWindowContainer(self.view_3d, self)
        self.container.setMinimumSize(400, 300)
        self.layout().addWidget(self.container)
        
        self.root_entity = Qt3DCore.QEntity()
        self.view_3d.setRootEntity(self.root_entity)

        # Camera
        self.camera = self.view_3d.camera()
        self.camera.lens().setPerspectiveProjection(45.0, 16.0 / 9.0, 0.1, 1000.0)
        
        # Position camera at origin (0,0,0) matching the physical RGBD sensor
        # Look forward towards positive Z axis to match the depth direction usually used in robotics
        self.camera.setPosition(QVector3D(0, 0, 0))
        self.camera.setViewCenter(QVector3D(0, 0, 5.0))
        
        # In webots pointcloud, Y axis commonly points Upwards (ROS/Webots standard).
        # We need the camera Up vector to point +Y.
        self.camera.setUpVector(QVector3D(0, 1, 0))

        # Camera controls
        self.cam_controller = WebotsStyleCameraController(self.root_entity)
        self.cam_controller.setCamera(self.camera)

        # Point cloud setup
        self.pc_entity = Qt3DCore.QEntity(self.root_entity)
        self.geom_renderer = Qt3DRender.QGeometryRenderer(self.pc_entity)
        self.geometry = Qt3DCore.QGeometry(self.geom_renderer)
        
        # Position buffer
        self.vertex_buffer = Qt3DCore.QBuffer(self.geometry)
        self.pos_attr = Qt3DCore.QAttribute()
        self.pos_attr.setName(Qt3DCore.QAttribute.defaultPositionAttributeName())
        self.pos_attr.setAttributeType(Qt3DCore.QAttribute.VertexAttribute)
        self.pos_attr.setBuffer(self.vertex_buffer)
        self.pos_attr.setVertexBaseType(Qt3DCore.QAttribute.Float)
        self.pos_attr.setVertexSize(3)
        self.pos_attr.setByteOffset(0)
        self.pos_attr.setByteStride(12)
        
        # Color buffer
        self.color_buffer = Qt3DCore.QBuffer(self.geometry)
        self.color_attr = Qt3DCore.QAttribute()
        self.color_attr.setName(Qt3DCore.QAttribute.defaultColorAttributeName())
        self.color_attr.setAttributeType(Qt3DCore.QAttribute.VertexAttribute)
        self.color_attr.setBuffer(self.color_buffer)
        self.color_attr.setVertexBaseType(Qt3DCore.QAttribute.Float)
        self.color_attr.setVertexSize(3)
        self.color_attr.setByteOffset(0)
        self.color_attr.setByteStride(12)
        
        self.geometry.addAttribute(self.pos_attr)
        self.geometry.addAttribute(self.color_attr)
        self.geom_renderer.setGeometry(self.geometry)
        self.geom_renderer.setPrimitiveType(Qt3DRender.QGeometryRenderer.PrimitiveType.Points)

        self.material = Qt3DExtras.QPerVertexColorMaterial(self.root_entity)

        self.pc_entity.addComponent(self.geom_renderer)
        self.pc_entity.addComponent(self.material)


    ###################################################################################33333
    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompImageSegmentation.PointCloud from ifaces.RoboCompImageSegmentation")
        test = ifaces.RoboCompImageSegmentation.PointCloud()
        print(f"Testing RoboCompImageSegmentation.Polygon from ifaces.RoboCompImageSegmentation")
        test = ifaces.RoboCompImageSegmentation.Polygon()
        print(f"Testing RoboCompImageSegmentation.SegmentedObject from ifaces.RoboCompImageSegmentation")
        test = ifaces.RoboCompImageSegmentation.SegmentedObject()
        print(f"Testing RoboCompImageSegmentation.TImage from ifaces.RoboCompImageSegmentation")
        test = ifaces.RoboCompImageSegmentation.TImage()
        print(f"Testing RoboCompImageSegmentation.TDepth from ifaces.RoboCompImageSegmentation")
        test = ifaces.RoboCompImageSegmentation.TDepth()
        print(f"Testing RoboCompImageSegmentation.TData from ifaces.RoboCompImageSegmentation")
        test = ifaces.RoboCompImageSegmentation.TData()
        QTimer.singleShot(200, QApplication.instance().quit)

    ################################################################################

    # IMPLEMENTATION of getSegmentedObjects method from ImageSegmentation interface
    #
    def _objects_with_optional_points(self, objects, points3d):
        if points3d:
            return objects

        filtered_objects = []
        for obj in objects:
            filtered = ifaces.RoboCompImageSegmentation.SegmentedObject()
            filtered.label = obj.label
            filtered.score = obj.score
            filtered.imagePolygon = obj.imagePolygon
            filtered.points3D = ifaces.RoboCompImageSegmentation.PointCloud()  # Empty point cloud
            filtered_objects.append(filtered)
        return filtered_objects

    def ImageSegmentation_getSegmentedObjects(self, points3d, rgb):
        with self._state_lock:
            self.last_objects_request_time = time.time()
            objects, _, _, _ = self._latest_snapshot
        return self._objects_with_optional_points(objects, points3d)
    
    # IMPLEMENTATION of getAll method from ImageSegmentation interface
    def ImageSegmentation_getAll(self, points3d, rgb):
        with self._state_lock:
            self.last_image_request_time = time.time()
            self.last_objects_request_time = time.time()
            objects, image, depth, timestamp = self._latest_snapshot

        ret = ifaces.RoboCompImageSegmentation.TData()
        ret.objects = self._objects_with_optional_points(objects, points3d)
        ret.image = image
        ret.depth = depth
        ret.timestamp = timestamp
        return ret

    # IMPLEMENTATION of getDepth method from ImageSegmentation interface
    def ImageSegmentation_getDepth(self):
        with self._state_lock:
            _, _, depth, _ = self._latest_snapshot
        return depth
    
    #
    
    # IMPLEMENTATION of getImage method from ImageSegmentation interface
    #
    def ImageSegmentation_getImage(self):
        with self._state_lock:
            self.last_image_request_time = time.time()
            _, image, _, _ = self._latest_snapshot
        return image
    

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # RoboCompCameraRGBDSimple.TRGBD self.camerargbdsimple_proxy.getAll(str camera)
    # RoboCompCameraRGBDSimple.TDepth self.camerargbdsimple_proxy.getDepth(str camera)
    # RoboCompCameraRGBDSimple.TImage self.camerargbdsimple_proxy.getImage(str camera)
    # RoboCompCameraRGBDSimple.TPoints self.camerargbdsimple_proxy.getPoints(str camera)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # ifaces.RoboCompCameraRGBDSimple.Point3D
    # ifaces.RoboCompCameraRGBDSimple.TPoints
    # ifaces.RoboCompCameraRGBDSimple.TImage
    # ifaces.RoboCompCameraRGBDSimple.TDepth
    # ifaces.RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompImageSegmentation you can use this types:
    # ifaces.RoboCompImageSegmentation.PointCloud
    # ifaces.RoboCompImageSegmentation.Polygon
    # ifaces.RoboCompImageSegmentation.SegmentedObject
    # ifaces.RoboCompImageSegmentation.TImage
    # ifaces.RoboCompImageSegmentation.TDepth
    # ifaces.RoboCompImageSegmentation.TData


