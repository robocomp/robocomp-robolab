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
from ultralytics import YOLO

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

        # Load the latest YOLO model (YOLO26 large) for object detection
        print("Loading YOLO model...")
        try:
            self.yolo_model = YOLO('yolo26m-seg.engine')
        except Exception as e:
            self.yolo_model = YOLO('yolo26m-seg.pt')
        print("YOLO loaded.")

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

            img = np.frombuffer(img_struct.image, dtype=np.uint8)

            img = img.reshape(img_struct.height, img_struct.width, img_struct.depth)

            results = self.yolo_model(img, verbose=False)
            
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

            self._update_qt3d_from_mask(seg_mask, depth_np, fx, fy, cx, cy, img)

            image_out = self._build_output_image(img_struct, img.tobytes())
            depth_out = self._build_output_depth(depth_struct)

            with self._state_lock:
                if self._shutting_down:
                    return
                self._latest_snapshot = (segmented_objects, image_out, depth_out, int(time.time() * 1000))

            self._update_2d_display(annotated_img)
            
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

                if self._proxy_thread_stop.is_set():
                    break
                self._update_proxy_wait_from_period(rgbd)

                self._proxy_queue.put(rgbd, timeout=0.1)
                
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

        # Build imagePolygon using numpy for efficient reshape
        contour_points = contour_i32.reshape(-1, 2)
        polygon = ifaces.RoboCompImageSegmentation.Polygon()
        polygon.U = contour_points[:, 0].tolist()
        polygon.V = contour_points[:, 1].tolist()
        polygon.numberPoints = len(contour_points)
        polygon.timestamp = 0
        seg_obj.imagePolygon = polygon

        def _empty_cloud():
            cloud = ifaces.RoboCompImageSegmentation.PointCloud()
            cloud.X = []
            cloud.Y = []
            cloud.Z = []
            cloud.numberPoints = 0
            cloud.timestamp = 0
            return cloud

        x0, y0, width, height = cv2.boundingRect(contour_i32)
        if width <= 0 or height <= 0:
            seg_obj.points3D = _empty_cloud()
            return seg_obj

        # Rasterize mask and get pixel coords with numpy
        local_mask = np.zeros((height, width), dtype=np.uint8)
        shifted = (contour_points - np.array([x0, y0], dtype=np.int32)).reshape(-1, 1, 2)
        cv2.fillPoly(local_mask, [shifted], 1)

        local_v, local_u = np.where(local_mask > 0)
        obj_u = local_u + x0
        obj_v = local_v + y0

        x_val, y_val, z_val, _, _ = self._project_depth_points(obj_u, obj_v, depth_np, fx, fy, cx, cy)
        if z_val is None:
            seg_obj.points3D = _empty_cloud()
            return seg_obj

        # Filter out invalid (zero/nan) depth points vectorially
        z_arr = np.asarray(z_val)
        x_arr = np.asarray(x_val)
        y_arr = np.asarray(y_val)

        valid = (z_arr > 0) & np.isfinite(z_arr)
        x_arr, y_arr, z_arr = x_arr[valid], y_arr[valid], z_arr[valid]

        point_cloud = ifaces.RoboCompImageSegmentation.PointCloud()
        point_cloud.X = x_arr.tolist()
        point_cloud.Y = y_arr.tolist()
        point_cloud.Z = z_arr.tolist()
        point_cloud.numberPoints = len(z_arr)
        point_cloud.timestamp = 0
        seg_obj.points3D = point_cloud

        return seg_obj


    def _process_segmentation(self, results, annotated_img, depth_np, fx, fy, cx, cy, img_height, img_width):
        seg_mask = np.zeros((img_height, img_width), dtype=np.uint8)
        segmented_objects = []
        now = time.time()
        draw_overlay = self.display_enabled
        need_objects = (now - self.last_objects_request_time) <= self.objects_request_hold_seconds
        need_seg_mask = self.display_enabled

        result = results[0]

        if not (hasattr(result, 'masks') and result.masks is not None):
            return annotated_img, seg_mask, segmented_objects

        contours = result.masks.xy
        classes = result.boxes.cls.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        names = self.yolo_model.names

        for contour, cls, conf in zip(contours, classes, confs):
            contour_i32 = np.asarray(contour, dtype=np.int32)
            if contour_i32.size == 0:
                continue

            contour_poly = contour_i32.reshape(-1, 1, 2)
            if draw_overlay:
                cv2.polylines(annotated_img, [contour_poly], isClosed=True, color=(0, 255, 0), thickness=2)
            if need_seg_mask:
                cv2.fillPoly(seg_mask, [contour_poly], 1)

            class_id = int(cls)
            label_name = names[class_id] if isinstance(names, (list, tuple)) else names.get(class_id, str(class_id))
            if draw_overlay:
                cv2.putText(
                    annotated_img,
                    f"{label_name} {float(conf):.2f}",
                    tuple(contour_i32[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

            if need_objects:
                segmented_objects.append(
                    self._build_segmented_object(contour_poly, label_name, conf, depth_np, fx, fy, cx, cy)
                )

        return annotated_img, seg_mask, segmented_objects

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

        h, w, ch = annotated_img.shape
        bytes_per_line = ch * w
        # Force a deep copy so Qt does not keep a dangling pointer to numpy-owned memory.
        q_img = QImage(annotated_img.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
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


