#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

import sys
import os
import time
import numpy as np
import serial
from rich.console import Console
from PySide6.QtCore import QTimer, QPointF, QMutexLocker, Slot, Qt
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QColor, QPen, QBrush
import threading

# RoboComp e interfaces
sys.path.append('/opt/robocomp/lib')
import interfaces as ifaces
from genericworker import *
console = Console(highlight=False)
# Driver Hokuyo
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        # Agregamos la variable viewer para que no de error
        self.viewer = None

        # Variables de estado y tiempo
        self.start = time.time()
        self.counter = 0
        self.ldata_write = []
        self.ldata_read = []

        # Lista para persistir elementos gráficos y poder borrarlos
        self.draw_points = []
        self.mutex = threading.Lock()

        # --- Inicialización de objetos de apoyo ---
        # Asegúrate de que estas clases estén importadas o definidas
        # self.room_detector = RoomDetector()
        # self.hungarian = HungarianMatcher()
        # self.robot_pose = Pose2D() # Clase que maneje translate() y rotate()
        # self.nominal_room = NominalRoom()

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def setParams(self, params):
        try:
            uart_port = params["uart_port"]
            uart_speed = params["uart_speed"]
            print(f"Conectando a Lidar en: {uart_port} a {uart_speed} bps")

            laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=2)
            port = serial_port.SerialPort(laser_serial)
            self.laser = hokuyo.Hokuyo(port)
            res_on = self.laser.laser_on()
            print(f"Laser ON: {res_on}")
            return True
        except Exception as e:
            print(f"Error inicializando Lidar: {e}")
            return False

    @Slot()
    def compute(self):
        # En el código original de C++, compute llama a read_data
        self.read_data()

    def read_data(self):
        try:
            # 1. Obtener escaneo del driver
            scan = self.laser.get_single_scan()
            if not scan:
                return

            # 2. Procesar puntos y convertir a Cartesianas (x, y)
            current_points = []
            self.ldata_write = []

            for angle, dist in scan.items():
                rad_angle = np.radians(angle)

                # Filtrado básico de distancia
                if dist < 25:
                    dist = 5000  # Valor fuera de rango

                # Guardar para la interfaz RoboComp
                self.ldata_write.append(ifaces.RoboCompLaser.TData(rad_angle, dist))

                # Crear objeto punto para algoritmos internos (solo si es una lectura válida)
                if dist < 12000:
                    x = dist * np.cos(rad_angle)
                    y = dist * np.sin(rad_angle)
                    # Creamos un objeto simple con x e y (compatible con el resto del código)
                    p = type('Point', (object,), {'x': lambda: x, 'y': lambda: y, 'pos': QPointF(x, y)})
                    # Nota: Usamos x() y y() como métodos para mantener compatibilidad con C++
                    current_points.append(p)

            # 3. Filtrado de puntos aislados
            filter_values = self.filter_isolated_points(current_points, 200)
            if not filter_values:
                return

            # 4. Cálculos y Dibujo
            self.calculateDistances(filter_values)
            self.draw_lidar(filter_values)

            # 5. Localización (Hungarian + Mínimos Cuadrados)
            if hasattr(self, 'room_detector') and hasattr(self, 'nominal_room'):
                measurements_corners = self.room_detector.compute_corners(filter_values, self.viewer.scene)
                nominal_corners_on_robot_frame = self.nominal_room.transform_corners_to(self.robot_pose.inverse())

                # Match de esquinas (umbral 1500)
                matches = self.hungarian.match(measurements_corners, nominal_corners_on_robot_frame, 1500)

                if matches:
                    # Implementación de Eigen con NumPy
                    num_matches = len(matches)
                    W = np.zeros((num_matches * 2, 3))
                    b = np.zeros(num_matches * 2)

                    for i, (meas_c, nom_c, dist_match) in enumerate(matches):
                        p_meas = meas_c[0]  # Tomamos el punto de la esquina
                        p_nom = nom_c[0]

                        # Llenar vector de error b
                        b[2 * i] = p_nom.x() - p_meas.x()
                        b[2 * i + 1] = p_nom.y() - p_meas.y()

                        # Llenar matriz Jacobiana W
                        W[2 * i, :] = [1.0, 0.0, -p_meas.y()]
                        W[2 * i + 1, :] = [0.0, 1.0, p_meas.x()]

                    # Resolver W * r = b usando mínimos cuadrados
                    r, _, _, _ = np.linalg.lstsq(W, b, rcond=None)

                    if not np.isnan(r).any():
                        self.robot_pose.translate(r[0], r[1])
                        self.robot_pose.rotate(r[2])

            # 6. Actualizar UI
            self.draw_collisions()
            self.update_windows_values()

            # Gestión de frecuencia
            if time.time() - self.start > 1:
                print(f"Freq -> {self.counter} Hz")
                self.counter = 0
                self.start = time.time()
            else:
                self.counter += 1

            with self.mutex:
                # Intercambio de buffers para la interfaz externa
                self.ldata_read, self.ldata_write = self.ldata_write, self.ldata_read

        except Exception as e:
            print(f"Error en read_data: {e}")

    def draw_lidar(self, points):
        # Si el viewer no ha sido asignado, salimos de la funcion sin hacer nada
        if self.viewer is None:
            return

        # Borrar puntos anteriores
        for p in self.draw_points:
            try:
                self.viewer.scene.removeItem(p)
            except:
                pass
        self.draw_points.clear()

        pen = QPen(QColor("LightGreen"), 10)
        for p in points:
            # En Python usamos p.x() y p.y() porque los definimos como lambdas arriba
            dp = self.viewer.scene.addRect(-25, -25, 50, 50, pen)
            dp.setPos(p.x(), p.y())
            self.draw_points.append(dp)

        # Dibujar esquinas nominales
        try:
            color2 = QColor("cyan")
            brush2 = QBrush(color2)
            corners = self.nominal_room.transform_corners_to(self.robot_pose.inverse())
            for c_data in corners:
                p = c_data[0]
                i = self.viewer.scene.addEllipse(-100, -100, 200, 200, QPen(color2), brush2)
                i.setPos(p.x(), p.y())
                self.draw_points.append(i)
        except:
            pass

    # Métodos placeholder (deben estar implementados en tu clase o GenericWorker)
    def filter_isolated_points(self, points, threshold):
        # Implementar lógica de filtrado aquí
        return points

    def calculateDistances(self, points):
        pass

    def draw_collisions(self):
        pass

    def update_windows_values(self):
        pass

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # --- IMPLEMENTACIÓN DE INTERFACES ROBOTCOMP ---
    def Laser_getLaserAndBStateData(self):
        return [ifaces.RoboCompLaser.TLaserData(), ifaces.RoboCompGenericBase.TBaseState()]

    def Laser_getLaserConfData(self):
        return ifaces.RoboCompLaser.LaserConfData()

    def Laser_getLaserData(self):
        with self.mutex:
            return self.ldata_read

    ######################
    # From the RoboCompGenericBase you can call this methods:
    # RoboCompGenericBase.void self.genericbase_proxy.getBasePose(int x, int z, float alpha)
    # RoboCompGenericBase.void self.genericbase_proxy.getBaseState(RoboCompGenericBase.TBaseState state)

    ######################
    # From the RoboCompGenericBase you can use this types:
    # ifaces.RoboCompGenericBase.TBaseState

    ######################
    # From the RoboCompLaser you can use this types:
    # ifaces.RoboCompLaser.LaserConfData
    # ifaces.RoboCompLaser.TData

