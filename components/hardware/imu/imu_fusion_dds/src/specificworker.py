#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by NoeZC
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
"""
Componente imu_fusion_dds (RoboComp).

Dos modos, seleccionados por Simulation en etc/config -- misma interfaz de
salida en ambos, para que el resto de la arquitectura conmute sin más
cambios que ese valor:

  Simulation=false (robot real):
    1. Leer el Phidget real en proceso (250 Hz, callback del driver).
    2. Suscribirse a FullPoseEstimation de SVD48VBase (deltas mm/rad).
    3. Fusionar ambas fuentes con el ESKF (predict()/update_odom()).
    4. Publicar el pose fusionado (ver _publish_fused_odom).

  Simulation=true (Webots, vía webots-bridge):
    Sin IMU ni ESKF -- webots-bridge ya da posición y velocidad ground-truth
    (Supervisor.getPosition()/getVelocity(), sin ruido de sensor real que
    fusionar) por FullPoseEstimationPub. Se republica directamente (ver
    FullPoseEstimationPub_newFullPose/_publish_passthrough), evitando el
    ruido/deriva que introduce forzar eso por un ESKF pensado para sensores
    reales imperfectos (ver revisión de compatibilidad con webots-bridge).

  Publicación (ambos modos, mismo formato):
    - FullPoseEstimationPub (ICE): delta diferencial, mismo convenio que
      SVD48VBase -- consumido hoy por room_concept (solo adv/side/rot).
    - Plano DDS zero-copy (pose_dds_native, opcional, PublishDDS=true):
      velocidades + incertidumbre; x/y/yaw absolutos son ground-truth real
      en simulación, y se publican a 0 en el robot real (dead-reckoning sin
      ancla de corrección, ver nota en _publish_fused_odom).

Threading (solo aplica en Simulation=false; en simulación todo es
event-driven desde el hilo de ICE, sin hilo de driver ni RLock):
  - Hilo del driver Phidget → buffer interno.
  - Hilo de RoboComp ICE   → FullPoseEstimationPub_newFullPose() → cola thread-safe.
  - Hilo principal compute() → drain IMU → predict → update_odom → publish.
  Un único RLock protege el acceso al ESKF: predict y update_odom no son
  reentrantes entre sí.
"""

import csv
import json
import logging
import math
import queue
import subprocess
import sys
import threading
import time
import os
from enum import IntEnum, auto
from typing import Optional

import numpy as np
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
# Silenciar módulos ruidosos de terceros
for _noisy in ("Ice", "PySide6", "urllib3"):
    logging.getLogger(_noisy).setLevel(logging.WARNING)

console = Console(highlight=False)
log = logging.getLogger(__name__)

try:
    import setproctitle
    setproctitle.setproctitle(os.path.basename(os.getcwd()))
except Exception:
    pass

from src.ekf import ESKF, ESKFParams
from src.phidget import PhidgetIMU
# Publicador zero-copy FastDDS (common/media_transport, ver src/dds_publisher.h/.cpp
# + src/pose_dds_bindings.cpp) -- opcional en tiempo de import: si el .so no está
# compilado (o PublishDDS=false, ver initialize()) el componente sigue funcionando
# igual, solo sin ese plano de publicación adicional.
try:
    from src import pose_dds_native
except ImportError:
    pose_dds_native = None
# plot_viewer.py se lanza como proceso de sistema operativo aparte (subprocess.Popen,
# ver initialize()), NO se importa aquí — importarlo directamente sería inofensivo,
# pero mantenerlo desacoplado documenta la intención: nunca debe compartir el
# proceso ni el árbol de imports (Ice) de este módulo. Ver plot_viewer.py.
_PLOT_VIEWER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plot_viewer.py")


# ---------------------------------------------------------------------------
# Estado de la máquina de estados del componente
# ---------------------------------------------------------------------------
class AgentState(IntEnum):
    WAITING_IMU       = auto()   # esperando que el IMU (real o simulado) se conecte
    COLLECTING_STATIC = auto()   # acumulando muestras estáticas para init
    RUNNING           = auto()   # filtro activo
    DEGRADED          = auto()   # datos inválidos sostenidos; continúa con cautela
    FAULT             = auto()   # sin datos de IMU o base > timeout


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        if startup_check:
            self.startup_check()
        else:
            if self.initialize():
                self.timer.timeout.connect(self.compute)
                self.timer.start(self.Period)
            else:
                # Salir con código de error (no dejar el proceso vivo sin timer):
                # un supervisor externo (systemd, etc.) necesita un fallo visible
                # para reiniciar/alertar, no un proceso que parece arrancado sin
                # hacer nada.
                log.error("initialize() falló — componente no arrancado, saliendo")
                sys.exit(1)

        # Guardado determinista al salir: no depender de __del__/GC, cuyo timing
        # durante el shutdown del intérprete no está garantizado. aboutToQuit se
        # dispara siempre justo antes de que el event loop termine, tanto en
        # Ctrl+C como en quit().
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self._on_shutdown)

    def _on_shutdown(self) -> None:
        """Guarda los CSV de grabación (si hay datos) al cerrar el componente."""
        if getattr(self, 'recording', False):
            if getattr(self, '_rec', {}).get('t'):
                self.save_recording()
            if getattr(self, '_rec_odom', {}).get('t'):
                self.save_odom_recording()
            if getattr(self, '_rec_imu', {}).get('t'):
                self.save_imu_recording()
        if getattr(self, '_plot_proc', None) is not None:
            try:
                self._plot_proc.stdin.close()   # EOF: pide al hijo que cierre
            except Exception:
                pass
            try:
                self._plot_proc.wait(timeout=1.0)
            except Exception:
                self._plot_proc.terminate()

    def __del__(self):
        """Destructor: best-effort, termina el proceso de visualización si sigue vivo."""
        proc = getattr(self, '_plot_proc', None)
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass

    # ================================================================ initialize
    def initialize(self) -> bool:
        """
        Llamado una vez al arranque. Retorna True si el componente puede operar.
        Lee todos los parámetros del etc/config (secciones IMU, ESKF, Fusion).
        """
        log.info("=== imu_fusion_dds: initialize ===")

        cfg_fusion = self.configData.get("Fusion", {})

        # Publicar la odometría refinada por FullPoseEstimationPub, mismo
        # convenio de ejes que SVD48VBase, en un topic propio (ver
        # Proxies.FullPoseEstimationPubPrefix en config) para no interferir
        # con el topic "base" del que este componente es a la vez suscriptor.
        self.publish_fused_odom = bool(cfg_fusion.get("publish_fused_odom", True))

        self.simulation = bool(self.configData.get("Simulation", False))
        if self.simulation:
            # ---- Modo simulación: passthrough, sin IMU ni ESKF ----
            # webots-bridge ya expone posición y velocidad ground-truth
            # (Supervisor.getPosition()/getVelocity(), sin ruido de sensor
            # real que fusionar) por FullPoseEstimationPub -- forzar eso por
            # el ESKF (diseñado para fundir un IMU real ruidoso con
            # odometría real ruidosa) solo introduce el ruido/deriva que
            # costó una sesión entera de depuración diagnosticar (ver
            # revisión de compatibilidad con webots-bridge: bug de ejes en
            # el acelerómetro simulado, signo de pose.rot, divergencias por
            # la restricción planar...). Más simple y más correcto:
            # republicar directamente lo recibido (ver
            # FullPoseEstimationPub_newFullPose/_publish_passthrough). La
            # fusión ESKF+Phidget se deja tal cual para el robot real.
            self._sim_t_prev: Optional[float] = None
            self._sim_t_last_log = 0.0
            self._sim_log_interval_s = 1.0
            self.recording = False
            self.view = False
            log.info("Simulation=true — modo passthrough (sin IMU/ESKF), "
                     "republicando directamente adv/side/rot de webots-bridge")
        else:
            cfg_imu  = self.configData.get("IMU", {})
            cfg_eskf = self.configData.get("ESKF", {})

            # ---- Parámetros del driver IMU ----
            self._imu_interval_ms  = int(cfg_imu.get("interval_ms", 4))
            self._static_collect_s = float(cfg_imu.get("static_collect_s", 2.0))
            # rad/s — umbral de detección de reposo para la inicialización estática
            self._static_omega_thr = float(cfg_imu.get("static_omega_thr", 0.01))
            self._imu_timeout_s    = float(cfg_imu.get("timeout_s", 0.2))
            # Flags experimentales para probar el signo del yaw-rate (wz del gyro
            # y drz de la odometría) por separado — invertir para comparar contra
            # la convención actual (validada con datos: yaw_odom y la fusión
            # coinciden a <1° en un giro real, ver sesión de depuración). Poner
            # a False para volver al comportamiento normal.
            self._invert_imu_yaw_rate  = bool(cfg_imu.get("invert_yaw_rate", False))
            self._invert_odom_yaw_rate = bool(cfg_fusion.get("invert_odom_yaw_rate", False))

            # ---- Parámetros del filtro ESKF (Allan Variance, ver imu_params.json) ----
            params = ESKFParams(
                sigma_a              = float(cfg_eskf.get("sigma_a",      6.444261485614694e-04)),
                sigma_g              = float(cfg_eskf.get("sigma_g",      6.307237085088773e-05)),
                sigma_ba             = float(cfg_eskf.get("sigma_ba",     2.270468786768431e-05)),
                sigma_bg             = float(cfg_eskf.get("sigma_bg",     6.011926098425994e-06)),
                sigma_v_odom         = float(cfg_eskf.get("sigma_v_odom", 0.02)),
                sigma_w_odom         = float(cfg_eskf.get("sigma_w_odom", 0.01)),
                use_planar_constraint = True,
                # ---- Ruido adaptativo estático/dinámico (Bug #13, ver DEBUG_IMU_BASE_FUSION.md) ----
                enable_adaptive_noise = bool(cfg_eskf.get("enable_adaptive_noise", True)),
                sigma_a_static        = float(cfg_eskf.get("sigma_a_static",      0.012)),
                sigma_g_static        = float(cfg_eskf.get("sigma_g_static",      0.0007)),
                sigma_v_odom_static   = float(cfg_eskf.get("sigma_v_odom_static", 0.02)),
                sigma_w_odom_static   = float(cfg_eskf.get("sigma_w_odom_static", 0.01)),
                motion_detect_v_thr   = float(cfg_eskf.get("motion_detect_v_thr", 0.03)),
                motion_detect_w_thr   = float(cfg_eskf.get("motion_detect_w_thr", 0.05)),
                motion_hold_s         = float(cfg_eskf.get("motion_hold_s",       1.5)),
                motion_ramp_s         = float(cfg_eskf.get("motion_ramp_s",       2.0)),
            )
            self._eskf = ESKF(params)
            self._eskf_lock = threading.RLock()

            # ---- Parámetros de fusión / timeouts ----
            self._odom_timeout_s         = float(cfg_fusion.get("odom_timeout_s",         0.5))
            self._max_odom_queue         = int(cfg_fusion.get("max_odom_queue",           10))
            self._consec_reject_degraded = int(cfg_fusion.get("consec_reject_degraded",   10))
            self._consec_reject_fault    = int(cfg_fusion.get("consec_reject_fault",      30))

            # ---- IMU Phidget (robot real) ----
            self._imu = PhidgetIMU(data_interval_ms=self._imu_interval_ms)
            # ---- Rotación de montaje del IMU real (R_body_imu) ----
            # El Phidget está montado con Z apuntando hacia abajo (az = -g en reposo).
            # Esto equivale a una rotación R_x(180°) = diag(1, -1, -1):
            #   X_body =  X_imu  (frontal sin cambio)
            #   Y_body = -Y_imu  (izquierda ↔ derecha)
            #   Z_body = -Z_imu  (arriba ↔ abajo)
            # Verificación post-arranque: init_from_static debe reportar roll≈0°.
            # NOTA: diag(-1,1,-1) NO es una alternativa válida para un problema de
            # signo en el yaw-rate — el elemento [2,2] (el que afecta a wz) es -1 en
            # ambas opciones, así que cambiar entre ellas nunca altera el signo del
            # giro en Z. Confirmado con datos reales (corr(w_meas,w_pred)=-0.92,
            # ver odom_updates_*.csv) que el signo opuesto está en la odometría de
            # la base (drz), no en el montaje del IMU — el fix va en
            # _consume_odom_queue, no aquí.
            self._R_body_imu = np.diag([1., -1., -1.])

            if not self._imu.start(timeout_s=5.0):
                log.error("No se pudo conectar el IMU Phidget")
                return False

            # ---- Cola de mensajes de odometría ----
            # newFullPose() (hilo ICE) deposita aquí; compute() consume.
            self._odom_queue: queue.Queue = queue.Queue(maxsize=self._max_odom_queue)
            self._t_last_odom: float = time.monotonic()
            self._t_last_imu:  float = time.monotonic()
            self._t_prev_pose_ts_s: Optional[float] = None

            # ---- Pose de referencia para cómputo de deltas ----
            self._prev_pos = np.zeros(3)
            self._prev_yaw = 0.0
            self._last_eskf_state: Optional[dict] = None   # snapshot para la tabla de diagnóstico (_send_plot_frame)

            # ---- Máquina de estados ----
            self._state = AgentState.WAITING_IMU
            self._static_samples_a: list = []
            self._static_samples_g: list = []
            self._t_static_start:   Optional[float] = None

            # ---- Diagnóstico (rate-limit) ----
            self._t_last_imu_log = 0.0
            self._t_last_odom_log = 0.0
            self._t_last_fusion_log = 0.0
            self._diag_imu_interval_s = 2.0
            self._diag_odom_interval_s = 2.0
            self._diag_fusion_interval_s = 2.0
            self._diag_imu_samples = 0
            self._diag_imu_t0 = time.monotonic()

            # ================================================================
            # ---- Grabación y visualización de trayectoria ----
            # Activar/desactivar en tiempo real asignando True/False a estos flags.
            # Estadísticas (self._rec/_rec_odom/_rec_imu, ver save_*_recording):
            # por defecto desactivado -- activar en etc/config (Fusion.enable_recording)
            # solo para sesiones de depuración/validación, no en operación normal.
            self.recording = bool(cfg_fusion.get("enable_recording", False))
            # Ventana pyqtgraph en tiempo real (proceso aparte, ver plot_viewer.py).
            # El proceso hijo mantiene una ventana móvil de _WINDOW_S segundos
            # (por defecto 20s, ver plot_viewer.py) -- redibuja siempre sobre un
            # nº de puntos acotado, así que no se ralentiza por acumulación por
            # larga que sea la sesión. Desactivar aquí evita incluso lanzar el
            # proceso hijo.
            self.view = bool(cfg_fusion.get("enable_view", True))

            # Diccionario con listas paralelas; una fila por ciclo compute() en RUNNING
            self._rec: dict[str, list] = {
                # tiempo
                't':      [],   # [s] desde inicio de RUNNING
                # posición absoluta (frame local del ESKF, origen en el punto de arranque)
                'x':      [],   'y':      [],   'yaw':    [],   # [m] / [rad]
                # velocidad (frame mundo)
                'vx':     [],   'vy':     [],   'omega':  [],   # [m/s] / [rad/s]
                # incertidumbre marginal (1σ)
                'sx':     [],   'sy':     [],   'syaw':   [],   # [m] / [rad]
                'sz':     [],                                    # [m] sigma_pos[2] -- comparar horizontal (sx,sy) vs vertical
                'svx':    [],   'svy':    [],   'svz':    [],   # [m/s] sigma_vel completa (3 ejes)
                'sbax':   [],   'sbay':   [],   'sbaz':   [],   # [m/s²] sigma del sesgo acelerómetro (3 ejes)
                'sbgx':   [],   'sbgy':   [],   'sbgz':   [],   # [rad/s] sigma del sesgo giróscopo (3 ejes)
                # calidad del filtro
                'nis':    [],                                     # NIS ultimo update
                'motion_alpha': [],  # 0=régimen estático, 1=dinámico (Bug #13, ruido adaptativo)
                # posición relativa iteración a iteración (frame cuerpo previo)
                'dx':     [],   'dy':     [],   'dyaw':   [],   # [m] / [rad]
                'sdx':    [],   'sdy':    [],                    # sigma del delta [m]
                # trayectoria por odometría pura de la base (dead-reckoning, sin IMU)
                'x_odom': [],   'y_odom': [],   'yaw_odom': [],  # [m] / [rad]
            }
            self._rec_t0: Optional[float] = None  # t0 del primer sample RUNNING

            # ---- Dead-reckoning con SOLO odometría de la base (sin ESKF/IMU) ----
            # Se integra en _consume_odom_queue con cada mensaje, aceptado o no por
            # el ESKF, para tener una trayectoria de referencia independiente.
            self._odom_only_pos = np.zeros(2)
            self._odom_only_yaw = 0.0

            # Diccionario paralelo: una fila por cada update_odom() (~30 Hz), para
            # depurar alineación temporal imu+base y validar medida vs. predicción.
            self._rec_odom: dict[str, list] = {
                't':          [],   # [s] time.monotonic() absoluto — mismo reloj que _rec_imu['t'],
                                    # para poder alinear/correlar ambas series entre sí offline
                'dt_msg':     [],   # [s] entre mensajes de odometría consecutivos
                'lag_s':      [],   # [s] reloj de pared: recepción - pose.timestamp
                'dx_body':    [],   'dy_body':    [],   'drz':        [],  # medida cruda [m]/[rad]
                'vx_meas':    [],   'vy_meas':    [],   'w_meas':     [],  # medida [m/s]/[rad/s]
                'vx_pred':    [],   'vy_pred':    [],   'w_pred':     [],  # predicción ESKF
                'mahalanobis':[],   'accepted':   [],
            }

            # Diccionario paralelo: una fila por cada muestra IMU (~250 Hz),
            # cruda tras R_body_imu (sin quitar sesgo) + estado del filtro en
            # ese instante, para calibrar offline el extrínseco del IMU a
            # partir de tramos rectos.
            self._rec_imu: dict[str, list] = {
                't':    [],
                'ax':   [], 'ay':   [], 'az':   [],   # accel frame cuerpo [m/s²], cruda
                'wx':   [], 'wy':   [], 'wz':   [],   # gyro  frame cuerpo [rad/s], cruda
                'bax':  [], 'bay':  [], 'baz':  [],   # sesgo acelerómetro estimado (ba)
                'roll': [], 'pitch':[], 'yaw':  [],   # actitud estimada (para quitar gravedad offline)
            }

            # Plot: proceso de sistema operativo aparte (src/plot_viewer.py), lanzado
            # con subprocess.Popen -- NO multiprocessing.Process. Con multiprocessing
            # el hijo tendría que reimportar generated/imu_fusion_dds.py (Ice/phidget
            # se importan a nivel de módulo ahí, antes del __main__ guard) para poder
            # resolver la función objetivo, arrastrando esas dependencias pesadas y
            # sus hilos de fondo — visto en vivo: ~35 hilos en el hijo y el
            # componente bloqueado. subprocess.Popen ejecuta plot_viewer.py como
            # script aislado, sin tocar ese árbol de imports en absoluto.
            # IPC: una línea JSON por frame en su stdin (ver _send_plot_frame),
            # nunca bloqueante para compute() -- ver manejo de BrokenPipeError ahí.
            self._view_counter  = 0
            self._VIEW_EVERY    = 10      # 100 ms @ 100 Hz → cadencia de envío ~10 Hz
            self._plot_sent_idx = 0       # índice de self._rec ya enviado al proceso hijo
            self._plot_proc     = None
            if self.view:
                try:
                    self._plot_proc = subprocess.Popen(
                        [sys.executable, _PLOT_VIEWER_PATH],
                        stdin=subprocess.PIPE, text=True, bufsize=1)
                    # Sin contorno de room que dibujar (ver nota en _publish_fused_odom
                    # sobre la retirada de la corrección de room_concept vía DSR).
                    self._plot_proc.stdin.write(json.dumps({"room_contour": None}) + "\n")
                    self._plot_proc.stdin.flush()
                    log.info("Proceso de visualización lanzado (pid=%d)", self._plot_proc.pid)
                except Exception as exc:
                    log.warning("No se pudo lanzar el proceso de visualización (%s) → view=False", exc)
                    self.view = False

        # ---- Publicación zero-copy DDS (FastDDS) del pose fusionado ----
        # Plano ADICIONAL al FullPoseEstimationPub (ICE, deltas mm/s convenio
        # SVD48VBase) -- este va en SI (m, rad, m/s, rad/s), para consumidores
        # nuevos que quieran leerlo por zero-copy. Ver dds_publisher.h.
        cfg_dds = self.configData.get("DDS", {})
        self.publish_dds = bool(self.configData.get("PublishDDS", False))
        self._dds_pub = None
        if self.publish_dds:
            if pose_dds_native is None:
                log.warning("PublishDDS=true pero pose_dds_native no está compilado (ver src/CMakeLists.txt) — desactivando")
                self.publish_dds = False
            else:
                dds_cfg = pose_dds_native.Config()
                if "Domain" in cfg_dds:           dds_cfg.domain_id = int(cfg_dds["Domain"])
                if "Topic" in cfg_dds:             dds_cfg.topic = str(cfg_dds["Topic"])
                if "HistoryDepth" in cfg_dds:      dds_cfg.history_depth = int(cfg_dds["HistoryDepth"])
                if "SharedMemoryOnly" in cfg_dds:  dds_cfg.shared_memory_only = bool(cfg_dds["SharedMemoryOnly"])
                if "DataSharing" in cfg_dds:       dds_cfg.data_sharing = bool(cfg_dds["DataSharing"])
                self._dds_pub = pose_dds_native.PoseDDSPublisher()
                if not self._dds_pub.init(dds_cfg):
                    log.warning("DDS pose media plane init FAILED — desactivando PublishDDS")
                    self._dds_pub = None
                    self.publish_dds = False

        if self.simulation:
            log.info("initialize OK (simulation=true, passthrough)")
        else:
            log.info("initialize OK (simulation=false) — esperando IMU y reposo para calibración")
        return True

    # ================================================================= compute
    @QtCore.Slot()
    def compute(self) -> None:
        """
        Llamado periódicamente por el scheduler de RoboComp (~100 Hz).
        Envoltorio fino: cualquier excepción no prevista en _compute_impl()
        (numérica en el ESKF, fallo del binding DDS nativo, etc.) se loguea
        con traceback y se descarta -- sin este guardado, una excepción que
        escape de un slot Qt puede matar el timer en silencio (el proceso
        sigue vivo respondiendo a ICE, pero deja de fusionar sin ningún
        aviso de FAULT), que es peor que perder un ciclo de compute().
        """
        try:
            self._compute_impl()
        except Exception:
            log.exception("compute(): excepción no controlada, se descarta este ciclo")

    def _compute_impl(self) -> None:
        """
        Drain del buffer IMU → predict; consume odometría → update; publica.
        En modo simulación (Simulation=true) no hay nada que hacer aquí: la
        publicación es event-driven desde FullPoseEstimationPub_newFullPose
        (ver _publish_passthrough), sin IMU ni ESKF.
        """
        if self.simulation:
            return

        now = time.monotonic()

        # ---- Drenar muestras IMU y propagar el filtro ----
        # Debe ir antes del chequeo de timeout: si no, _t_last_imu nunca se
        # refresca una vez entrado en FAULT y el estado queda bloqueado para
        # siempre aunque el IMU vuelva a enviar datos.
        samples = self._imu.drain()
        if samples:
            self._t_last_imu = now
            if self._state == AgentState.FAULT:
                if self._eskf.is_initialized:
                    self._state = (AgentState.DEGRADED
                                    if now - self._t_last_odom > self._odom_timeout_s
                                    else AgentState.RUNNING)
                else:
                    self._state = AgentState.WAITING_IMU
                log.info("IMU recuperado tras FAULT → %s", self._state.name)
            self._dispatch_imu(samples)
            self._diag_imu_samples += len(samples)
            if now - self._t_last_imu_log > self._diag_imu_interval_s:
                dt = max(1e-6, now - self._diag_imu_t0)
                hz = self._diag_imu_samples / dt
                latest = samples[-1]
                log.info("IMU: %d muestras (%.1f Hz) ax=%.3f ay=%.3f az=%.3f | wx=%.4f wy=%.4f wz=%.4f",
                         self._diag_imu_samples, hz,
                         latest.accel[0], latest.accel[1], latest.accel[2],
                         latest.gyro[0], latest.gyro[1], latest.gyro[2])
                self._diag_imu_samples = 0
                self._diag_imu_t0 = now
                self._t_last_imu_log = now

        # ---- Detección de FAULT/DEGRADED por timeout ----
        if now - self._t_last_odom > self._odom_timeout_s:
            if self._state == AgentState.RUNNING:
                log.warning("Sin odometría desde %.1f s → DEGRADED", self._odom_timeout_s)
                self._state = AgentState.DEGRADED
        if now - self._t_last_imu > self._imu_timeout_s:
            if self._state != AgentState.FAULT:
                log.error("Sin IMU desde %.1f s → FAULT", self._imu_timeout_s)
            self._state = AgentState.FAULT
            return

        # ---- Estado WAITING_IMU: aún no hay conexión ----
        if self._state == AgentState.WAITING_IMU:
            if self._imu.is_connected:
                self._state = AgentState.COLLECTING_STATIC
                self._t_static_start = now
                log.info("IMU conectado → recogiendo datos estáticos (%.1f s)",
                         self._static_collect_s)
            return

        # ---- Estado COLLECTING_STATIC: acumular para init_from_static ----
        if self._state == AgentState.COLLECTING_STATIC:
            self._try_static_init()
            return

        # ---- Estados RUNNING / DEGRADED: consumir cola de odometría ----
        self._consume_odom_queue()

        if self._eskf.is_initialized:
            self._publish_pose()
            if self.view:
                self._view_counter += 1
                if self._view_counter >= self._VIEW_EVERY:
                    self._view_counter = 0
                    self._send_plot_frame()

        if now - self._t_last_fusion_log > self._diag_fusion_interval_s and self._eskf.is_initialized:
            state = self._eskf.get_state()
            log.info("Fusion: pos=(%.3f %.3f %.3f) yaw=%.3f rad | sig_pos=(%.3f %.3f %.3f)",
                     state["pos"][0], state["pos"][1], state["pos"][2], state["yaw"],
                     state["sigma_pos"][0], state["sigma_pos"][1], state["sigma_pos"][2])
            self._t_last_fusion_log = now

    def startup_check(self):
        print(f"Testing RoboCompIMU.Acceleration from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Acceleration()
        print(f"Testing RoboCompIMU.Gyroscope from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Gyroscope()
        print(f"Testing RoboCompIMU.Magnetic from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Magnetic()
        print(f"Testing RoboCompIMU.Orientation from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Orientation()
        print(f"Testing RoboCompIMU.DataImu from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.DataImu()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    def FullPoseEstimationPub_newFullPose(self, pose) -> None:
        """
        Callback del topic FullPoseEstimation. Llamado en el hilo de ICE.

        En simulación (Simulation=true) el publicador es webots-bridge
        (receiving_robotSpeed()): republica directamente, sin IMU ni ESKF
        (ver _publish_passthrough).

        En robot real (Simulation=false) el publicador es SVD48VBase:
        pose.x/y/rz son DELTAS en mm y rad, convención RoboComp (Y=avance,
        X=lateral; ver DifferentialRobot_setSpeedBase/calcOdometry en
        SVD48VBase). Sólo deposita en la cola; no llama al ESKF desde este
        hilo. El remapeo a la convención del ESKF (X=frontal, Y=izquierda,
        REP-103) se hace en _consume_odom_queue.
        """
        if self.simulation:
            # Guarda explícita: IceStorm entrega esta llamada como oneway, así
            # que una excepción sin capturar aquí se descarta en silencio (sin
            # traceback, sin log) -- este callback quedaría "mudo" sin ningún
            # aviso de qué falló.
            try:
                self._publish_passthrough(pose)
            except Exception:
                log.exception("_publish_passthrough: excepción no controlada")
            return

        msg = {
            "dx_mm":     pose.x,
            "dy_mm":     pose.y,
            "drz_rad":   pose.rz,
            "t_recv":    time.monotonic(),   # para dt_msg (robusto a saltos NTP)
            "pose_ts_ms": pose.timestamp,    # reloj de pared de SVD48VBase (time()*1000)
            "t_recv_wall": time.time(),      # reloj de pared local, comparable a pose_ts_ms
        }
        try:
            self._odom_queue.put_nowait(msg)
        except queue.Full:
            # Cola llena: descartar el más antiguo para no bloquear el hilo ICE
            try:
                self._odom_queue.get_nowait()
                self._odom_queue.put_nowait(msg)
            except queue.Empty:
                pass

    def _publish_passthrough(self, pose) -> None:
        """
        Simulation=true: sin IMU ni ESKF. webots-bridge (receiving_robotSpeed())
        ya da posición absoluta (Supervisor.getPosition(), metros, ground
        truth) y velocidad body-frame ya calculada (Supervisor.getVelocity(),
        sin integrar) en pose.adv/side/rot -- se asume correcta (viene del
        propio simulador, no de un sensor real con ruido) y se republica tal
        cual, con el mismo formato de salida (ICE diferencial + DDS) que usa
        el robot real, para que el resto de la arquitectura no distinga el
        modo. Llamado desde el hilo de ICE (FullPoseEstimationPub_newFullPose).
        """
        now_s = pose.timestamp * 1e-3
        dt = (now_s - self._sim_t_prev) if self._sim_t_prev is not None else 0.01
        dt = max(1e-4, min(dt, 0.5))
        self._sim_t_prev = now_s

        vx_body = pose.adv    # m/s, avance -> ESKF X, ground truth de Webots
        vy_body = pose.side   # m/s, lateral -> ESKF Y
        omega_z = pose.rot    # rad/s, CCW positive (convenio REP-103, confirmado hoy)

        diff = {
            "dx": vx_body * dt, "dy": vy_body * dt, "dyaw": omega_z * dt,
            "vx": vx_body, "vy": vy_body, "omega_z": omega_z,
        }
        # pose.rz: yaw absoluto ground truth (atan2 de la orientación real del
        # robot en Webots, no integrado) -- se usa para pasar la velocidad de
        # cuerpo a frame mundo (vel) y, en el plano DDS, como pose absoluta
        # real (a diferencia del robot real, aquí sí hay ancla fiable: no es
        # dead-reckoning, es la posición/orientación real del simulador).
        # Normalizado a [-pi,pi] por seguridad: pose.rz sale de un atan2 en
        # webots-bridge (debería venir ya acotado), pero se envuelve aquí para
        # no depender de que eso se mantenga así.
        yaw = (pose.rz + math.pi) % (2 * math.pi) - math.pi
        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        vel_world = np.array([cyaw * vx_body - syaw * vy_body,
                               syaw * vx_body + cyaw * vy_body])
        state = {
            "pos":         np.array([pose.x, pose.y, 0.0]),  # absoluto, ground truth (m)
            "yaw":         yaw,
            "vel":         vel_world,
            "sigma_pos":   np.zeros(3),
            "sigma_theta": np.zeros(3),
            "sigma_vel":   np.zeros(3),
        }
        if self.publish_fused_odom:
            self._publish_fused_odom(diff, state)

        # Diagnóstico rate-limited (~1 Hz): pose absoluta ground-truth +
        # velocidad body-frame, para poder depurar a ojo sin CSV/plot (ambos
        # desactivados en este modo, ver initialize()).
        now = time.monotonic()
        if now - self._sim_t_last_log > self._sim_log_interval_s:
            log.info("Sim: pos=(%.3f %.3f) yaw=%.3f rad | adv=%.3f side=%.3f rot=%.3f rad/s | dt=%.3fs",
                     pose.x, pose.y, yaw, vx_body, vy_body, omega_z, dt)
            self._sim_t_last_log = now

    # ===================================================================
    # ===================================================================

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getMediaDescriptor method from MediaPlaneDDS interface
    #
    def MediaPlaneDDS_getMediaDescriptor(self) -> str:
        """
        Descriptor JSON (rc::media::MediaDescriptor) del plano DDS de pose, para
        que robot_concept lo relaye al grafo. "" si PublishDDS=false o el plano
        no llegó a inicializarse (ver initialize()).
        """
        if self._dds_pub is not None:
            return self._dds_pub.descriptor_json()
        return ""

    # ===================================================================
    # ===================================================================

    # ============================================================ internos
    def _dispatch_imu(self, samples: list) -> None:
        """
        Propaga el ESKF con cada muestra IMU desde el último compute().
        En COLLECTING_STATIC acumula en los buffers de inicialización.
        """
        with self._eskf_lock:
            for s in samples:
                # Transformar al frame del cuerpo antes de cualquier uso.
                # R_body_imu compensa el montaje físico del Phidget (identidad
                # en simulación, ver initialize()).
                a = self._R_body_imu @ s.accel_np
                g = self._R_body_imu @ s.gyro_np
                if self._invert_imu_yaw_rate:
                    g[2] = -g[2]
                if self._state == AgentState.COLLECTING_STATIC:
                    self._static_samples_a.append(a)
                    self._static_samples_g.append(g)
                elif self._state in (AgentState.RUNNING, AgentState.DEGRADED):
                    self._eskf.predict(a, g, s.timestamp_s)
                    if self.recording:
                        self._record_imu_step(a, g, self._eskf.get_state())

    def _try_static_init(self) -> None:
        """
        Inicializa el ESKF cuando se han acumulado suficientes muestras estáticas.
        Verifica que el robot estuvo inmóvil durante la colección.
        """
        if time.monotonic() - self._t_static_start < self._static_collect_s:
            return

        if not self._static_samples_a:
            # No llegaron muestras: reintentar
            self._t_static_start = time.monotonic()
            return

        a_buf = np.array(self._static_samples_a)
        g_buf = np.array(self._static_samples_g)
        self._static_samples_a.clear()
        self._static_samples_g.clear()

        omega_rms = float(np.sqrt(np.mean(g_buf ** 2)))
        if omega_rms > self._static_omega_thr:
            log.warning("Robot en movimiento durante init (|ω|=%.4f rad/s); reintentando",
                        omega_rms)
            self._t_static_start = time.monotonic()
            return

        with self._eskf_lock:
            ok = self._eskf.init_from_static(a_buf, g_buf)

        if ok:
            self._state = AgentState.RUNNING
            log.info("ESKF inicializado. Iniciando fusión en RUNNING.")
        else:
            log.warning("init_from_static falló; reintentando")
            self._t_static_start = time.monotonic()

    def _consume_odom_queue(self) -> None:
        """
        Consume todos los mensajes de odometría pendientes.
        Cada mensaje produce un update_odom del ESKF.
        """
        while not self._odom_queue.empty():
            try:
                msg = self._odom_queue.get_nowait()
            except queue.Empty:
                break

            t_recv = msg["t_recv"]
            self._t_last_odom = t_recv

            # dt entre mensajes de odometría consecutivos: se calcula con el
            # timestamp de ORIGEN (pose.timestamp, reloj de SVD48VBase), no con
            # el instante de llegada local (t_recv). La entrega por ICE puede
            # llegar en ráfaga (varios mensajes con gap de recepción de pocos
            # cientos de µs aunque en el origen se generaran ~10 ms separados),
            # y SVD48VBase ya integra sus propios deltas usando su timestamp
            # (ver calcOdometry: dt = (timestamp - last_timestamp)/1000).
            # Usar t_recv aquí puede dividir un delta normal entre un dt casi
            # nulo y disparar velocidades espurias (visto hasta 93 rad/s en
            # test dinámico real, ver DEBUG_IMU_BASE_FUSION.md).
            pose_ts_s = msg["pose_ts_ms"] * 1e-3
            dt_msg = ((pose_ts_s - self._t_prev_pose_ts_s)
                      if self._t_prev_pose_ts_s is not None else 0.01)
            dt_msg = max(1e-4, min(dt_msg, 0.2))  # clamping de seguridad
            self._t_prev_pose_ts_s = pose_ts_s

            # SVD48VBase publica en su propia convención de ejes (RoboComp:
            # Y = avance, X = lateral; ver DifferentialRobot_setSpeedBase/
            # calcOdometry), mientras que el ESKF asume X = frontal,
            # Y = izquierda (REP-103). Se remapea aquí:
            dx_body_m = msg["dy_mm"] * 1e-3   # avance  (base.y) → ESKF X
            dy_body_m = msg["dx_mm"] * 1e-3   # lateral (base.x) → ESKF Y
            # Signo invertido respecto al yaw-rate del IMU: confirmado con datos
            # reales (corr(w_meas,w_pred)=-0.92 antes de este fix; ver análisis
            # en odom_updates_*.csv). No es un problema de montaje del IMU (ver
            # nota en self._R_body_imu) sino de la convención de signo de rz en
            # SVD48VBase.
            drz = -msg["drz_rad"]
            if self._invert_odom_yaw_rate:
                drz = -drz
            v_body_xy = np.array([dx_body_m / dt_msg, dy_body_m / dt_msg])
            omega_z   = drz / dt_msg

            # Sanity check: rechazar deltas de overflow o singularidades.
            if abs(dx_body_m) > 0.30 or abs(dy_body_m) > 0.30 or abs(drz) > 1.0:
                log.warning("Delta odom fuera de rango (dx=%.3f dy=%.3f drz=%.3f); ignorando",
                            dx_body_m, dy_body_m, drz)
                continue

            # Dead-reckoning independiente del ESKF: integra SIEMPRE (incluso si el
            # ESKF rechaza el update), para tener una trayectoria de referencia
            # basada únicamente en odometría de la base.
            cyaw, syaw = math.cos(self._odom_only_yaw), math.sin(self._odom_only_yaw)
            self._odom_only_pos[0] += cyaw * dx_body_m - syaw * dy_body_m
            self._odom_only_pos[1] += syaw * dx_body_m + cyaw * dy_body_m
            self._odom_only_yaw = (self._odom_only_yaw + drz + math.pi) % (2 * math.pi) - math.pi

            with self._eskf_lock:
                upd = self._eskf.update_odom(v_body_xy, omega_z, t_odom=t_recv)
            accepted = upd["accepted"]

            if not accepted:
                log.debug("update_odom rechazado (Mahalanobis=%s)", upd["mahalanobis"])

            # Latencia base→imu_fusion_dds en reloj de pared (mismo host: comparable)
            latency_s = msg["t_recv_wall"] - msg["pose_ts_ms"] * 1e-3

            if self.recording:
                self._record_odom_step(dt_msg, dx_body_m, dy_body_m, drz,
                                        v_body_xy, omega_z, upd, latency_s)

            now = time.monotonic()
            if now - self._t_last_odom_log > self._diag_odom_interval_s:
                log.info("Base: dx=%.3f dy=%.3f drz=%.3f | v=(%.3f %.3f) m/s w=%.3f rad/s "
                         "accepted=%s mahal=%s lag=%.1fms",
                         dx_body_m, dy_body_m, drz, v_body_xy[0], v_body_xy[1], omega_z,
                         accepted, upd["mahalanobis"], latency_s * 1000.0)
                self._t_last_odom_log = now

    def _publish_fused_odom(self, diff: dict, state: dict) -> None:
        """
        Publica el delta refinado (odom+IMU) por FullPoseEstimationPub, en el
        topic propio de este componente (Proxies.FullPoseEstimationPubPrefix,
        p.ej. "imuFusion" — distinto del "base" del que se es suscriptor).

        Mismo convenio de ejes y mismos campos que SVD48VBase (para que
        cualquier consumidor ya preparado para su publicación, incluido el
        propio _consume_odom_queue de este componente, pueda leer este topic
        sin cambios): RoboComp usa Y=avance/X=lateral, y el signo de rz/vrz
        está invertido respecto al yaw del ESKF -- es la misma conversión que
        _consume_odom_queue aplica en sentido contrario al leer de la base
        (ver comentario allí sobre la convención de signo de rz en SVD48VBase).
        """
        pose = ifaces.RoboCompFullPoseEstimation.FullPoseEuler()
        pose.x  = diff["dy"] * 1000.0     # lateral (ESKF Y-izq) -> base.x, mm
        pose.y  = diff["dx"] * 1000.0     # avance  (ESKF X-frontal) -> base.y, mm
        pose.z  = 0.0
        pose.rx = 0.0
        pose.ry = 0.0
        pose.rz = diff["dyaw"]            # sin invertir: coincide con el convenio de diff/state usado en el plot (_record_step), ya validado visualmente
        pose.vx = -diff["vy"] * 1000.0    # m/s -> mm/s, mismo convenio que SVD48VBase
        pose.vy =  diff["vx"] * 1000.0    # m/s -> mm/s, mismo convenio que SVD48VBase
        pose.vz = 0.0
        pose.vrx = 0.0
        pose.vry = 0.0
        pose.vrz = diff["omega_z"]         # sin invertir: idem — el -1 anterior no coincidía con el plot correcto
        pose.ax = 0.0; pose.ay = 0.0; pose.az = 0.0
        pose.arx = 0.0; pose.ary = 0.0; pose.arz = 0.0
        pose.adv  = pose.vy
        pose.side = pose.vx
        pose.rot  = pose.vrz
        pose.confidence = 0      # sin usar (int en el IDL), igual que en SVD48VBase
        pose.timestamp = np.longlong(time.time() * 1000)
        try:
            self.fullposeestimationpub_proxy.newFullPose(pose)
        except Ice.Exception as e:
            log.warning("publish_fused_odom: fallo al publicar (%s)", e)

        # Plano DDS adicional (SI) -- ver initialize(). state/diff son los mismos
        # ya calculados para este ciclo (nada se recalcula).
        # x/y/yaw (posición absoluta): en simulación es la pose ground-truth
        # real de Webots (state["pos"]/state["yaw"], sin deriva -- ver
        # _publish_passthrough), así que sí se publica tal cual. En el robot
        # real es dead-reckoning puro (sin corrección de room_concept desde
        # que este agente se convirtió en componente) y publicarlo como si
        # fuera una posición absoluta fiable induciría a error a los
        # consumidores del plano DDS, así que se publica a 0. Las velocidades
        # (vx/vy/omega, adv/side/rot) sí son válidas en ambos modos.
        if self._dds_pub is not None:
            if self.simulation:
                x0, y0, yaw0 = float(state["pos"][0]), float(state["pos"][1]), float(state["yaw"])
            else:
                x0, y0, yaw0 = 0.0, 0.0, 0.0
            try:
                self._dds_pub.publish(
                    int(pose.timestamp),
                    x0, y0, yaw0,
                    float(state["vel"][0]), float(state["vel"][1]), float(diff["omega_z"]),
                    float(diff["vx"]), float(-diff["vy"]), float(diff["omega_z"]),
                    float(state["sigma_pos"][0]), float(state["sigma_pos"][1]), float(state["sigma_theta"][2]),
                    float(state["sigma_vel"][0]), float(state["sigma_vel"][1]),
                    0)
            except Exception as e:
                log.warning("publish_fused_odom: fallo al publicar en el plano DDS (%s)", e)

    def _publish_pose(self) -> None:
        """
        Obtiene el estado actual del ESKF, actualiza la pose de referencia de
        deltas y graba/visualiza.

        TODO: no hay guarda de sanidad numérica sobre el estado del ESKF (NaN/
        Inf en pos/vel, covarianza no positiva-definida, etc.). El estado
        actual solo detecta FAULT por timeout de sensores (ver compute()); si
        el filtro divergiera numéricamente seguiría "RUNNING" publicando
        basura sin que nada lo detecte. Pendiente decidir política (¿reset del
        ESKF? ¿forzar FAULT?) antes de confiar en esto para producción.
        """
        with self._eskf_lock:
            state = self._eskf.get_state()
            diff  = self._eskf.get_differential(self._prev_pos, self._prev_yaw)
            self._prev_pos = state["pos"].copy()
            self._prev_yaw = state["yaw"]
            self._last_eskf_state = state

        if self.publish_fused_odom and self._eskf.is_initialized:
            self._publish_fused_odom(diff, state)

        # self._rec alimenta dos cosas independientes: la persistencia a CSV
        # (self.recording, off por defecto — ver save_recording/_on_shutdown)
        # y el dibujado en tiempo real (self.view, on por defecto). Hace falta
        # rellenarlo si CUALQUIERA de las dos está activa, si no la ventana
        # del plot se queda vacía aunque el proceso siga corriendo.
        if self.recording or self.view:
            self._record_step(state, diff)

    # ================================================================
    # ---- Grabación y visualización de trayectoria ----
    # ================================================================

    def _record_step(self, state: dict, diff: dict) -> None:
        """Añade una fila de datos a self._rec. Llamado desde _publish_pose()."""
        if self._rec_t0 is None:
            self._rec_t0 = time.monotonic()
        t = time.monotonic() - self._rec_t0
        r = self._rec
        r['t'].append(t)
        r['x'].append(float(state['pos'][0]))
        r['y'].append(float(state['pos'][1]))
        r['yaw'].append(float(state['yaw']))
        r['vx'].append(float(state['vel'][0]))
        r['vy'].append(float(state['vel'][1]))
        r['omega'].append(float(diff['omega_z']))
        r['sx'].append(float(state['sigma_pos'][0]))
        r['sy'].append(float(state['sigma_pos'][1]))
        r['syaw'].append(float(state['sigma_theta'][2]))
        r['sz'].append(float(state['sigma_pos'][2]))
        r['svx'].append(float(state['sigma_vel'][0]))
        r['svy'].append(float(state['sigma_vel'][1]))
        r['svz'].append(float(state['sigma_vel'][2]))
        r['sbax'].append(float(state['sigma_ba'][0]))
        r['sbay'].append(float(state['sigma_ba'][1]))
        r['sbaz'].append(float(state['sigma_ba'][2]))
        r['sbgx'].append(float(state['sigma_bg'][0]))
        r['sbgy'].append(float(state['sigma_bg'][1]))
        r['sbgz'].append(float(state['sigma_bg'][2]))
        r['nis'].append(float(state['nis_last']))
        r['motion_alpha'].append(float(state['motion_alpha']))
        r['dx'].append(float(diff['dx']))
        r['dy'].append(float(diff['dy']))
        r['dyaw'].append(float(diff['dyaw']))
        r['sdx'].append(float(diff['sigma_dx']))
        r['sdy'].append(float(diff['sigma_dy']))
        # Trayectoria por odometría pura (dead-reckoning, sin IMU) para comparar
        # visualmente contra la fusión y aislar si la deriva viene del ESKF o de la base.
        r['x_odom'].append(float(self._odom_only_pos[0]))
        r['y_odom'].append(float(self._odom_only_pos[1]))
        r['yaw_odom'].append(float(self._odom_only_yaw))

    def _record_odom_step(self, dt_msg: float, dx_body_m: float, dy_body_m: float,
                          drz: float, v_body_xy: np.ndarray, omega_z: float,
                          upd: dict, latency_s: float) -> None:
        """Añade una fila a self._rec_odom. Llamado desde _consume_odom_queue (~30 Hz)."""
        r = self._rec_odom
        r['t'].append(time.monotonic())
        r['dt_msg'].append(dt_msg)
        r['lag_s'].append(latency_s)
        r['dx_body'].append(dx_body_m)
        r['dy_body'].append(dy_body_m)
        r['drz'].append(drz)
        r['vx_meas'].append(float(v_body_xy[0]))
        r['vy_meas'].append(float(v_body_xy[1]))
        r['w_meas'].append(omega_z)
        vp = upd["v_body_pred"]
        r['vx_pred'].append(float(vp[0]) if vp is not None else float('nan'))
        r['vy_pred'].append(float(vp[1]) if vp is not None else float('nan'))
        r['w_pred'].append(upd["omega_pred"] if upd["omega_pred"] is not None else float('nan'))
        r['mahalanobis'].append(upd["mahalanobis"] if upd["mahalanobis"] is not None else float('nan'))
        r['accepted'].append(bool(upd["accepted"]))

    def _record_imu_step(self, a: np.ndarray, g: np.ndarray, state: dict) -> None:
        """Añade una fila a self._rec_imu. Llamado desde _dispatch_imu (~250 Hz real / ~100 Hz simulación)."""
        r = self._rec_imu
        r['t'].append(time.monotonic())
        r['ax'].append(float(a[0])); r['ay'].append(float(a[1])); r['az'].append(float(a[2]))
        r['wx'].append(float(g[0])); r['wy'].append(float(g[1])); r['wz'].append(float(g[2]))
        ba = state['ba']
        r['bax'].append(float(ba[0])); r['bay'].append(float(ba[1])); r['baz'].append(float(ba[2]))
        r['roll'].append(float(state['roll']))
        r['pitch'].append(float(state['pitch']))
        r['yaw'].append(float(state['yaw']))

    def save_imu_recording(self, path: str | None = None) -> str | None:
        """
        Guarda self._rec_imu (una fila por muestra IMU cruda, tras R_body_imu)
        en un CSV. Si path=None genera /tmp/imu_samples_YYYYMMDD_HHMMSS.csv.
        """
        if not self._rec_imu['t']:
            log.warning("save_imu_recording: sin datos grabados")
            return None
        if path is None:
            path = f"/tmp/imu_samples_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        fields = list(self._rec_imu.keys())
        n = len(self._rec_imu['t'])
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(fields)
            for i in range(n):
                w.writerow([self._rec_imu[k][i] for k in fields])
        log.info("Muestras IMU guardadas: %s  (%d filas)", path, n)
        return path

    def save_odom_recording(self, path: str | None = None) -> str | None:
        """
        Guarda self._rec_odom (una fila por update_odom, medida vs. predicción)
        en un CSV. Si path=None genera /tmp/odom_updates_YYYYMMDD_HHMMSS.csv.
        """
        if not self._rec_odom['t']:
            log.warning("save_odom_recording: sin datos grabados")
            return None
        if path is None:
            path = f"/tmp/odom_updates_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        fields = list(self._rec_odom.keys())
        n = len(self._rec_odom['t'])
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(fields)
            for i in range(n):
                w.writerow([self._rec_odom[k][i] for k in fields])
        log.info("Updates de odometría guardados: %s  (%d filas)", path, n)
        return path

    def save_recording(self, path: str | None = None) -> str | None:
        """
        Guarda self._rec en un CSV.
        Si path=None genera /tmp/traj_YYYYMMDD_HHMMSS.csv.
        Devuelve la ruta del archivo o None si no hay datos.

        Columnas:
            t, x, y, yaw, vx, vy, omega, sx, sy, syaw,
            sz, svx, svy, svz, sbax, sbay, sbaz, sbgx, sbgy, sbgz, nis,
            motion_alpha, dx, dy, dyaw, sdx, sdy
        """
        if not self._rec['t']:
            log.warning("save_recording: sin datos grabados")
            return None
        if path is None:
            path = f"/tmp/traj_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        fields = list(self._rec.keys())
        n = len(self._rec['t'])
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(fields)
            for i in range(n):
                w.writerow([self._rec[k][i] for k in fields])
        log.info("Trayectoria guardada: %s  (%d muestras, %.1f s)",
                 path, n, self._rec['t'][-1] if n else 0.0)
        return path

    def _build_state_table(self) -> dict | None:
        """
        Snapshot JSON-serializable de get_state() para la tabla de variables
        internas del filtro (ver plot_viewer.py) -- a diferencia de self._rec,
        no acumula histórico: cada envío sustituye al anterior, el proceso
        hijo solo necesita el valor MÁS RECIENTE para pintar la tabla.
        """
        s = self._last_eskf_state
        if s is None:
            return None
        return {
            "pos":         s["pos"].tolist(),
            "vel":         s["vel"].tolist(),
            "rpy_deg":     [math.degrees(s["roll"]), math.degrees(s["pitch"]), math.degrees(s["yaw"])],
            "ba":          s["ba"].tolist(),
            "bg_deg":      np.degrees(s["bg"]).tolist(),
            "sigma_pos":   s["sigma_pos"].tolist(),
            "sigma_vel":   s["sigma_vel"].tolist(),
            "sigma_rpy_deg": np.degrees(s["sigma_theta"]).tolist(),
            "sigma_ba":    s["sigma_ba"].tolist(),
            "sigma_bg_deg": np.degrees(s["sigma_bg"]).tolist(),
            "nis_last":    s["nis_last"],
            "nis_mean":    s["nis_mean"],
            "n_updates":   s["n_updates"],
            "n_rejected":  s["n_rejected"],
            "n_consec_rej": s["n_consec_rej"],
            "motion_alpha": s["motion_alpha"],   # 0=estático, 1=dinámico (Bug #13, ruido adaptativo)
        }

    def _send_plot_frame(self) -> None:
        """
        Envía a src/plot_viewer.py (proceso aparte, ver initialize()) solo las
        filas NUEVAS de self._rec desde el último envío — nunca el histórico
        completo, para no recargar la serialización/IPC a medida que crece la
        sesión. El proceso hijo acumula su propia copia y redibuja por su
        cuenta (ver plot_viewer.main). Una línea JSON por frame en su stdin.
        Si el hijo murió (pipe roto) se desactiva view y no se reintenta.
        Incluye además "table": snapshot (no acumulativo) de get_state()
        completo -- ver _build_state_table -- para la tabla de variables
        internas del filtro (sesgos, covarianzas, NIS...), útil para
        diagnosticar deriva sin depender de lo que ya se grafica (p.ej.
        deriva lateral en estático: ver vel/sigma_vel/ba en la tabla aunque
        x,y en la trayectoria apenas se muevan).
        """
        if self._plot_proc is None:
            return
        r = self._rec
        n = len(r['t'])
        i0 = self._plot_sent_idx
        if n <= i0:
            table = self._build_state_table()
            if table is None:
                return
            payload = {"table": table}
            try:
                self._plot_proc.stdin.write(json.dumps(payload) + "\n")
                self._plot_proc.stdin.flush()
            except (BrokenPipeError, OSError) as e:
                log.warning("Proceso de visualización no disponible (%s) → view=False", e)
                self.view = False
                self._plot_proc = None
            return
        keys = ('t', 'x', 'y', 'yaw', 'sx', 'sy', 'nis', 'x_odom', 'y_odom')
        payload = {k: r[k][i0:n] for k in keys}
        payload["table"] = self._build_state_table()
        self._plot_sent_idx = n
        try:
            self._plot_proc.stdin.write(json.dumps(payload) + "\n")
            self._plot_proc.stdin.flush()
        except (BrokenPipeError, OSError) as e:
            log.warning("Proceso de visualización no disponible (%s) → view=False", e)
            self.view = False
            self._plot_proc = None
