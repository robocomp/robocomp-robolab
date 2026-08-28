"""
phidget.py — Lectura del IMU Phidget Spatial Precision 3/3/3.

Proporciona un objeto PhidgetIMU con:
  - Callback asíncrono registrado en el driver (hilo propio del driver Phidget).
  - Buffer circular thread-safe de muestras IMU.
  - API simple para el agente: drain() y get_latest().
  - Detección de pérdida de conexión y reconexión automática.

Dependencias:
    pip install Phidget22
    + drivers nativos: https://www.phidgets.com/docs/OS_-_Linux

Uso típico:
    imu = PhidgetIMU(data_interval_ms=4)   # 250 Hz
    if not imu.start():
        raise RuntimeError("No se pudo conectar el IMU")
    ...
    samples = imu.drain()   # vaciado atómico del buffer
    for s in samples:
        eskf.predict(s.accel, s.gyro, s.timestamp_s)
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger(__name__)

try:
    from Phidget22.Devices.Spatial import Spatial
    from Phidget22.Phidget import PhidgetException
    _PHIDGET_AVAILABLE = True
except ImportError:
    _PHIDGET_AVAILABLE = False
    log.warning("Phidget22 no disponible — PhidgetIMU en modo simulado")

# Conversiones a SI (el driver devuelve en g y °/s)
_G_TO_MPS2  = 9.80665
_DEG_TO_RAD = math.pi / 180.0
_ATTACH_TIMEOUT_MS = 5_000
_MAX_BUFFER = 1_024          # muestras; ~4 s a 250 Hz antes de truncar


@dataclass(frozen=True)
class IMUSample:
    """Una muestra del IMU en unidades SI, frame del cuerpo."""
    timestamp_s:  float          # tiempo en segundos (monotónico; timestamp HW del Phidget)
    timestamp_us: int            # mismo valor en microsegundos (entero, para DSR)
    accel:        tuple[float, float, float]   # m/s²  (incluye gravedad; SIN restar sesgo)
    gyro:         tuple[float, float, float]   # rad/s
    mag:          tuple[float, float, float]   # Gauss (NaN si no disponible)
    temperature:  float          # °C

    @property
    def accel_np(self):
        import numpy as np
        return np.array(self.accel)

    @property
    def gyro_np(self):
        import numpy as np
        return np.array(self.gyro)


class PhidgetIMU:
    """
    Interfaz al IMU Phidget Spatial Precision 3/3/3.

    El driver Phidget llama a _on_data() en su propio hilo. Los datos se
    depositan en un deque protegido por un Lock. El agente los consume en
    su propio ciclo con drain() o consulta la última muestra con get_latest().
    """

    def __init__(self,
                 data_interval_ms: int = 4,
                 serial:    Optional[int] = None,
                 hub_port:  Optional[int] = None,
                 max_buffer: int = _MAX_BUFFER):

        self._interval_ms   = max(1, data_interval_ms)
        self._serial        = serial
        self._hub_port      = hub_port
        self._max_buffer    = max_buffer

        self._lock:   threading.Lock = threading.Lock()
        self._buffer: deque[IMUSample] = deque(maxlen=max_buffer)
        self._latest: Optional[IMUSample] = None
        self._connected: bool = False
        self._t0_hw:  Optional[float] = None  # primer timestamp HW (ms) → ancla

        self._spatial: Optional[object] = None  # Phidget22.Spatial

    # ------------------------------------------------------------- lifecycle
    def start(self, timeout_s: float = 5.0) -> bool:
        """Abre el dispositivo y espera el attach. Retorna True si OK."""
        if not _PHIDGET_AVAILABLE:
            log.error("Phidget22 no instalado. Ejecuta: pip install Phidget22")
            return False

        self._spatial = Spatial()
        if self._serial is not None:
            self._spatial.setDeviceSerialNumber(self._serial)
        if self._hub_port is not None:
            self._spatial.setHubPort(self._hub_port)

        self._spatial.setOnAttachHandler(self._on_attach)
        self._spatial.setOnDetachHandler(self._on_detach)
        self._spatial.setOnErrorHandler(self._on_error)
        self._spatial.setOnSpatialDataHandler(self._on_data)

        try:
            self._spatial.openWaitForAttachment(int(timeout_s * 1000))
            return True
        except PhidgetException as ex:
            log.error("No se pudo conectar el IMU: %s", ex)
            return False

    def stop(self) -> None:
        """Cierra la conexión limpiamente."""
        if self._spatial is not None:
            try:
                self._spatial.close()
            except Exception:
                pass
            self._spatial = None
        self._connected = False
        log.info("PhidgetIMU: cerrado")

    # -------------------------------------------------------------- consumer API
    def drain(self) -> list[IMUSample]:
        """
        Vacía atómicamente el buffer y retorna todas las muestras acumuladas,
        ordenadas de más antigua a más reciente.
        Llamar en cada ciclo del agente antes de predict().
        """
        with self._lock:
            samples = list(self._buffer)
            self._buffer.clear()
        return samples

    def get_latest(self) -> Optional[IMUSample]:
        """Última muestra recibida, sin vaciar el buffer."""
        with self._lock:
            return self._latest

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def sample_rate_hz(self) -> float:
        return 1000.0 / self._interval_ms

    # ------------------------------------------------------------- diagnóstico
    def collect_static(self, duration_s: float = 2.0) -> tuple:
        """
        Bloquea durante duration_s y retorna (accel_buf, gyro_buf) como arrays
        numpy de forma (N, 3). Útil para llamar init_from_static() del ESKF.
        Presupone que el IMU ya está conectado.
        """
        import numpy as np
        # Vaciar buffer previo
        self.drain()
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            time.sleep(0.05)
        samples = self.drain()
        if not samples:
            return np.empty((0, 3)), np.empty((0, 3))
        accel = np.array([s.accel for s in samples])
        gyro  = np.array([s.gyro  for s in samples])
        log.info("collect_static: %d muestras en %.1f s (%.0f Hz efectivos)",
                 len(samples), duration_s, len(samples) / duration_s)

        return accel, gyro

    # ------------------------------------------- callbacks del driver Phidget
    def _on_attach(self, _) -> None:
        assert self._spatial is not None
        try:
            min_di = self._spatial.getMinDataInterval()
            di = max(self._interval_ms, min_di)
            self._spatial.setDataInterval(di)
            log.info("IMU attached: dataInterval=%d ms (%.0f Hz)", di, 1000/di)
        except PhidgetException as ex:
            log.warning("No se pudo configurar dataInterval: %s", ex)
        self._connected = True
        self._t0_hw = None  # resetear ancla temporal en reconexión

    def _on_detach(self, _) -> None:
        self._connected = False
        log.warning("IMU detached")

    def _on_error(self, _, code, description) -> None:
        log.error("IMU error [%d]: %s", code, description)

    def _on_data(self, _, acceleration, angularRate, magneticField,
                 timestamp_ms: float) -> None:
        """
        Callback llamado en el hilo del driver Phidget.
        acceleration : [ax, ay, az] en g
        angularRate  : [wx, wy, wz] en °/s
        magneticField: [mx, my, mz] en Gauss (puede ser None)
        timestamp_ms : timestamp hardware del Phidget en ms
        """
        # Ancla: primer timestamp HW → origen de tiempo monotónico en segundos
        if self._t0_hw is None:
            self._t0_hw = timestamp_ms

        t_s  = (timestamp_ms - self._t0_hw) * 1e-3
        t_us = int(timestamp_ms * 1e3)  # microsegundos absolutos del HW

        ax = acceleration[0] * _G_TO_MPS2
        ay = acceleration[1] * _G_TO_MPS2
        az = acceleration[2] * _G_TO_MPS2
        wx = angularRate[0]  * _DEG_TO_RAD
        wy = angularRate[1]  * _DEG_TO_RAD
        wz = angularRate[2]  * _DEG_TO_RAD

        if magneticField is not None and len(magneticField) == 3:
            mx, my, mz = float(magneticField[0]), float(magneticField[1]), float(magneticField[2])
        else:
            mx = my = mz = float('nan')

        # Temperatura: disponible sólo en algunos modelos; ignorar si falla
        temp = 0.0
        try:
            if self._spatial is not None:
                temp = self._spatial.getTemperature()
        except Exception:
            pass

        sample = IMUSample(
            timestamp_s  = t_s,
            timestamp_us = t_us,
            accel = (ax, ay, az),
            gyro  = (wx, wy, wz),
            mag   = (mx, my, mz),
            temperature = temp,
        )

        with self._lock:
            self._buffer.append(sample)
            self._latest = sample
