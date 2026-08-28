"""
simulated_imu.py — Emulación del IMU real para el path de simulación (Webots).

Cuando Simulation=true en etc/config, SpecificWorker usa SimulatedIMU en vez de
PhidgetIMU: en vez de leer el driver Phidget en proceso, pide muestras al
componente que expone el sensor IMU del simulador (p.ej. webots-bridge) a
través del proxy Ice imu_proxy (interfaz RoboCompIMU, ver imu_fusion_dds.cdsl).

Mismo contrato público que PhidgetIMU (start/stop/is_connected/drain) para que
el resto del pipeline (_dispatch_imu, máquina de estados) no distinga el
origen de las muestras.

Nota: se usan getAcceleration()/getAngularVel() por separado, no getDataImu()
-- el servidor de referencia en simulación (webots-bridge) no tiene
getDataImu() implementado (ver IMU_getDataImu en su specificworker.cpp, que
solo imprime un warning "not implemented" y devuelve la estructura vacía).
"""
from __future__ import annotations

import logging
import time
from typing import Optional

import Ice

from src.phidget import IMUSample

log = logging.getLogger(__name__)


class SimulatedIMU:
    """
    Sondea imu_proxy (RoboCompIMU) una vez por llamada a drain() y empaqueta
    el resultado como una única IMUSample, en el reloj monotónico local del
    propio agente -- el simulador no garantiza un timestamp de origen útil
    (p.ej. webots-bridge no rellena Acceleration.timestamp/Gyroscope.timestamp).
    """

    def __init__(self, imu_proxy):
        self._imu_proxy = imu_proxy
        self._connected = False
        self._t0: Optional[float] = None

    # ------------------------------------------------------------- lifecycle
    def start(self, timeout_s: float = 5.0) -> bool:
        """Comprueba que el imu_proxy responde. Retorna True si OK."""
        try:
            self._imu_proxy.getAcceleration()
        except Ice.Exception as e:
            log.error("No se pudo conectar al IMU simulado (imu_proxy): %s", e)
            return False
        self._connected = True
        return True

    def stop(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # -------------------------------------------------------------- consumer API
    def drain(self) -> list[IMUSample]:
        """
        Pide una muestra al simulador y la devuelve como lista de un elemento
        (o vacía si falla la llamada Ice) -- mismo contrato que
        PhidgetIMU.drain() (vaciado del buffer), para que _dispatch_imu no
        distinga el origen de las muestras.
        """
        try:
            acc = self._imu_proxy.getAcceleration()
            gyro = self._imu_proxy.getAngularVel()
        except Ice.Exception as e:
            log.warning("SimulatedIMU: fallo leyendo imu_proxy (%s)", e)
            self._connected = False
            return []
        self._connected = True

        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now

        sample = IMUSample(
            timestamp_s=now - self._t0,
            timestamp_us=int(now * 1e6),
            accel=(acc.XAcc, acc.YAcc, acc.ZAcc),
            gyro=(gyro.XGyr, gyro.YGyr, gyro.ZGyr),
            mag=(float("nan"), float("nan"), float("nan")),
            temperature=0.0,
        )
        return [sample]
