"""
ekf.py — Error-State Kalman Filter para fusión IMU + odometría de ruedas.

Estado nominal (16):  p ∈ R³, v ∈ R³, q ∈ S³ (world←body), b_a ∈ R³, b_g ∈ R³
Estado de error (15): [δp(3), δv(3), δθ(3), δb_a(3), δb_g(3)]

Convenciones:
  - Frame mundo : ENU, Z-up.  Gravedad g = [0, 0, -9.80665] m/s².
  - Frame cuerpo: X frontal, Y izquierda, Z arriba (ROS REP-103).
  - Cuaternión  : [w, x, y, z], hemisferio canónico (w ≥ 0).
  - Parámetros de ruido: PSD-sqrt (salida directa del análisis Allan).
      Q[v,v]  = σ_a²  · Δt · I₃   (NO Δt²; σ_a en m/s²/√Hz)
      Q[θ,θ]  = σ_g²  · Δt · I₃
      Q[ba,ba]= σ_ba² · Δt · I₃
      Q[bg,bg]= σ_bg² · Δt · I₃

Referencia: Solà, J. (2017). arXiv:1711.02508.
"""
from __future__ import annotations

import math
import time
import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------
G_WORLD       = np.array([0.0, 0.0, -9.80665])  # m/s², ENU
CHI2_3_99     = 11.345   # χ²(3 dof, 99 %) para gating Mahalanobis
CHI2_2_99     = 9.210    # χ²(2 dof, 99 %) — update_room_pose (solo x,y, sin yaw)
DELTA_MAX_M   = 0.30     # sanity: rechazo si |Δxy| > 30 cm en un paso
DELTA_MAX_RAD = 1.0      # sanity: rechazo si |Δθ| > 1 rad en un paso (~57°)


# ---------------------------------------------------------------------------
# Parámetros del filtro
# ---------------------------------------------------------------------------
@dataclass
class ESKFParams:
    # --- Ruido IMU (PSD-sqrt, unidades SI) — valores del Phidget calibrado ---
    sigma_a:   float = 9.3170e-04   # VRW accel   [m/s²/√Hz]   (peor eje)
    sigma_g:   float = 7.1914e-05   # ARW gyro    [rad/s/√Hz]  (peor eje; wx)
    sigma_ba:  float = 4.4917e-05   # RRW bias_a  [m/s³/√Hz]
    sigma_bg:  float = 1.1049e-05   # RRW bias_g  [rad/s²/√Hz]
    # --- Ruido de odometría (calibrar empíricamente, Sec. 6.6 doc) ---
    sigma_v_odom:  float = 0.02  # m/s   (velocidad lineal en cuerpo)
    sigma_w_odom:  float = 0.01  # rad/s (velocidad angular en cuerpo)
    # --- Adaptativo estático/dinámico (Bug #13, ver DEBUG_IMU_BASE_FUSION.md) ---
    # sigma_a/sigma_g/sigma_v_odom/sigma_w_odom (arriba) están calibrados para
    # el régimen EN MOVIMIENTO real (vibración de chasis/motores, medida
    # 40-70x mayor que en reposo — necesarios para evitar la cascada de
    # rechazos Mahalanobis que causaba divergencia de posición sin corregir,
    # ver Bug #13). Aplicados también en reposo, dejan sigma_vel estancada muy
    # por encima de sigma_v_odom (validado empíricamente: ~0.056-0.058 m/s en
    # estático puro, sin rechazos, con el robot realmente quieto). Estos
    # _static son el suelo Allan-Variance en reposo real; con
    # enable_adaptive_noise=True el filtro arranca en el suelo estático (recién
    # salido de init_from_static, robot quieto) y solo sube hacia los valores
    # de movimiento cuando la odometría MEDIDA indica movimiento real.
    enable_adaptive_noise: bool = True
    sigma_a_static:      float = 0.012   # m/s²  — std(ax,ay) medido en reposo real
    sigma_g_static:      float = 0.0007  # rad/s — std(wx,wy) medido en reposo real
    sigma_v_odom_static: float = 0.02    # m/s   — valor pre-Bug#13
    sigma_w_odom_static: float = 0.01    # rad/s — idem
    motion_detect_v_thr: float = 0.03    # m/s   — |v_body| medido por encima: régimen dinámico
    motion_detect_w_thr: float = 0.05    # rad/s — |omega_z| medido por encima: idem
    motion_hold_s:       float = 1.5     # s — mantiene el régimen dinámico tras el último movimiento real
    motion_ramp_s:       float = 2.0     # s — rampa lineal de bajada dinámico → estático
    # --- Restricción planar (robot 2D) ---
    use_planar_constraint: bool  = True
    sigma_z_constraint:    float = 1e-3   # m   (varianza para z≈0)
    sigma_rp_constraint:   float = 1e-3   # rad (varianza para roll/pitch≈0)
    # --- Pose absoluta de room_concept (lectura del grafo DSR, intermitente) ---
    # Suelo de seguridad sobre R: room_concept aún no ha convergido su propia
    # covarianza en el primer instante tras crear/reubicar la room, y una R
    # degenerada (~0) haría la corrección demasiado confiada de golpe.
    sigma_room_pos_floor: float = 0.02              # m
    sigma_room_yaw_floor: float = math.radians(1.0) # rad
    # --- Covarianza inicial ---
    sigma_p0:    float = 0.0
    sigma_v0:    float = 0.10
    sigma_th_rp: float = math.radians(2.0)
    # El yaw en t=0 es una referencia arbitraria (el eje X del mundo se define
    # como el rumbo inicial del robot), no una magnitud medida con incertidumbre
    # real como roll/pitch (que sí vienen de la gravedad medida). Dejarlo con
    # incertidumbre alta invita a que update_odom() "explique" el primer
    # transitorio fuerte de aceleración (sesgo del acelerómetro aún sin
    # converger) inyectando parte del residuo en yaw vía el acoplamiento
    # H[0:2,6:9]=skew(v_body_pred) — visto en datos reales: wz≈ruido base
    # (sin rotación real) pero yaw deriva ~7-8° durante el primer arranque
    # tras la inicialización estática. Igual que sigma_p0, se trata como
    # referencia exacta; crecerá de forma natural con el ruido del gyro
    # integrado en predict() según corresponda.
    sigma_th_yaw:float = 0.0
    sigma_ba0:   float = 0.05
    sigma_bg0:   float = math.radians(0.5)
    # --- Control ---
    dt_max:      float = 0.05    # [s] descartar gaps mayores (CPU pausa, etc.)
    static_omega_thr: float = 0.01   # rad/s — umbral para detección de reposo
    static_accel_thr: float = 0.05   # m/s²  — idem


# ---------------------------------------------------------------------------
# Utilidades SO(3) / S³
# ---------------------------------------------------------------------------
def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[ 0.0, -v[2],  v[1]],
                     [ v[2],  0.0, -v[0]],
                     [-v[1],  v[0],  0.0]])

def _qmul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def _qnorm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    q = q / n if n > 1e-12 else np.array([1., 0., 0., 0.])
    return q if q[0] >= 0.0 else -q  # hemisferio canónico

def _exp_so3(phi: np.ndarray) -> np.ndarray:
    """Vector de rotación → cuaternión unitario (mapa exponencial)."""
    angle = float(np.linalg.norm(phi))
    if not math.isfinite(angle) or angle < 1e-9:
        return np.array([1., 0., 0., 0.])
    ax = phi / angle
    s  = math.sin(0.5 * angle)
    return np.array([math.cos(0.5 * angle), ax[0]*s, ax[1]*s, ax[2]*s])

def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])

def _euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll*0.5),  math.sin(roll*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cy, sy = math.cos(yaw*0.5),   math.sin(yaw*0.5)
    return _qnorm(np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]))


# ---------------------------------------------------------------------------
# Filtro
# ---------------------------------------------------------------------------
class ESKF:
    """
    Error-State Kalman Filter.

    Ciclo de uso:
        eskf = ESKF(params)
        eskf.init_from_static(accel_buf, gyro_buf)   # robot quieto ≥ 1 s
        # loop:
        eskf.predict(accel_m, gyro_m, t)             # cada muestra IMU
        eskf.update_odom(v_body_xy, omega_z)         # cada mensaje de odometría
        state = eskf.get_state()
    """

    def __init__(self, params: Optional[ESKFParams] = None):
        self.p = params or ESKFParams()
        self._reset_state()
        self._t_prev:   Optional[float] = None
        self._last_gyro: Optional[np.ndarray] = None
        self.is_initialized: bool = False

        # Ruido adaptativo estático/dinámico (Bug #13): instante (reloj interno
        # self._t_prev) del último movimiento real detectado en update_odom().
        # None = nunca se ha detectado movimiento -> régimen estático.
        self._last_motion_ts: Optional[float] = None

        # Diagnóstico
        self.n_updates:   int = 0
        self.n_rejected:  int = 0
        self.n_consecutive_rejected: int = 0
        self._nis_window: list[float] = []
        # El origen local del ESKF (fijado arbitrariamente en init_from_static,
        # posición (0,0) y yaw=0 = rumbo inicial del robot) no coincide con el
        # frame de room_concept. El primer offset entre ambos es, por
        # construcción, mucho mayor que cualquier innovación normal, así que
        # update_room_pose() lo rechazaría siempre por Mahalanobis. Se resuelve
        # con una alineación única y no gateada (align_to_room_pose) la primera
        # vez que hay una lectura de room válida; después, update_room_pose ya
        # gatea con normalidad porque ambos frames coinciden.
        self.room_aligned: bool = False

    # ------------------------------------------------------------------ init
    def _reset_state(self) -> None:
        pr = self.p
        self.pos  = np.zeros(3)
        self.vel  = np.zeros(3)
        self.quat = np.array([1., 0., 0., 0.])
        self.ba   = np.zeros(3)
        self.bg   = np.zeros(3)
        # Referencia de actitud para la restricción planar (se fija en
        # init_from_static con el tilt real medido por gravedad)
        self.roll_ref  = 0.0
        self.pitch_ref = 0.0
        self.P = np.diag([
            pr.sigma_p0**2,   pr.sigma_p0**2,   pr.sigma_p0**2,
            pr.sigma_v0**2,   pr.sigma_v0**2,   pr.sigma_v0**2,
            pr.sigma_th_rp**2, pr.sigma_th_rp**2, pr.sigma_th_yaw**2,
            pr.sigma_ba0**2,  pr.sigma_ba0**2,  pr.sigma_ba0**2,
            pr.sigma_bg0**2,  pr.sigma_bg0**2,  pr.sigma_bg0**2,
        ])

    def init_from_static(self,
                         accel_buf: np.ndarray,
                         gyro_buf:  np.ndarray) -> bool:
        """
        Inicialización estática: robot quieto ≥ 1 s.
        accel_buf, gyro_buf: (N, 3) en frame del cuerpo, unidades SI.
        Retorna True si la inicialización fue válida.
        """
        if len(accel_buf) < 50:
            log.warning("init_from_static: pocas muestras (%d < 50)", len(accel_buf))
            return False

        omega_rms = float(np.sqrt(np.mean(gyro_buf**2)))
        if omega_rms > self.p.static_omega_thr * 3:
            log.warning("init_from_static: robot en movimiento (|ω|_rms=%.4f)", omega_rms)
            return False

        # Sesgo del giroscopio = media en estático
        self.bg = gyro_buf.mean(axis=0)

        # Roll/pitch desde el vector de gravedad medido
        a_mean = accel_buf.mean(axis=0)
        a_norm = float(np.linalg.norm(a_mean))
        if not np.isfinite(a_norm) or a_norm < 1e-3:
            log.warning("init_from_static: aceleración inválida (norma=%.3e)", a_norm)
            return False
        a_hat  = a_mean / a_norm
        roll   = math.atan2( a_hat[1], a_hat[2])
        pitch  = math.atan2(-a_hat[0], math.sqrt(a_hat[1]**2 + a_hat[2]**2))
        # Yaw = 0 (no referencia absoluta disponible; se irá estimando con la odometría)
        self.quat = _euler_to_quat(roll, pitch, 0.0)
        # La restricción planar debe anclar roll/pitch a ESTOS valores, no a 0:
        # el tilt medido (montaje del IMU / suelo) es real. Forzarlo a 0 hace que
        # la gravedad no se cancele (g·sin(tilt) ≈ 0.12 m/s² con 0.68°) y esa
        # fuga, absorbida en ba por rumbo, es la que hacía divergir la velocidad
        # en cada giro (ba queda apuntando mal al rotar el cuerpo).
        self.roll_ref  = roll
        self.pitch_ref = pitch

        log.info("init_from_static OK: roll=%.2f° pitch=%.2f° bg=[%.2e %.2e %.2e]",
                 math.degrees(roll), math.degrees(pitch), *self.bg)
        self.is_initialized = True
        return True

    # ------------------------------------------------- ruido adaptativo (Bug #13)
    def _motion_blend_alpha(self) -> float:
        """
        0.0 = régimen estático (ruido calibrado con Allan Variance en reposo),
        1.0 = régimen dinámico (ruido medido en movimiento real, ver Bug #13 en
        DEBUG_IMU_BASE_FUSION.md). Sube a 1 al instante en cuanto update_odom()
        detecta movimiento real (_mark_motion) -- prioriza seguridad, evita
        repetir la sobre-confianza que causaba cascadas de rechazo Mahalanobis
        en movimiento. Baja con un hold (motion_hold_s) + rampa lineal
        (motion_ramp_s) tras el último movimiento detectado, para no oscilar
        con cada micro-parada de la odometría.
        """
        if self._last_motion_ts is None or self._t_prev is None:
            return 0.0
        dt_since = self._t_prev - self._last_motion_ts
        if dt_since <= self.p.motion_hold_s:
            return 1.0
        if dt_since >= self.p.motion_hold_s + self.p.motion_ramp_s:
            return 0.0
        return 1.0 - (dt_since - self.p.motion_hold_s) / self.p.motion_ramp_s

    def _mark_motion(self, v_body_xy: np.ndarray, omega_z: float) -> None:
        """Actualiza _last_motion_ts si la odometría MEDIDA indica movimiento
        real (por encima de motion_detect_v_thr/motion_detect_w_thr). Llamado
        desde update_odom() con la medida cruda, antes de tocar sigma_v/w_odom."""
        moving = (float(np.linalg.norm(v_body_xy)) > self.p.motion_detect_v_thr
                  or abs(omega_z) > self.p.motion_detect_w_thr)
        if moving and self._t_prev is not None:
            self._last_motion_ts = self._t_prev

    # --------------------------------------------------------------- predict
    def predict(self, accel_m: np.ndarray, gyro_m: np.ndarray, t: float) -> None:
        """
        Propaga estado nominal + covarianza con una muestra IMU.
        t: timestamp en segundos (monotónico; preferir timestamp HW del Phidget).
        """
        if (not np.all(np.isfinite(accel_m)) or
                not np.all(np.isfinite(gyro_m)) or
                not math.isfinite(t)):
            return
        self._last_gyro = gyro_m.copy()

        if self._t_prev is None:
            self._t_prev = t
            return
        dt = t - self._t_prev
        self._t_prev = t

        if dt <= 0.0 or dt > self.p.dt_max:
            return

        R  = _quat_to_rot(self.quat)
        ah = accel_m - self.ba       # accel corregida de sesgo
        wh = gyro_m  - self.bg       # gyro  corregida de sesgo
        a_world = R @ ah + G_WORLD

        # ---- Estado nominal ----
        self.pos  = self.pos + self.vel * dt + 0.5 * a_world * dt * dt
        self.vel  = self.vel + a_world * dt
        self.quat = _qnorm(_qmul(self.quat, _exp_so3(wh * dt)))
        # ba, bg: random walk → no integrar en nominal (sólo ruido de proceso)

        # ---- Jacobiano F (15×15) — Solà 2017 eq. 268-271 ----
        F = np.eye(15)
        F[0:3,  3:6 ] =  np.eye(3) * dt
        F[3:6,  6:9 ] = -R @ _skew(ah) * dt
        F[3:6,  9:12] = -R * dt
        F[6:9,  6:9 ] =  _quat_to_rot(_exp_so3(-wh * dt))  # exacto SO(3)
        F[6:9, 12:15] = -np.eye(3) * dt

        # ---- Ruido de proceso Q (discreto, convenio PSD: varianza = σ² · Δt) ----
        pa = self.p
        if pa.enable_adaptive_noise:
            alpha = self._motion_blend_alpha()
            sigma_a_eff = pa.sigma_a_static + alpha * (pa.sigma_a - pa.sigma_a_static)
            sigma_g_eff = pa.sigma_g_static + alpha * (pa.sigma_g - pa.sigma_g_static)
        else:
            sigma_a_eff, sigma_g_eff = pa.sigma_a, pa.sigma_g
        Q = np.zeros((15, 15))
        Q[3:6,   3:6 ] = sigma_a_eff**2 * dt * np.eye(3)
        Q[6:9,   6:9 ] = sigma_g_eff**2 * dt * np.eye(3)
        Q[9:12,  9:12] = pa.sigma_ba**2 * dt * np.eye(3)
        Q[12:15,12:15] = pa.sigma_bg**2 * dt * np.eye(3)

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)  # simetría numérica

        # ---- Restricción planar opcional (z≈0, roll≈0, pitch≈0) ----
        if pa.use_planar_constraint:
            self._apply_planar_constraint()

    # ----------------------------------------------------- restricción planar
    _diag_counter: int = 0   # throttle diagnóstico P

    def _apply_planar_constraint(self) -> None:
        """
        Pseudo-medidas: z≈0, vz≈0, roll≈roll_ref, pitch≈pitch_ref.
        vz≈0 es imprescindible: acota P[5,5] que de otro modo crece sin medir
        y propaga vía F[2,5]=dt hasta desbordar la ganancia del constraint de z.
        roll/pitch se anclan al tilt REAL medido en init_from_static, no a 0:
        anclarlos a 0 pelea contra la actitud verdadera y fuga g·sin(tilt) al
        plano horizontal (divergencia de velocidad en giros vía ba mal aprendido).
        Para giro puro alrededor de la gravedad (descomposición ZYX), roll/pitch
        de un tilt de montaje se mantienen constantes, así que la referencia
        fija es exacta en suelo nivelado.
        """
        w, x, y, z = self.quat
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))

        # z posición — sesgos excluidos: la ruta pos→sesgo es inestable
        Rz = np.zeros((1, 15)); Rz[0, 2] = 1.0
        self._scalar_update(0.0, self.pos[2], self.p.sigma_z_constraint**2, Rz,
                            update_biases=False)
        # z velocidad (robot terrestre: vz ≈ 0 en frame mundo)
        Rv = np.zeros((1, 15)); Rv[0, 5] = 1.0
        self._scalar_update(0.0, self.vel[2], self.p.sigma_z_constraint**2, Rv,
                            update_biases=False)
        # roll
        Hr = np.zeros((1, 15)); Hr[0, 6] = 1.0
        self._scalar_update(self.roll_ref, roll, self.p.sigma_rp_constraint**2, Hr,
                            update_biases=False)
        # pitch
        Hp = np.zeros((1, 15)); Hp[0, 7] = 1.0
        self._scalar_update(self.pitch_ref, pitch, self.p.sigma_rp_constraint**2, Hp,
                            update_biases=False)

        # diagnóstico de salud de P cada 500 predict steps
        ESKF._diag_counter += 1
        if ESKF._diag_counter % 500 == 0:
            d = np.diag(self.P)
            neg = np.where(d < 0)[0]
            log.info("P_diag: pos=[%.2e %.2e %.2e] vel=[%.2e %.2e %.2e] "
                     "att=[%.2e %.2e %.2e] ba=[%.2e %.2e %.2e] bg=[%.2e %.2e %.2e]%s",
                     *d,
                     f" *** NEGATIVO idx={neg.tolist()} ***" if len(neg) else "")

    def _scalar_update(self, z_obs: float, z_pred: float,
                       r_var: float, H: np.ndarray,
                       update_biases: bool = True) -> None:
        """Update escalar genérico (H: 1×15).
        update_biases=False: zeroes k[9:15] para evitar corrupción de sesgos
        por pseudo-medidas de posición/actitud (ruta de observabilidad inestable).
        """
        y  = z_obs - z_pred
        S  = float((H @ self.P @ H.T).item()) + r_var   # escalar
        if not math.isfinite(S) or S < r_var * 0.5:
            log.debug("_scalar_update: S inválida/pequeña (%.2e); P_diag=%s", S,
                      np.array2string(np.diag(self.P), precision=2, suppress_small=True))
            return
        k  = (self.P @ H.T).flatten() / S               # vector 15
        if not update_biases:
            k[9:15] = 0.0
        dx = k * y
        if not np.all(np.isfinite(dx)):
            idx = int(np.argmax(~np.isfinite(dx)))
            log.warning("_scalar_update: desborde en dx[%d] "
                        "(S=%.2e k_max=%.2e y=%.2e P[idx,idx]=%.2e); descartando",
                        idx, S, float(np.nanmax(np.abs(k))), y,
                        float(self.P[idx, idx]))
            return
        self._inject(dx)
        I_KH = np.eye(15) - np.outer(k, H.flatten())
        self.P = I_KH @ self.P @ I_KH.T + np.outer(k, k) * r_var
        self.P = 0.5 * (self.P + self.P.T)

    # --------------------------------------------------------- update odometría
    def update_odom(self,
                    v_body_xy: np.ndarray,
                    omega_z:   float,
                    t_odom:    Optional[float] = None) -> dict:
        """
        Update con velocidades derivadas de la odometría de ruedas.
            v_body_xy : [vx, vy] en frame cuerpo [m/s]
            omega_z   : yaw rate en frame cuerpo [rad/s]
            t_odom    : timestamp del mensaje (para logging; no altera el filtro)

        Retorna un dict de diagnóstico (para depuración imu+base):
            accepted     : bool — si la corrección se aplicó
            v_body_pred  : [vx, vy] predichos por el estado nominal (None si no evaluado)
            omega_pred   : ω_z predicho (gyro - bg)
            innovation   : z - h (None si no evaluado)
            mahalanobis  : distancia² de Mahalanobis (None si no evaluado)
        """
        diag = {"accepted": False, "v_body_pred": None, "omega_pred": None,
                "innovation": None, "mahalanobis": None}

        if not self.is_initialized or self._last_gyro is None:
            return diag

        # Sanity check de tamaño de medida
        if np.linalg.norm(v_body_xy) > 3.0 or abs(omega_z) > 6.0:
            log.warning("update_odom: medida fuera de rango (v=%.2f w=%.2f), rechazando",
                        np.linalg.norm(v_body_xy), omega_z)
            return diag

        if self.p.enable_adaptive_noise:
            self._mark_motion(v_body_xy, omega_z)

        R_wb = _quat_to_rot(self.quat)
        R_bw = R_wb.T

        # Predicción de la medida desde el estado nominal
        v_body_pred  = R_bw @ self.vel
        omega_z_pred = float((self._last_gyro - self.bg)[2])

        z    = np.array([v_body_xy[0], v_body_xy[1], omega_z])
        h    = np.array([v_body_pred[0], v_body_pred[1], omega_z_pred])
        inov = z - h  # innovación

        diag["v_body_pred"] = v_body_pred[:2].copy()
        diag["omega_pred"]  = omega_z_pred
        diag["innovation"]  = inov.copy()

        # Jacobiano H (3×15)
        H = np.zeros((3, 15))
        H[0:2,  3:6 ] = R_bw[0:2, :]            # ∂v_body/∂δv
        H[0:2,  6:9 ] = _skew(v_body_pred)[0:2, :]  # ∂v_body/∂δθ
        H[2,   12:15] = np.array([0., 0., -1.])  # ∂ω_z/∂δb_g_z

        if self.p.enable_adaptive_noise:
            alpha = self._motion_blend_alpha()
            sigma_v_eff = self.p.sigma_v_odom_static + alpha * (self.p.sigma_v_odom - self.p.sigma_v_odom_static)
            sigma_w_eff = self.p.sigma_w_odom_static + alpha * (self.p.sigma_w_odom - self.p.sigma_w_odom_static)
        else:
            sigma_v_eff, sigma_w_eff = self.p.sigma_v_odom, self.p.sigma_w_odom
        R_cov = np.diag([sigma_v_eff**2, sigma_v_eff**2, sigma_w_eff**2])

        S = H @ self.P @ H.T + R_cov

        # Gating de Mahalanobis
        try:
            m2 = float(inov @ np.linalg.solve(S, inov))
        except np.linalg.LinAlgError:
            return diag
        diag["mahalanobis"] = m2

        if m2 > CHI2_3_99:
            self.n_rejected += 1
            self.n_consecutive_rejected += 1
            log.debug("update_odom: rechazado Mahalanobis=%.1f (thr=%.1f)", m2, CHI2_3_99)
            return diag

        # Ganancia de Kalman y corrección (forma de Joseph para estabilidad)
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ inov
        self._inject(dx)

        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_cov @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        # Estadísticas
        self.n_updates += 1
        self.n_consecutive_rejected = 0
        self._nis_window.append(m2)
        if len(self._nis_window) > 100:
            self._nis_window.pop(0)

        diag["accepted"] = True
        return diag

    def align_to_room_pose(self,
                           pos_xy:  np.ndarray,
                           yaw:     float,
                           cov_xy:  np.ndarray,
                           var_yaw: float) -> None:
        """
        Alineación única, no gateada, entre el frame local del ESKF y el de
        room_concept — ver comentario en __init__ sobre por qué hace falta.
        Se llama una sola vez (comprueba/marca self.room_aligned); llamadas
        posteriores deben ir por update_room_pose(), que sí gatea.
        """
        self.pos[0] = pos_xy[0]
        self.pos[1] = pos_xy[1]
        w, x, y, z = self.quat
        roll    = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch   = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        yaw_old = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        dyaw    = yaw - yaw_old

        self.quat = _euler_to_quat(roll, pitch, yaw)

        # self.vel es un vector en frame mundo: hay que rotarlo el mismo dyaw
        # que la actitud, o quedaría expresado con el offset de yaw viejo y
        # el siguiente predict() integraría el movimiento en la dirección
        # equivocada respecto al frame de la room recién adoptado.
        c, s = math.cos(dyaw), math.sin(dyaw)
        vx, vy = float(self.vel[0]), float(self.vel[1])
        self.vel[0] = c*vx - s*vy
        self.vel[1] = s*vx + c*vy

        self.P[0, 0] = max(float(cov_xy[0, 0]), self.p.sigma_room_pos_floor**2)
        self.P[1, 1] = max(float(cov_xy[1, 1]), self.p.sigma_room_pos_floor**2)
        self.P[0, 1] = self.P[1, 0] = float(cov_xy[0, 1])
        self.P[8, 8] = max(float(var_yaw), self.p.sigma_room_yaw_floor**2)

        self.room_aligned = True
        log.info("ESKF alineado a pose de room_concept: pos=(%.3f, %.3f) yaw=%.3f rad",
                 pos_xy[0], pos_xy[1], yaw)

    def update_room_pose(self,
                         pos_xy:  np.ndarray,
                         cov_xy:  np.ndarray,
                         yaw:     Optional[float] = None,
                         var_yaw: Optional[float] = None) -> dict:
        """
        Update con la pose absoluta leída del RT de room_concept en el grafo
        DSR (ver dsr_room_pose.py). No sustituye a una medida GPS real: es
        una referencia de localización indoor intermitente (solo disponible
        cuando room_concept tiene una estimación fresca y válida), usada
        para acotar la deriva de posición del dead-reckoning odom+IMU -- la
        única corrección de este ESKF que toca directamente pos[0:2] (sin
        ella, sigma_pos crece sin límite: ver discusión de la sesión).

        yaw/var_yaw son opcionales (None = solo posición, 2 g.d.l., gate
        CHI2_2_99). Si se pasan, se añade una tercera fila de yaw (3 g.d.l.,
        gate CHI2_3_99) -- probar con cautela: con datos reales se vio que
        el yaw de room_concept coincide con el nuestro (odom+IMU) casi
        perfectamente durante largos tramos (~110s, error <1°), pero puede
        "engancharse" a un valor erróneo tras perder el tracking en un giro
        real (saltos de 90-170° entre lecturas consecutivas, coherente con
        la propia detección de "FLIP" que tiene su código) y quedarse así de
        forma persistente -- si eso vuelve a pasar, la corrección de yaw
        arrastraría la fusión en el sentido equivocado de ahí en adelante.

            pos_xy  : [x, y] en el frame de la room/mundo [m]
            cov_xy  : covarianza 2x2 de la posición [m²]
            yaw     : orientación en el mismo frame [rad], o None
            var_yaw : varianza del yaw [rad²], requerido si yaw no es None

        Retorna un dict de diagnóstico (mismo esquema que update_odom).
        """
        diag = {"accepted": False, "pos_pred": None, "yaw_pred": None,
                "innovation": None, "mahalanobis": None}

        if not self.is_initialized:
            return diag

        use_yaw = yaw is not None

        pos_pred = self.pos[:2].copy()
        w, x, y, z = self.quat
        yaw_pred = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        diag["pos_pred"] = pos_pred
        diag["yaw_pred"] = yaw_pred

        if use_yaw:
            dyaw = yaw - yaw_pred
            dyaw = (dyaw + math.pi) % (2*math.pi) - math.pi
            inov = np.array([pos_xy[0] - pos_pred[0], pos_xy[1] - pos_pred[1], dyaw])
            H = np.zeros((3, 15))
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 8] = 1.0
            R_cov = np.zeros((3, 3))
            R_cov[0:2, 0:2] = cov_xy
            R_cov[2, 2] = var_yaw
            R_cov[2, 2] = max(R_cov[2, 2], self.p.sigma_room_yaw_floor**2)
            chi2_thr = CHI2_3_99
        else:
            inov = np.array([pos_xy[0] - pos_pred[0], pos_xy[1] - pos_pred[1]])
            H = np.zeros((2, 15))
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            R_cov = np.array(cov_xy, dtype=float).copy()
            chi2_thr = CHI2_2_99

        diag["innovation"] = inov.copy()
        R_cov[0, 0] = max(R_cov[0, 0], self.p.sigma_room_pos_floor**2)
        R_cov[1, 1] = max(R_cov[1, 1], self.p.sigma_room_pos_floor**2)

        S = H @ self.P @ H.T + R_cov

        try:
            m2 = float(inov @ np.linalg.solve(S, inov))
        except np.linalg.LinAlgError:
            return diag
        diag["mahalanobis"] = m2

        if m2 > chi2_thr:
            self.n_rejected += 1
            self.n_consecutive_rejected += 1
            log.debug("update_room_pose: rechazado Mahalanobis=%.1f (thr=%.1f)", m2, chi2_thr)
            return diag

        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ inov
        self._inject(dx)

        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_cov @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self.n_updates += 1
        self.n_consecutive_rejected = 0
        self._nis_window.append(m2)
        if len(self._nis_window) > 100:
            self._nis_window.pop(0)

        diag["accepted"] = True
        return diag

    # --------------------------------------------------- inyección error→nominal
    def _inject(self, dx: np.ndarray) -> None:
        """Aplica el vector de error al estado nominal y resetea el error a 0."""
        if not np.all(np.isfinite(dx)):
            log.warning("_inject: dx inválido; se descarta la corrección")
            return
        self.pos  += dx[0:3]
        self.vel  += dx[3:6]
        self.quat  = _qnorm(_qmul(self.quat, _exp_so3(dx[6:9])))
        self.ba   += dx[9:12]
        self.bg   += dx[12:15]

    def get_yaw_rate(self) -> float:
        """
        ω_z estimado (gyro corregido de sesgo). Usado para propagar hacia el
        presente medidas retrasadas (p.ej. la pose de room_concept, calculada
        sobre un scan de lidar del pasado) antes de fusionarlas — ver
        specificworker._consume_room_pose().
        """
        if self._last_gyro is None:
            return 0.0
        return float(self._last_gyro[2] - self.bg[2])

    # ------------------------------------------------------------------ output
    def get_state(self) -> dict:
        """
        Snapshot del estado con incertidumbres y métricas de calidad.
        Todas las magnitudes en SI.
        """
        sig = np.sqrt(np.maximum(np.diag(self.P), 0.0))
        w, x, y, z = self.quat
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        nis_mean = float(np.mean(self._nis_window)) if self._nis_window else 0.0
        return {
            # Estado
            "pos":   self.pos.copy(),
            "vel":   self.vel.copy(),
            "quat":  self.quat.copy(),
            "roll":  roll,  "pitch": pitch,  "yaw": yaw,
            "ba":    self.ba.copy(),
            "bg":    self.bg.copy(),
            # Covarianza completa
            "P":     self.P.copy(),
            # Incertidumbres marginales
            "sigma_pos":   sig[0:3],
            "sigma_vel":   sig[3:6],
            "sigma_theta": sig[6:9],   # [roll, pitch, yaw] rad
            "sigma_ba":    sig[9:12],
            "sigma_bg":    sig[12:15],
            # Métricas
            "nis_last":    self._nis_window[-1] if self._nis_window else 0.0,
            "nis_mean":    nis_mean,
            "n_updates":   self.n_updates,
            "n_rejected":  self.n_rejected,
            "n_consec_rej": self.n_consecutive_rejected,
            # 0=régimen estático, 1=dinámico (Bug #13, ver _motion_blend_alpha)
            "motion_alpha": self._motion_blend_alpha() if self.p.enable_adaptive_noise else 1.0,
        }

    def get_differential(self, prev_pos: np.ndarray,
                         prev_yaw: float) -> dict:
        """
        Movimiento diferencial desde (prev_pos, prev_yaw) hasta el estado actual,
        expresado en el frame de cuerpo previo.
        Útil para alimentar el grafo DSR con deltas de pose.
        """
        dp_world = self.pos - prev_pos
        cyaw, syaw = math.cos(prev_yaw), math.sin(prev_yaw)
        dx_body =  cyaw * dp_world[0] + syaw * dp_world[1]
        dy_body = -syaw * dp_world[0] + cyaw * dp_world[1]

        w, x, y, z = self.quat
        curr_yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        dyaw = curr_yaw - prev_yaw
        # Normalizar a [-π, π]
        dyaw = (dyaw + math.pi) % (2*math.pi) - math.pi

        # Velocidades en cuerpo actual
        R_bw = _quat_to_rot(self.quat).T
        v_body = R_bw @ self.vel
        # omega_z = velocidad angular de yaw (gyro corregido de sesgo), NO v_body[2]
        # (que es la velocidad lineal vertical del cuerpo, ~0 por la restricción planar).
        omega_z = float(self._last_gyro[2] - self.bg[2]) if self._last_gyro is not None else 0.0

        sig = np.sqrt(np.maximum(np.diag(self.P), 0.0))
        return {
            "dx":      dx_body,    # m
            "dy":      dy_body,    # m
            "dyaw":    dyaw,       # rad
            "vx":      v_body[0],  # m/s
            "vy":      v_body[1],  # m/s
            "omega_z": omega_z,    # rad/s
            "sigma_dx":   float(sig[0]),
            "sigma_dy":   float(sig[1]),
            "sigma_dyaw": float(sig[8]),  # σ(yaw)
        }

    @property
    def yaw(self) -> float:
        w, x, y, z = self.quat
        return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))