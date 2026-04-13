# Cambios respecto al repositorio — sdf_localizer

Documento de referencia para los cambios introducidos en esta sesión de desarrollo.
Cada sección describe el problema detectado, la solución aplicada y los ficheros afectados.

---

## 1. Corrección de signo en odometría medida (`odom.adv`)

**Problema**  
`FullPoseEstimationPub_newFullPose` almacenaba `pose.adv` tal cual en `odom.adv`.
El signo de `adv` en el mensaje FPE es el opuesto al convenio del localizador: cuando el
robot avanza (`vel_adv_y > 0`), `odom.adv` era negativo.  
Consecuencia: en el 100 % de los fotogramas con movimiento hacia adelante, el prior de
odometría medida (`meas_dy < 0`) se oponía al prior de velocidad comandada (`cmd_dy > 0`),
cancelándose mutuamente y haciendo que la predicción de movimiento fuera prácticamente nula
("resistencia al avance").

**Solución**  
Se niega `pose.adv` al almacenarlo, análogamente a como ya se había corregido `odom.rot`.

```diff
- odom.adv  = add_noise(pose.adv);    // forward velocity, m/s
+ odom.adv  = add_noise(-pose.adv);   // FPE adv is sign-inverted vs localizer convention; negate
```

**Fichero:** `src/specificworker.cpp` (función `FullPoseEstimationPub_newFullPose`)

---

## 2. Corrección del bug `meas_valid = 0`

**Problema**  
`compute_odometry_prior()` tenía como efecto lateral actualizar `last_lidar_timestamp` antes
de que `compute_measured_odometry_prior()` se ejecutara.  
Como resultado, dentro de `compute_measured_odometry_prior`, el cálculo
`dt = lidar_timestamp - last_lidar_timestamp` daba siempre 0 → `meas_valid = false` en el
100 % de los fotogramas. El prior de odometría medida nunca se aplicaba.

**Solución**  
Se usa `last_update_result.timestamp_ms` como ancla temporal del frame anterior en lugar de
`last_lidar_timestamp` (que ya fue sobreescrita).

```diff
- if (last_lidar_timestamp == 0 || odometry_history.empty() || !last_update_result.ok)
-     return prior;
- const auto dt = lidar_timestamp - last_lidar_timestamp;
- prior.delta_pose = integrate_odometry_over_window(..., last_lidar_timestamp, lidar_timestamp);

+ const int64_t prev_ts = last_update_result.timestamp_ms;
+ if (prev_ts == 0 || odometry_history.empty() || !last_update_result.ok)
+     return prior;
+ const auto dt = lidar_timestamp - prev_ts;
+ prior.delta_pose = integrate_odometry_over_window(..., prev_ts, lidar_timestamp);
```

**Fichero:** `src/room_concept.cpp` (función `compute_measured_odometry_prior`)

---

## 3. Escalado del learning rate de Adam con el tamaño de ventana

**Problema**  
El learning rate de Adam (`lr = 0.05`) estaba calibrado para optimizar 3 parámetros con
una ventana de 1 slot.  
Con `WindowSize = 50`, la magnitud total del gradiente escala con W, y el primer paso de
Adam sobreajustaba en un factor ~W, causando explosiones en la curva de pérdida
(`L[0]→L[1]` subía 5–10×).

**Solución**  
Se escala `lr ∝ 1/√W` para mantener el tamaño de paso efectivo constante
independientemente del número de slots.

```cpp
const int ws = static_cast<int>(window_mgr_.size());
const float ws_scale = 1.0f / std::sqrt(static_cast<float>(std::max(1, ws)));
const float lr = params.learning_rate_pos * ws_scale;
// ws=1  → lr=0.050
// ws=10 → lr=0.016
// ws=50 → lr=0.007
```

El parámetro `convergence_min_iters` se incrementó de 5 a 8, porque con lr reducido Adam
necesita más pasos para acumular momento antes de que el criterio de convergencia sea fiable.

**Ficheros:** `src/room_concept.cpp` (función `run_adam_loop`), `src/room_concept.h`

---

## 4. Visualización sincronizada (scan + pose del mismo frame)

**Problema**  
El visor 2D dibujaba siempre el scan LIDAR del frame *actual* (`lidar_data_->first`)
pero la pose venía del resultado de localización *anterior* (hay latencia entre el
cómputo de localización y el tick de `compute()`).  
Esto causaba offsets visuales perceptibles en rotación: los puntos del scan no coincidían
con la pose del robot.

**Solución**  
Se cachea el scan LIDAR usado en cada llamada a `update()` dentro de `UpdateResult`
(campo `lidar_scan`).  
Cuando hay localización válida, se dibujan:
- **Scan** con `loc_res->lidar_scan` (el scan que produjo ese resultado)
- **Pose del scan** con `loc_res->robot_pose` (exacta, sin proyección)
- **Marcador del robot** con `display_pose` (proyectado hacia adelante para suavidad)

```cpp
// specificworker.cpp — compute()
const bool use_loc_scan = have_loc && !loc_res->lidar_scan.empty();
const std::vector<Eigen::Vector3f>& draw_points =
    use_loc_scan ? loc_res->lidar_scan : lidar_data_->first;
const Eigen::Affine2f loc_pose = have_loc ? loc_res->robot_pose : display_pose;

viewer_2d_->update_frame({
    .lidar_points = draw_points,
    .display_pose = display_pose,   // marcador robot (suavizado)
    .loc_pose     = loc_pose,       // pose para dibujar el scan
    .use_loc_pose = use_loc_scan,
    ...
});

// Esquinas también con la pose exacta, no la proyectada
if (have_loc && !loc_res->corner_matches.empty())
    viewer_2d_->draw_corners(loc_res->corner_matches, loc_res->robot_pose);
```

**Ficheros:** `src/room_concept.h` (campo `lidar_scan` en `UpdateResult`),
`src/room_concept.cpp` (asignación en los 3 paths de retorno de `update()`),
`src/specificworker.cpp`, `src/viewer_2d.h` (campo `loc_pose` y `use_loc_pose` en `FrameData`),
`src/viewer_2d.cpp` (uso de `loc_pose` al llamar a `draw_lidar_points`)

---

## 5. Timestamps precisos para odometría y velocidad

**Problema**  
`OdometryReading` y `VelocityCommand` sólo tenían un `std::chrono::time_point` heredado.
`integrate_odometry_over_window` e `integrate_velocity_over_window` hacían aritmética
de tiempo con `chrono`, lo que dependía de que los timestamps de recepción local
coincidieran con los del sensor — no siempre es cierto.

**Solución**  
Se añaden campos `source_ts_ms` (timestamp del evento en el sensor) y `recv_ts_ms`
(timestamp de recepción local), ambos en epoch-milliseconds.  
El método `effective_ts_ms()` devuelve `source_ts_ms` si es > 0, si no `recv_ts_ms`.  
La integración temporal se hace ahora con enteros de 64 bits (ms), eliminando la
dependencia de `chrono` y los posibles errores de redondeo.

```cpp
// common_types.h
struct OdometryReading {
    ...
    std::int64_t source_ts_ms = 0;
    std::int64_t recv_ts_ms   = 0;
    std::chrono::time_point<...> timestamp;  // mantenido por compatibilidad
    std::int64_t effective_ts_ms() const { return source_ts_ms > 0 ? source_ts_ms : recv_ts_ms; }
};
```

Todos los sitios que generan readings (FPE, joystick, navegador) ahora rellenan ambos
campos.

**Ficheros:** `src/common_types.h`, `src/specificworker.cpp`

---

## 6. Integración con heading de punto medio (midpoint rule)

**Problema**  
`integrate_velocity_over_window` e `integrate_odometry_over_window` transformaban el
desplazamiento local al frame global usando el ángulo al inicio del segmento.
Cuando hay rotación significativa en el segmento, esto introduce un sesgo proporcional
a `dθ × v`.

**Solución**  
Se usa el ángulo a mitad del segmento (`θ_mid = θ + 0.5·dθ`) para la transformación.
Esto es equivalente a una integración trapezoidal de primer orden y reduce el sesgo
sin necesitar información de aceleración.

```cpp
const float theta_mid = running_theta + 0.5f * dtheta;
total_delta[0] += dx_local * std::cos(theta_mid) - dy_local * std::sin(theta_mid);
total_delta[1] += dx_local * std::sin(theta_mid) + dy_local * std::cos(theta_mid);
```

También se corrigió la jacobiana de la matriz de transición `F` en `predict_step`,
que tenía swapeados los términos `dx_local`/`dy_local`.

**Fichero:** `src/room_concept.cpp`

---

## 7. Log de diagnóstico en `tmp/` del componente

**Problema**  
El log CSV se escribía en `/tmp/sdf_localizer/log.csv` (directorio global del sistema).

**Solución**  
Se escribe en `tmp/sdf_localizer/log.csv` (relativo al directorio de trabajo del
componente). Los directorios se crean automáticamente con `mkdir("tmp", 0755)` y
`mkdir("tmp/sdf_localizer", 0755)`.

**Fichero:** `src/room_concept.cpp` (función `init_debug_log`)

---

## 8. Log CSV con columna `lr_eff` y diagnóstico de Adam

**Añadido**  
- Columna `lr_eff` en el CSV: learning rate efectivo usado en cada frame
  (`lr_base / sqrt(window_size)`).
- Columna `adam_losses`: pérdidas por iteración separadas por `|` (e.g. `1.2|0.9|0.7`).
  Permite ver en el CSV la curva de convergencia completa de Adam.
- El path de *early exit* también escribe una fila en el CSV (con `early_exit=1, iters=0`).

**Fichero:** `src/room_concept.cpp`, `src/room_concept.h`

---

## 9. Traza de rotación integrada (`rotation_trace`)

**Añadido**  
Log periódico (cada 250 ms) que imprime la rotación acumulada total desde el inicio del
experimento, útil para verificar que la integración angular es coherente con la realidad.
Controlado por `params.rotation_trace_enabled`.

**Fichero:** `src/room_concept.cpp`, `src/room_concept.h`

---

## 10. Corner tracking desactivable

**Añadido**  
Nuevo parámetro `enable_corner_tracking = false` (desactivado por defecto).
Cuando es `false`, las detecciones de esquinas no se añaden al factor de pérdida de Adam
ni al loop de optimización, eliminando una posible fuente de ruido cuando la geometría
de la sala no tiene esquinas claras o el modelo de esquinas no está ajustado.

**Fichero:** `src/room_concept.h`, `src/room_concept.cpp`

---

## 11. Forward projection: EMA sobre velocidades, no sobre deltas

**Problema**  
La EMA de suavizado en `forward_project_pose` se aplicaba sobre los deltas de posición
(`smooth_dx_`, `smooth_dy_`). Esto filtraba dos veces el lag: una al calcular el delta
y otra al suavizarlo, causando inconsistencias cuando la latencia cambiaba.

**Solución**  
La EMA se aplica ahora sobre las velocidades en frame robot (`smooth_v_adv_`,
`smooth_v_side_`, `smooth_v_rot_`). El delta se recalcula en cada tick multiplicando
la velocidad suavizada por el lag actual, lo que es físicamente correcto.

**Fichero:** `src/specificworker.cpp`, `src/specificworker.h`

---

## 12. UI: FE diferenciada Adam vs Predicción

**Añadido**  
- El label de estado muestra `[Adam]` o `[Pred]` según si el frame usó optimización Adam
  o salió por la vía de early-exit.
- El gráfico de Free Energy tiene ahora dos series: `fe_adam` (azul) y `fe_pred`
  (naranja), permitiendo distinguir visualmente la calidad de los dos caminos.

**Fichero:** `src/specificworker.cpp`

---

## 13. `config.toml`: `NumIterations` 10 → 20

```diff
- NumIterations  = 10   # Adam steps per update (25 = accurate, 5 = fast)
+ NumIterations  = 20   # Adam steps per update. With lr scaled by 1/sqrt(ws), more iters needed vs fixed lr.
```

Con el lr escalado por `1/√W`, Adam converge más despacio en las primeras iteraciones.
Doblar el número de pasos recupera la calidad de convergencia previa.

**Fichero:** `etc/config.toml`

---

## Resumen de ficheros modificados

| Fichero | Cambios |
|---|---|
| `src/common_types.h` | Añade `source_ts_ms`, `recv_ts_ms`, `effective_ts_ms()` a `OdometryReading` y `VelocityCommand` |
| `src/room_concept.h` | `lidar_scan` en `UpdateResult`; params nuevos (`enable_corner_tracking`, `rotation_trace_*`); `convergence_min_iters` 5→8; estado interno debug log; declaraciones de métodos nuevos |
| `src/room_concept.cpp` | Fix `meas_valid`; Adam lr scaling; midpoint rule; debug log CSV; traza de rotación; `lidar_scan` en 3 paths; corner tracking condicional; log path relativo |
| `src/specificworker.cpp` | Fix signo `odom.adv`; EMA sobre velocidades; visualización sincronizada; timestamps en FPE/joystick/navegador; UI Adam vs Pred |
| `src/specificworker.h` | EMA state renombrado a velocidades |
| `src/viewer_2d.h` | `FrameData` añade `loc_pose`, `use_loc_pose` |
| `src/viewer_2d.cpp` | `update_frame` usa `loc_pose` para dibujar el scan |
| `etc/config.toml` | `NumIterations` 10→20 |
