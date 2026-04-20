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

## 14. Columnas de desglose FE y timing en el log CSV

**Añadido**  
Se añaden 9 columnas nuevas al log CSV para diagnóstico de Adam y rendimiento:

- `loss_boundary`, `loss_obs`, `loss_motion`, `loss_corner` — términos individuales de la
  función de energía libre (sólo en frames Adam; `nan` en early-exit).
- `loss_init` — pérdida antes del primer paso de Adam, permite medir caída de convergencia.
- `t_update_ms`, `t_adam_ms`, `t_cov_ms`, `t_breakdown_ms` — tiempos de las fases
  principales del ciclo `update()`, en milisegundos.

El nuevo método `WindowManager::compute_rfe_loss_breakdown()` calcula el desglose sin
gradientes (`.detach()`) para no interferir con la optimización.  
Los miembros `t_update_start_`, `last_t_adam_ms_`, `last_t_cov_ms_`, `last_t_breakdown_ms_`
se añaden como campos de `RoomConcept` para compartir los instantes entre `update()` y
`try_prediction_early_exit()`.

**Ficheros:** `src/room_concept.h`, `src/room_concept.cpp`

---

## 15. Telemetría en tiempo real con Rerun (`RerunLogger`)

**Añadido**  
Nueva clase `RerunLogger` que exporta todos los datos de cada ciclo de localización a
[Rerun](https://rerun.io/) en tiempo real mediante un socket TCP local.

### Arquitectura

```
sdf_localizer (C++)                    Python bridge              Rerun viewer
┌───────────────────┐  TCP 9877        ┌──────────────┐           ┌────────────┐
│ RerunLogger       │──[4B len][JSON]──▶│rerun_bridge  │──rr.log──▶│  Rerun UI  │
│ background thread │                  │ .py          │           └────────────┘
└───────────────────┘                  └──────────────┘
```

`RerunLogger` serializa cada frame a JSON (con puntos lidar y SDF grid en base64 float32 LE)
y lo encola en un `std::deque` con ring buffer (máx. 30 frames).  Un hilo sender en
background gestiona la conexión TCP y el reenvío, sin bloquear el hilo de localización.

### Entidades registradas en Rerun

| Path Rerun | Tipo | Contenido |
|---|---|---|
| `world/robot` | Transform3D | Pose del robot (traslación + rotación) |
| `world/robot/body` | Boxes3D | Cuerpo del robot (25×20×15 cm) |
| `world/robot/fwd` | Arrows3D | Vector de avance |
| `world/path` | LineStrips3D | Trayectoria acumulada (máx. 5000 poses) |
| `world/lidar` | Points3D | Nube de puntos en frame mundo (azul=Adam, gris=Pred) |
| `world/sdf` + `sdf_2d/grid` | Transform3D + Image | Grid SDF cada N frames |
| `metrics/timing/*` | Scalar | `t_update`, `t_adam`, `t_cov`, `t_breakdown` (ms) |
| `metrics/loss/*` | Scalar | `loss_init`, `final_loss`, `l_bnd`, `l_obs`, `l_mot`, `l_cor` |
| `metrics/quality/*` | Scalar | `sdf_mse`, `innov_norm`, `cov_xx`, `cov_tt`, `cond_num` |
| `metrics/pipeline/*` | Scalar | `window_size`, `iters`, `early_exit` |

### Layout del visor

Split horizontal: vista 3D (izquierda) + 4 paneles de series temporales (derecha):
Timing · Loss · Quality · Pipeline.

### Uso

```bash
# Terminal 1 — arrancar el bridge Python
python3 scripts/rerun_bridge.py          # lanza también el visor Rerun

# Terminal 2 — arrancar el componente (logs automáticamente si Rerun.Enabled=true)
./bin/sdf_localiser --Ice.Config=etc/config
```

**Ficheros:** `src/rerun_logger.h` (nuevo), `src/rerun_logger.cpp` (nuevo),
`scripts/rerun_bridge.py` (nuevo), `src/CMakeLists.txt` (añade `rerun_logger.cpp`)

---

## 16. Corrección: `slot_odom_delta` usa odometría medida en lugar de velocidad de comando

**Problema**  
El constraint de movimiento (`odom_delta_tensor`) de cada slot se construía siempre a partir
de la integración de la **velocidad comandada** (`odometry_prior.delta_pose`).  
Sin embargo, la pose inicial del slot se inicializaba desde la **predicción fusionada**
(comando + odometría medida de encoders/IMU) calculada en `apply_dual_prior_fusion`.

Esta inconsistencia creaba una contradicción en el factor de movimiento durante la rotación:
- Pose inicial del slot → `prev_pose + Δ_fused` (usa rot medido del encoder)
- Constraint de movimiento → `curr_pose - prev_pose ≈ Δ_cmd` (usa rot comandado)

Durante los giros, `rot_comandado ≠ rot_medido` (deslizamiento, retardo), por lo que Adam
tenía que "luchar" contra el constraint para alcanzar el theta correcto que el SDF requería.

**Solución**  
Cuando `last_measured_prior_.valid` (odometría FPE disponible), se usa
`last_measured_prior_.delta_pose` para el constraint del slot, con la covarianza del
modelo de ruido de odometría medida (más ajustada).  
En ausencia de odometría medida, se cae al comportamiento anterior (velocidad comandada).

```cpp
if (last_measured_prior_.valid)
{
    slot_odom_delta = last_measured_prior_.delta_pose;
    slot_motion_cov = compute_motion_covariance(last_measured_prior_, true);  // ruido medido
}
else if (odometry_prior.valid)
{
    slot_odom_delta = odometry_prior.delta_pose;
    slot_motion_cov = compute_motion_covariance(odometry_prior, false);
}
```

**Fichero:** `src/room_concept.cpp` (función `update()`, sección BUILD NEW WINDOW SLOT)

---

## 17. Early exit ahora funciona durante rotación (sin bloqueo por `fast_rotation`)

**Problema**  
`try_prediction_early_exit` bloqueaba el early exit incondicionalmente cuando detectaba
rotación (`angular_velocity > angular_velocity_threshold = 0.05 rad/s`), forzando Adam en
cualquier giro, por suave que fuera (~3 deg/s).  
Esto causaba que el sistema "pasara a modo Adam casi automáticamente" al entrar en curvas.

**Causa raíz**  
El bloqueo existía porque, con el `slot_odom_delta` basado en velocidad comandada (bug #16),
la predicción y el constraint eran inconsistentes. Durante la rotación esto era especialmente
visible porque `rot_cmd ≠ rot_real`.

**Solución**  
Con el fix del bug #16, la predicción y el constraint usan la misma fuente (odometría
medida). La predicción de theta ya es precisa: si el SDF evaluado en la pose predicha es
bajo (`mean_sdf_pred < sigma_sdf × prediction_trust_factor`), la rotación ha sido predicha
correctamente y el early exit puede activarse con seguridad.

Se elimina el bloqueo `fast_rotation` de `try_prediction_early_exit`. El check de calidad
SDF es el único árbitro para todos los tipos de movimiento, incluida la rotación:

```
Predicción buena (theta preciso) → SDF bajo  → early exit ✓
Predicción mala  (theta erróneo) → SDF alto  → Adam corre ✓
```

El parámetro `angular_velocity_threshold` se mantiene, pero ahora sólo se usa para los
pesos adaptativos de gradiente en Adam (boost de gradiente en theta durante rotación rápida).

**Fichero:** `src/room_concept.cpp` (función `try_prediction_early_exit`),
`src/room_concept.h` (actualización del comentario del parámetro `angular_velocity_threshold`)

---

## 18. BoundaryQualityGate: peso adaptativo del prior de frontera

**Problema**  
El prior de frontera Gaussiano ancla Adam a la pose MAP del frame anterior.
Cuando esa pose era mala (sdf_mse alto), el prior impedía que Adam convergiera hacia
el mínimo real del SDF, atrapando al optimizador en frames consecutivos de alta pérdida.

**Solución**  
Se escala el peso del prior de frontera por la calidad del frame anterior:

```
boundary_weight = min(1, σ²_sdf / sdf_mse_prev)
```

Cuando la localización es buena (sdf_mse ≈ 0), el prior tiene peso completo (w ≈ 1).
Cuando la localización fue mala (sdf_mse >> σ_sdf), el prior se suprime (w → 0) y Adam
puede moverse libremente hacia el mínimo del SDF.  
Se añade `prev_sdf_mse_` como estado interno, actualizado al final de cada frame y
reseteado en `set_rect_room`/`set_polygon_room`.

**Nuevos parámetros:** `BoundaryQualityGate = true`, `sigma_sdf = 0.15`  
**Ficheros:** `src/room_concept.h`, `src/room_concept.cpp` (`run_adam_loop`),
`src/specificworker.cpp`, `etc/config.toml`

---

## 19. Corrección: prior de frontera desactivado con WindowSize=1 (patrón diente de sierra)

**Problema**  
Con `WindowSize = 1` (single-step), el slot más antiguo es siempre el slot actual.
El prior de frontera anclaba la pose actual a la estimación del frame anterior,
convirtiendo el optimizador en un integrador de error.  
Consecuencia visible: el `sdf_mse` de Adam crecía de forma monótona ~6% por frame
durante ~33 frames (patrón en diente de sierra), hasta que el recovery grid search lo
reseteaba, y el ciclo se repetía indefinidamente incluso con el robot estático.

**Solución**  
El prior de frontera sólo se aplica cuando hay más de un slot en la ventana:

```diff
- if (boundary_prior.valid && !window.empty())
+ if (boundary_prior.valid && window.size() > 1)
```

Con W=1 el prior queda desactivado y Adam optimiza únicamente contra el SDF de la
observación, resolviendo la deriva monótona.

**Fichero:** `src/room_concept.cpp` (función `WindowManager::compute_rfe_loss`)

---

## 20. Modelo de deslizamiento angular del encoder (`EncoderRotSlipK`)

**Problema**  
La fusión dual-prior (comando + encoder) daba al encoder ~85% de peso
(`meas_cov_tt ≈ 0.00014` vs `cmd_cov_tt ≈ 0.0028`). Sin embargo, a velocidades
angulares altas los encoders de rueda subestiman la rotación real por deslizamiento,
haciendo que la predicción fusionada sea peor que usar el comando solo.  
Diagnóstico: RMSE del residuo theta (`innov_theta`) = 2.62° durante rotaciones
agresivas vs 0.69° estático; la predicción subestimaba sistemáticamente la rotación.

**Solución**  
Se infla la covarianza de rotación del prior medido de forma proporcional a la
velocidad angular:

```cpp
if (is_measured_odometry && params.encoder_rot_slip_k > 0.f)
{
    const float ang_speed = std::abs(odometry_prior.delta_pose[2]) /
                            std::max(odometry_prior.dt * 0.001f, 0.001f);
    const float slip_std  = params.encoder_rot_slip_k * ang_speed;
    rotation_std = std::sqrt(rotation_std * rotation_std + slip_std * slip_std);
}
```

A velocidades bajas (robot estático) el encoder sigue dominando la fusión.
A velocidades altas (`|vel_rot| ≫ 0`), la covarianza del encoder crece
cuadráticamente con la velocidad, transfiriendo el peso al prior de comando,
que es más fiable para rotaciones rápidas.

**Nuevo parámetro:** `EncoderRotSlipK = 0.15` (rad de incertidumbre por rad/s)  
**Ficheros:** `src/room_concept.h`, `src/room_concept.cpp` (`compute_motion_covariance`),
`src/specificworker.cpp`, `etc/config.toml`

---

---

## 21. FIX 1 — Suelo de covarianza estacionaria (`StationaryMotionThreshold`)

**Problema**  
Con el robot parado y `stationary_motion_threshold = 0.001 m`, la covarianza diagonal de
traslación era `cov(0,0) = (0.001)² = 1e-6 m²`. Un residuo de tan solo 3.4 cm generaba:

```
loss_motion = 0.5 × (0.034)² / 1e-6 = 578
```

Estos spikes de `loss_motion` dominaban la pérdida total, impidiendo que el criterio de
convergencia se cumpliera y forzando que Adam agotara siempre las iteraciones máximas
en frames estacionarios.

**Solución**  
Se sube el umbral a `0.02 m`, dando `cov(0,0) = 4e-4 m²`. El mismo residuo de 3.4 cm
produce `loss_motion = 1.4` — tres órdenes de magnitud menor.

```toml
StationaryMotionThreshold = 0.02   # m  (anterior: 0.001)
```

**Ficheros:** `src/room_concept.h`, `src/specificworker.cpp`, `etc/config.toml`

---

## 22. FIX 2 — Criterio de convergencia Adam más estricto (`ConvergenceRelTol`)

**Problema**  
`convergence_relative_tol = 0.01` detenía Adam cuando la pérdida relativa cambiaba menos
del 1%. Con la cola larga del motion factor (W=10), esto ocurría en escalones intermedios
(`loss ~15–20`) antes de llegar al plateau real (`loss ~10`), generando poses subóptimas.

**Solución**  
```toml
ConvergenceRelTol = 0.001   # anterior: 0.01
```

Adam ahora solo para en el plateau verdadero. Impacto medido: frames que agotaban iters
con `sdf_mse=6.3 cm` pasan a converger con `sdf_mse=4.4 cm`.

**Ficheros:** `src/room_concept.h`, `src/specificworker.cpp`, `etc/config.toml`

---

## 23. FIX 3 — Incremento de iteraciones máximas para W=10 (`NumIterations`)

**Problema**  
Con `NumIterations = 50` y `WindowSize = 10`, el motion factor acumulado en 9 slots
necesita más iteraciones para drenar su gradiente. Los frames que agotaban 50 iters
reducían la pérdida ×77 (1000→13) con `sdf_mse = 4.4 cm`, pero los que no llegaban al
máximo terminaban con `sdf_mse = 6.3 cm`.

**Solución**  
```toml
NumIterations = 75   # anterior: 50
```

Proporciona margen sin alcanzar los 200 ms del antiguo límite de 100 iters.

**Ficheros:** `etc/config.toml`

---

## 24. Exposición de 20+ parámetros hardcoded al config (`specificworker.cpp`)

Parámetros que estaban fijados en código y se han expuesto al `config.toml` para permitir
experimentación sin recompilar:

| Clave config | Campo en `Params` | Grupo |
|---|---|---|
| `LearningRatePos` | `learning_rate_pos` | Adam |
| `ObsSigma` | `rfe_obs_sigma` | Adam |
| `HuberDelta` | `rfe_huber_delta` | Adam |
| `ConvergenceRelTol` | `convergence_relative_tol` | Adam |
| `ConvergenceMinIters` | `convergence_min_iters` | Adam |
| `SigmaSdf` | `sigma_sdf` | Early exit |
| `PredictionTrustFactor` | `prediction_trust_factor` | Early exit |
| `MinTrackingSteps` | `min_tracking_steps` | Early exit |
| `RotationSdfCoupling` | `rotation_sdf_coupling` | Early exit |
| `RecoveryCooldownFrames` | `recovery_cooldown_frames` | Recovery |
| `VelocityAdaptiveWeights` | `velocity_adaptive_weights` | Pesos adaptativos |
| `LinearVelocityThreshold` | `linear_velocity_threshold` | Pesos adaptativos |
| `AngularVelocityThreshold` | `angular_velocity_threshold` | Pesos adaptativos |
| `WeightBoostFactor` | `weight_boost_factor` | Pesos adaptativos |
| `WeightReductionFactor` | `weight_reduction_factor` | Pesos adaptativos |
| `WeightSmoothingAlpha` | `weight_smoothing_alpha` | Pesos adaptativos |
| `CmdNoiseTrans/Rot/Base` | `cmd_noise_*` | Modelo ruido |
| `OdomNoiseTrans/Rot/Base` | `odom_noise_*` | Modelo ruido |
| `EncoderRotSlipK` | `encoder_rot_slip_k` | Modelo ruido |
| `StationaryMotionThreshold` | `stationary_motion_threshold` | Modelo ruido |

**Ficheros:** `src/specificworker.cpp`, `etc/config.toml`

---

## 25. Boundary prior quality gates — Soluciones B y C

Análisis de convergencia con desplazamiento forzado y obstáculos detectó que el prior de
frontera se "envenenaba" con scans contaminados, causando explosiones de `loss_boundary`
hasta 1.385 y tiempos de recuperación de 25+ frames.

### Causa raíz

`recompute_boundary_prior()` calcula la Hessiana como `H = H_obs(scan) + H_motion + H_prior`.
Si el scan era malo (obstáculo, confusión post-desplazamiento), `H_obs` creaba una precisión
alta en una pose incorrecta. Análogamente, `boundary_prior.mu` se actualizaba desde el slot
descartado incluso si su `sdf_mse` era alto.

### Solución B — Hessiana condicional

Cuando `oldest.sdf_mse_final > boundary_hessian_quality_threshold`, la precisión del prior
se calcula solo desde `motion_prec_tensor` (cinemática pura), sin `H_obs`:

```cpp
const bool use_obs_hessian =
    (oldest.sdf_mse_final < params.boundary_hessian_quality_threshold);
// Si false: H = motion_prec_tensor del slot siguiente
// Si true:  H = H_obs + H_motion + H_prior (comportamiento original)
```

### Solución C — mu condicional

`WindowManager::append()` recibe `mu_quality_threshold`. Si `sdf_mse_final` del slot
descartado supera el umbral, `boundary_prior.mu` no se actualiza — el ancla permanece
en el último lugar conocido fiable:

```cpp
const bool slot_is_good = (window.front().sdf_mse_final < mu_quality_threshold)
                          || !boundary_prior.valid;
if (slot_is_good)
    boundary_prior.mu = ...;
```

### Solución A — Techo de autovalores

Se añade `eigenvalue_clamp_boundary_max` para prevenir priors excesivamente rígidos
independientemente de la causa:

```cpp
evals = evals.cwiseMax(params.eigenvalue_clamp_boundary)
             .cwiseMin(params.eigenvalue_clamp_boundary_max);
```

### Campo `sdf_mse_final` en `WindowSlot`

Nuevo campo que almacena la calidad del slot cuando era el más nuevo, para ser leído
cuando se convierte en el más antiguo:

```cpp
window_mgr_.newest().sdf_mse_final = res.sdf_mse;          // camino Adam
window_mgr_.newest().sdf_mse_final = early->sdf_mse;        // camino early exit
```

**Nuevos parámetros en config:**
```toml
BoundaryHessianQualityThreshold = 0.08   # m
BoundaryMuQualityThreshold      = 0.10   # m
EigenvalueClampBoundaryMax      = 500.0
```

**Ficheros:** `src/room_concept.h`, `src/room_concept.cpp`, `src/specificworker.cpp`,
`etc/config.toml`

---

## 26. Ponderación de puntos lejanos con exponente configurable (`FarPointsExponent`)

**Motivación**  
La ponderación lineal `w_i = dist_i / mean(dist)` ya existía pero estaba desactivada.
Los puntos lejanos tienen un brazo de palanca mayor para la corrección de orientación y
están infrarrepresentados en el gradiente respecto a su valor informativo.

**Implementación**  
Fórmula potencial con normalización y cota inferior:

```
w_i = dist_i^α / mean(dist^α)       ← mean(w) ≈ 1 (escala invariante)
w_i = clamp(w_i, min_weight, ∞)     ← floor para no anular puntos cercanos
w_i = w_i / mean(w_i)               ← re-normalización tras el clamp
```

Con `α=2` (cuadrático): un punto al doble de la distancia media recibe `×4` de peso
(frente a `×2` con `α=1`). Esto alinea la contribución al gradiente de orientación
con el error geométrico real (`R·ε`).

**Nuevos parámetros:**
```toml
FarPointsWeight    = true
FarPointsExponent  = 2.0    # α=1: lineal; α=2: cuadrático (recomendado)
FarPointsMinWeight = 0.1    # floor de peso
```

**Ficheros:** `src/room_concept.h`, `src/room_concept.cpp` (2 puntos: `compute_rfe_loss`
y `compute_rfe_loss_breakdown`), `src/specificworker.cpp`, `etc/config.toml`

---

## 27. Documento de análisis `debug_ADAM.md`

Documento de referencia creado en la raíz del componente con:
- Arquitectura completa del pipeline (`update()`, función de pérdida RFE, todos los términos)
- Análisis de convergencia con logs de W=5–10: causas de no convergencia, métricas medidas
- Descripción detallada de tres experimentos: steady state, desplazamiento forzado, obstáculo
- Análisis del boundary prior: por qué es necesario, cómo se envenena, comparativa de eventos
- Implementación y motivación de las soluciones A, B y C
- Tabla completa de parámetros expuestos y su motivación
- Trabajo pendiente y próximos experimentos

**Fichero:** `debug_ADAM.md` (nuevo)

---

## 28. Correcciones de calidad de código

### 28a. Race condition en `RerunLogger::frame_counter_`

**Problema**  
`frame_counter_` en `rerun_logger.h:119` era un `int` plano compartido entre el hilo
localizador (que llama a `log_frame()` para encolar frames) y el hilo sender (que lo
incrementa en `sender_loop()`). Sin protección, las lecturas y escrituras concurrentes
son un data race definido por el estándar C++ (UB).

**Solución**  
Cambio a `std::atomic<int>` con inicialización en declaración:

```diff
- int                      frame_counter_ = 0;
+ std::atomic<int>          frame_counter_{0};
```

El header ya incluía `<atomic>` (usado por `connected_` y `stop_requested_`), por lo que
no se necesita ningún include adicional.

**Fichero:** `src/rerun_logger.h:119`

---

### 28b. `catch(...){}` silenciosos en `initialize()`

**Problema**  
Los 54 bloques `try { configLoader.get<T>(...); } catch (...) {}` en `specificworker.cpp`
tragaban cualquier excepción sin traza alguna. Un error tipográfico en la clave, un tipo
incorrecto, o un fichero de config corrupto causaban que el parámetro se quedase con el
valor por defecto sin ninguna indicación en el log. Esto dificultaba el diagnóstico cuando
el componente se comportaba de forma inesperada con un config nuevo.

**Solución**  
Reemplazar todos los `catch (...) {}` por `catch (const std::exception& e)` con `qDebug()`:

```diff
- } catch (...) {}
+ } catch (const std::exception& e) { qDebug() << "[config]" << e.what(); }
```

Se usa `qDebug()` (no `qWarning()`) porque los parámetros opcionales ausentes son el caso
normal cuando se usa un config parcial. El mensaje contiene el texto de la excepción del
ConfigLoader, que habitualmente incluye la clave que faltaba.

**Fichero:** `src/specificworker.cpp` (initialize(), 54 ocurrencias, reemplazado con `replace_all`)

---

### Correcciones omitidas con justificación

| Candidato identificado | Diagnóstico final | Decisión |
|---|---|---|
| `ts_plot_sdf_` raw pointer → `unique_ptr` | Widget Qt con padre (`new T(plotFrame)` + `addWidget`) — Qt asume ownership; `unique_ptr` causaría double-delete | **No procede** |
| `compute_rfe_loss_breakdown` duplicación | Ya subsampled a 1/5 frames + `NoGradGuard`; refactor requeriría exponer términos individuales de `run_adam_loop` | **Ya mitigado** |
| `smooth_v_adv_/side_/rot_` dead code | No son dead code: EMA state de `forward_project_pose()` para compensar lag display | **Diagnóstico erróneo** |

---

## Resumen de ficheros modificados

| Fichero | Cambios |
|---|---|
| `src/common_types.h` | Añade `source_ts_ms`, `recv_ts_ms`, `effective_ts_ms()` a `OdometryReading` y `VelocityCommand` |
| `src/room_concept.h` | `lidar_scan` en `UpdateResult`; params nuevos (20+ expuestos + B/C/far_points); `sdf_mse_final` en `WindowSlot`; firma `append()` actualizada; comentarios actualizados |
| `src/room_concept.cpp` | Fix `meas_valid`; Adam lr scaling; midpoint rule; debug log CSV; traza rotación; `lidar_scan`; corner tracking; `slot_odom_delta` medida; eliminación `fast_rotation`; quality-gated boundary prior (B+C+A); far_points potenciado; `sdf_mse_final` en early exit y Adam |
| `src/specificworker.cpp` | Fix signo `odom.adv`; EMA velocidades; visualización sincronizada; timestamps; UI Adam vs Pred; 20+ params config; params B/C/far_points; `catch(...){}` → logging (§28b) |
| `src/specificworker.h` | EMA state renombrado a velocidades |
| `src/viewer_2d.h` | `FrameData` añade `loc_pose`, `use_loc_pose` |
| `src/viewer_2d.cpp` | `update_frame` usa `loc_pose` para dibujar el scan |
| `src/rerun_logger.h` | **Nuevo** — struct `RerunFrame`, clase `RerunLogger` (hilo sender, ring buffer); `frame_counter_` → `std::atomic<int>` (§28a) |
| `src/rerun_logger.cpp` | **Nuevo** — serialización JSON, base64, TCP, reconexión automática |
| `scripts/rerun_bridge.py` | **Nuevo** — bridge Python: TCP → Rerun SDK; blueprint 3D + 4 series |
| `etc/config.toml` | `NumIterations` 10→75; `ConvergenceRelTol` 0.01→0.001; `StationaryMotionThreshold`; 20+ params expuestos; params B/C/far_points |
| `debug_ADAM.md` | **Nuevo** — análisis completo de convergencia, boundary prior y experimentos |
| `RERUN_PIPELINE.md` | **Nuevo** — documentación de la integración Rerun |
| `ANALYSIS.md` | **Nuevo** — análisis adicional de experimentos |
| `benchmarks/` | **Nuevo** — snapshots de métricas de referencia pre/post cambios |
