# Rerun Integration Pipeline (sdf_localizer)

Este documento resume el pipeline de telemetria hacia Rerun, que metricas hay disponibles y donde se calculan.

## Estado actual (audit)

- Existe puente TCP+JSON en `src/rerun_logger.h` y `src/rerun_logger.cpp`.
- Existe consumidor Python en `scripts/rerun_bridge.py` que publica en Rerun.
- **Integrado en el localizador**: `RoomConcept::run` crea y alimenta `RerunFrame` por update.
- **En build**: `src/CMakeLists.txt` incluye `../src/rerun_logger.cpp` y `../src/rerun_logger.h`.
- **Visual debug activo** en bridge: contorno de sala corner-to-corner (`world/room/outline`) dibujado una sola vez y tabla de poses (`metrics/debug/pose_table_doc`).

## Sobre SDK C++ de Rerun

Si, Rerun ofrece API/SDK en C++. Se puede usar para evitar el puente Python y loggear directamente desde el componente, pero requiere cablear dependencias y decidir estrategia de build (vendorizado/submodulo/paquete del sistema).

## Diagrama de bloque (pipeline + metricas)

```mermaid
flowchart TD
    A[Lidar3D + FPE + Joystick\nSpecificWorker callbacks] --> B[Buffers\nSensorBuffer / VelocityBuffer / OdometryBuffer]
    B --> C[RoomConcept::run]
    C --> D[RoomConcept::update]

    D --> E1[Prediction\ncompute_odometry_prior + compute_measured_odometry_prior\napply_dual_prior_fusion]
    D --> E2[RFE optimization\nrun_adam_loop + WindowManager::compute_rfe_loss]
    D --> E3[Posterior covariance\ncompute_posterior_covariance]
    D --> E4[Early-exit\ntry_prediction_early_exit]

    E1 --> F[UpdateResult]
    E2 --> F
    E3 --> F
    E4 --> F

    F --> G[UI plots\nSpecificWorker::update_ui]
    F --> H[CSV tmp log\nRoomConcept::init_debug_log + debug_log_ writes]
    F --> I[RerunFrame builder \n(implementado)]

    I --> J1[Opcion A\nRerunLogger TCP JSON]
    J1 --> J2[scripts/rerun_bridge.py]
    J2 --> K[Rerun Viewer]

    I --> L[Opcion B\nRerun C++ SDK directo]
    L --> K
```

## Mapa de metricas y referencias de codigo

### 1) Estado y calidad de localizacion

- `x, y, theta`: `src/room_concept.cpp` en `RoomConcept::update` (extraccion de pose final en `res.state` y `res.robot_pose`).
- `sdf_mse`: `compute_sdf_mse_unscaled(...)` usado en `RoomConcept::update`.
- `innovation`/`innovation_norm`: calculadas en `RoomConcept::update` y tambien en `try_prediction_early_exit`.
- `cov_xx, cov_tt, cond_num`: de `compute_posterior_covariance` y asignadas en `res.covariance` / `res.condition_number`.

### 2) Pipeline/optimizacion

- `iters`, `final_loss`: `run_adam_loop(...)` -> `res.iterations_used`, `res.final_loss`.
- `early_exit`: `try_prediction_early_exit(...)` (si se toma, `iterations_used = 0`).
- `window_size`: `window_mgr_.size()`.
- `tracking_steps`: `tracking_step_count_`.

### 3) Terminos de coste (FE desagregado)

En `WindowManager::compute_rfe_loss(...)`:
- `loss_boundary` (prior de borde),
- `loss_obs` (SDF observacional),
- `loss_motion` (consistencia dinamica),
- `loss_corner` (observacion de esquinas, si esta habilitado),
- `n_corner_obs`.

Estos terminos ya se vuelcan en el CSV debug (`debug_log_`) en `src/room_concept.cpp`.

### 4) Timing del pipeline

En `src/room_concept.cpp` y `src/specificworker.cpp`:
- `t_update_ms`, `t_adam_ms`, `t_cov_ms`, `t_breakdown_ms` (si se empaquetan en `RerunFrame`).
- `buf/draw ms` del compute thread en `SpecificWorker::compute`.

### 5) Datos geometricos para 3D

- Nube lidar robot-frame: `lidar.first` en `RoomConcept::update` y `res.lidar_scan`.
- Pose robot world-frame: `res.robot_pose`.
- Trayectoria acumulada: secuencia de `res.state[2], res.state[3]`.
- SDF grid: disponible via evaluacion de modelo (`Model::sdf_at_pose` / `Model::sdf`); envio cada N frames recomendado.
- Ejes del robot: a partir de `theta` (transform `SE(2)`), representable como `Arrows3D`.

## Propuesta de instrumentacion Rerun (incremental)

### Fase 1 (rapida) — completada

1. Incluir `rerun_logger.cpp` en `src/CMakeLists.txt`.
2. Crear `RerunLogger` en `RoomConcept` o `SpecificWorker`.
3. En cada frame, enviar:
   - pose, early_exit, iters, final_loss,
   - `loss_boundary/obs/motion/corner`,
   - `sdf_mse`, `innovation_norm`, `cov_xx`, `cov_tt`, `cond_num`,
   - nube lidar + trayectoria.

### Fase 2 (analisis fino) — en progreso

1. Publicar `SDF grid` cada `N` frames (configurable).
2. Publicar timings detallados del pipeline en series temporales.
3. Publicar estado planner/objetivo/velocidades para correlacionar control-localizacion.
4. Añadir debug visual de offset/proyeccion (contorno de sala corner-to-corner y tabla de poses pred/opt/innov).

### Fase 3 (directo C++)

1. Sustituir puente Python por Rerun C++ SDK.
2. Mantener interfaz `RerunFrame` para aislar cambios.
3. Conservar fallback TCP (debug remoto) detras de flag de config.

## Flags de configuracion recomendados

Bajo `[RoomConcept]` o `[Rerun]` en `etc/config.toml`:

- `RerunEnabled`
- `RerunUseCppSdk`
- `RerunHost`
- `RerunPort`
- `RerunSdfEveryN`
- `RerunSdfResolution`
- `RerunMaxQueue`
- `RerunLogPoints`
- `RerunLogPath`
- `RerunLogTiming`

## Notas de rendimiento

- Nube + SDF pueden dominar ancho de banda/CPU; usar decimacion y envio por periodo.
- Mantener cola acotada y politicas drop-oldest (ya existe en `RerunLogger`).
- Evitar bloquear el hilo de localizacion: todo envio asincrono.

