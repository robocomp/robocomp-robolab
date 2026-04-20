# sdf_localizer

Localizador 2D basado en campo de distancias con signo (SDF) para robots en entornos
interiores estructurados. Implementa un estimador variacional de ventana deslizante
(Realized Free Energy) optimizado con Adam sobre LibTorch.

---

## Descripción

El componente estima la pose `[x, y, θ]` del robot dentro de una habitación cuya geometría
(rectangular o poligonal) se conoce a priori. En cada frame de LiDAR construye un factor
graph sobre los W frames más recientes y lo minimiza con Adam, usando el SDF de la sala
como función de observación.

El 95% de los frames en régimen estacionario se resuelven mediante **early exit** (evaluación
del SDF en la pose predicha sin optimización), reduciendo el coste computacional a ~5 ms por
frame. Adam solo se activa cuando el modelo cinemático no es suficientemente preciso.

---

## Pipeline

```
LiDAR (Lidar3D)  ──►┐
Odometría (FPE)  ──►├──► SensorBuffer / VelocityBuffer / OdometryBuffer
Comando (Xbox)   ──►┘
                          │
                          ▼
               ┌─────────────────────────────┐
               │   RoomConcept::update()      │
               │                              │
               │  1. Predicción cinemática    │  ← fusión dual-prior (cmd + odom)
               │  2. Append slot a ventana    │  ← WindowManager (deque de W slots)
               │  3. Early exit check ────────┼──► SDF(pose_pred) < umbral → devuelve pred
               │  4. Adam (LibTorch)          │  ← minimiza L_RFE sobre W slots
               │  5. Covarianza posterior     │  ← Hessiana via doble backprop
               │  6. Boundary prior update    │  ← quality-gated (Soluciones B+C)
               └─────────────────────────────┘
                          │
               ┌──────────┼──────────────────┐
               ▼          ▼                  ▼
          UpdateResult   CSV log          RerunLogger
          (pose, cov,    tmp/sdf_          (TCP → rerun_bridge.py)
           sdf_mse)      localizer/
                         log.csv
```

### Función de pérdida RFE

```
L_RFE = λ·L_boundary + Σ_t L_obs(t) + Σ_t L_motion(t) + Σ_t L_corner(t)

  L_boundary  = ½ (p₀ - μ)ᵀ Σ⁻¹ (p₀ - μ)           — ancla sobre slot más antiguo
  L_obs(t)    = ½ σ⁻² · Huber_δ(SDF(scan_t, pose_t)) — observación SDF con kernel robusto
  L_motion(t) = ½ (Δp_t - odom_t)ᵀ Σ_mot⁻¹ (...)    — consistencia cinemática
  L_corner(t) = Σ_c ½ σ_c⁻² · Huber(||e_c||²)        — esquinas (opcional)
```

---

## Dependencias

| Librería | Versión mínima | Uso |
|---|---|---|
| LibTorch (CPU o CUDA) | 2.0 | Autograd, Adam, tensores |
| Eigen3 | 3.4 | Álgebra lineal |
| Qt6 | 6.2 | UI, señales, threading |
| RoboComp ICE | — | Interfaces Lidar3D, OmniRobot, FPE |
| Python 3 + rerun-sdk | (opcional) | Telemetría en tiempo real |

---

## Compilación

```bash
cmake -B build && make -C build -j$(nproc)
```

Para CUDA (opcional, fallback automático a CPU si falla):

```bash
cmake -B build -DUSE_CUDA=ON && make -C build -j$(nproc)
```

---

## Ejecución

### Modo estándar

```bash
bin/sdf_localiser --Ice.Config=etc/config
```

### Con telemetría Rerun

```bash
# Terminal 1 — bridge Python (lanza también el visor Rerun)
pip install rerun-sdk numpy
python3 scripts/rerun_bridge.py

# Terminal 2 — componente con Rerun activado
# (en etc/config.toml: RerunEnabled = true)
bin/sdf_localiser --Ice.Config=etc/config
```

El bridge escucha en `TCP localhost:9877`. El componente intenta conectar al iniciar y
reconecta automáticamente si se pierde la conexión.

---

## Configuración principal (`etc/config.toml`)

### Ventana deslizante

```toml
WindowSize        = 10    # slots retenidos (W). W=1 degrada a single-step.
MaxLidarPoints    = 700   # puntos en el slot actual
MaxLidarOldSlot   = 200   # puntos en slots antiguos (subsampleados)
```

### Optimizador Adam

```toml
NumIterations     = 75    # máximo de iteraciones por frame
LearningRatePos   = 0.05  # LR base; se escala a lr/√W internamente
ObsSigma          = 0.05  # σ ruido SDF en la pérdida (m)
HuberDelta        = 0.15  # umbral Huber para saturar outliers (m)
ConvergenceRelTol = 0.001 # criterio de parada relativo
ConvergenceMinIters = 8   # mínimo de iters antes de aplicar criterio
```

### Early exit

```toml
PredictionEarlyExit   = true
SigmaSdf              = 0.15   # σ_sdf para el umbral de salida anticipada (m)
PredictionTrustFactor = 0.5    # threshold = SigmaSdf × factor
RotationSdfCoupling   = 0.5    # tolerancia extra por rotación (m/rad)
MinTrackingSteps      = 5      # warm-up tras reset antes de activar EE
```

### Boundary prior (quality gates)

```toml
BoundaryQualityGate             = true   # peso adaptativo: w = min(1, σ²/sdf_mse_prev)
BoundaryHessianQualityThreshold = 0.08   # m — por encima: Hessiana solo cinemática
BoundaryMuQualityThreshold      = 0.10   # m — por encima: mu no se actualiza
EigenvalueClampBoundaryMax      = 500.0  # techo de precisión del prior
```

### Recovery

```toml
RecoveryLossThreshold    = 0.45  # sqrt(sdf_mse) > umbral → frame malo
RecoveryConsecutiveCount = 10    # frames malos consecutivos para recovery
RecoveryCooldownFrames   = 60    # frames sin detection tras recovery
```

### Modelo de ruido

```toml
StationaryMotionThreshold = 0.02  # suelo de std posición estacionaria (m)
OdomNoiseTrans = 0.08             # fracción ruido translacional odometría
OdomNoiseRot   = 0.04             # fracción ruido rotacional odometría
EncoderRotSlipK = 0.15            # slip rotacional por velocidad (rad/rad·s⁻¹)
```

### Ponderación de puntos lejanos

```toml
FarPointsWeight    = true
FarPointsExponent  = 2.0    # α — α=1: lineal, α=2: cuadrático
FarPointsMinWeight = 0.1    # floor de peso
```

### Telemetría Rerun

```toml
RerunEnabled      = false   # activar para usar con rerun_bridge.py
RerunHost         = "127.0.0.1"
RerunPort         = 9877
RerunSdfEveryN    = 20      # enviar grid SDF cada N frames
RerunSdfResolution = 150    # celdas por eje del grid
```

---

## Integración con Rerun

[Rerun](https://rerun.io/) permite visualizar en tiempo real el estado del localizador:
pose, nube LiDAR transformada, grid SDF, trayectoria, y series temporales de todas las
métricas internas.

### Arquitectura

```
sdf_localizer (C++)              Python bridge           Rerun viewer
┌──────────────────┐  TCP 9877   ┌─────────────────┐    ┌─────────────┐
│ RerunLogger      │──[4B+JSON]──▶│ rerun_bridge.py │────▶│  Rerun UI   │
│ hilo background  │             │                 │    └─────────────┘
└──────────────────┘             └─────────────────┘
```

`RerunLogger` serializa cada frame a JSON (puntos LiDAR y grid SDF en base64 float32 LE)
y lo encola en un ring buffer de hasta 30 frames. Un hilo sender gestiona la conexión TCP
de forma no bloqueante respecto al hilo de localización.

### Entidades registradas

| Path Rerun | Tipo | Contenido |
|---|---|---|
| `world/robot` | Transform3D | Pose estimada (traslación + rotación) |
| `world/robot/body` | Boxes3D | Cuerpo del robot |
| `world/robot/fwd` | Arrows3D | Vector de avance |
| `world/path/adam` | LineStrips3D | Trayectoria (frames con Adam) |
| `world/path/pred` | LineStrips3D | Trayectoria (frames con early exit) |
| `world/lidar` | Points3D | Nube LiDAR en frame mundo |
| `world/sdf` | Transform3D + Image | Grid SDF cada N frames |
| `world/room/outline` | LineStrips3D | Contorno de la sala (una vez) |
| `metrics/timing/*` | Scalar | `t_update`, `t_adam`, `t_cov`, `t_breakdown` (ms) |
| `metrics/loss/*` | Scalar | `loss_init`, `final_loss`, `l_bnd`, `l_obs`, `l_mot`, `l_cor` |
| `metrics/quality/*` | Scalar | `sdf_mse`, `innov_norm`, `cov_xx`, `cov_tt`, `cond_num` |
| `metrics/pipeline/*` | Scalar | `window_size`, `iters`, `early_exit` |

### Uso del bridge

```bash
python3 scripts/rerun_bridge.py [--host 0.0.0.0] [--port 9877]
```

El script lanza automáticamente el visor Rerun con un blueprint preconfigurado:
vista 3D (izquierda) + 4 paneles de series temporales (Timing · Loss · Quality · Pipeline).

---

## Log CSV de diagnóstico

Cuando `DebugLog = true`, se escribe `tmp/sdf_localizer/log.csv` con una fila por frame.

| Columna | Descripción |
|---|---|
| `early_exit` | 1 si el frame usó early exit, 0 si Adam corrió |
| `iters` | Iteraciones Adam usadas (0 en early exit) |
| `sdf_mse` | Error medio absoluto SDF en pose final (m) |
| `final_loss` | Pérdida RFE final |
| `loss_init` | Pérdida antes del primer paso Adam |
| `loss_boundary/obs/motion/corner` | Desglose por término |
| `window_size` | Tamaño actual de la ventana |
| `t_adam_ms` | Tiempo de Adam (ms) |
| `innov_norm` | Norma de la innovación (m) |
| `cov_xx`, `cov_tt` | Varianza diagonal de pose |

---

## Estructura del repositorio

```
sdf_localizer/
├── src/
│   ├── room_concept.h/cpp        — estimador principal (pipeline RFE + Adam)
│   ├── specificworker.h/cpp      — worker RoboComp, UI, buffers
│   ├── room_model.h/cpp          — modelo SDF de la sala (rect. o polígono)
│   ├── corner_detector.h/cpp     — detección y tracking de esquinas
│   ├── rerun_logger.h/cpp        — telemetría TCP hacia rerun_bridge.py
│   ├── epistemic_controller.h    — controlador de exploración activa
│   └── CMakeLists.txt
├── scripts/
│   └── rerun_bridge.py           — bridge Python: TCP JSON → Rerun SDK
├── benchmarks/
│   └── *.json                    — snapshots de métricas de referencia
├── etc/
│   ├── config                    — configuración ICE
│   └── config.toml               — parámetros del localizador
├── debug_ADAM.md                 — análisis de convergencia y experimentos
├── RERUN_PIPELINE.md             — documentación del pipeline de telemetría
├── CHANGES.md                    — historial completo de cambios
└── README.md                     — este archivo
```

---

## Ficheros relevantes para git

**Modificados (ya tracked):**
- `src/room_concept.h` — nuevos params, `sdf_mse_final`, firma `append()`
- `src/room_concept.cpp` — pipeline completo con fixes y quality gates
- `src/specificworker.h` — EMA state
- `src/specificworker.cpp` — fixes, params config, UI
- `src/epistemic_controller.h` — velocidades máximas actualizadas
- `src/CMakeLists.txt` — añade `rerun_logger.cpp`
- `etc/config.toml` — configuración experimental completa
- `etc/config` — configuración ICE
- `CHANGES.md` — historial actualizado

**Nuevos (requieren `git add`):**
- `src/rerun_logger.h` / `src/rerun_logger.cpp`
- `scripts/rerun_bridge.py`
- `debug_ADAM.md`
- `RERUN_PIPELINE.md`
- `ANALYSIS.md`
- `benchmarks/`

**No subir:**
- `tmp/` — logs de ejecución locales
- `generated/` — código autogenerado por RoboComp
- `compile_commands.json`, `build.ninja` — artefactos de build
- `*.zip`, `doc/pipeline.*` — artefactos locales
- `etc/last_robot_pose.txt` — estado efímero en tiempo de ejecución
- `detect_cuda_*.c*` — pruebas locales de detección CUDA

-----
-----
# Developer Notes
This section explains how to work with the generated code of sdf_localiser, including what can be modified and how to use key features.
## Editable Files
You can freely edit the following files:
- etc/* – Configuration files
- src/* – Component logic and implementation
- README.md – Documentation

The `generated` folder contains autogenerated files. **Do not edit these files directly**, as they will be overwritten every time the component is regenerated with RoboComp.

## ConfigLoader
The `ConfigLoader` simplifies fetching configuration parameters. Use the `get<>()` method to retrieve parameters from the configuration file.
```C++
// Syntax
type variable = this->configLoader.get<type>("ParameterName");

// Example
int computePeriod = this->configLoader.get<int>("Period.Compute");
```

## StateMachine
RoboComp components utilize a state machine to manage the main execution flow. The default states are:

1. **Initialize**:
    - Executes once after the constructor.
    - May use for parameter initialization, opening devices, and calculating constants.
2. **Compute**:
    - Executes cyclically after Initialize.
    - Place your functional logic here. If an emergency is detected, call goToEmergency() to transition to the Emergency state.
3. **Emergency**:
    - Executes cyclically during emergencies.
    - Once resolved, call goToRestore() to transition to the Restore state.
4. **Restore**:
    - Executes once to restore the component after an emergency.
    - Transitions automatically back to the Compute state.

### Setting and Getting State Periods
You can get the period of some state with de function `getPeriod` and set with `setPeriod`
```C++
int currentPeriod = getPeriod("Compute");   // Get the current Compute period
setPeriod("Compute", currentPeriod * 0.5); // Set Compute period to half
```

### Creating Custom States
To add a custom state, follow these steps in the constructor:
1. **Define Your State** Use `GRAFCETStep` to create your state. If any function is not required, use `nullptr`.

```C++
states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
                                                      std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
                                                      std::bind(&SpecificWorker::customEnter, this), // On-enter function
                                                      std::bind(&SpecificWorker::customExit, this)); // On-exit function

```
2. **Define Transitions** Add transitions between states using `addTransition`. You can trigger transitions using Qt signals such as `entered()` and `exited()` or custom signals in .h.
```C++
// Syntax
states[srcState]->addTransition(originOfSignal, signal, dstState)

// Example
states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get());

```
3. **Add State to the StateMachine** Include your state in the state machine:
```C++
statemachine.addState(states["CustomState"].get());

```

## Hibernation Flag
The `#define HIBERNATION_ENABLED` flag in `specificworker.h` activates hibernation mode. When enabled, the component reduces its state execution frequency to 500ms if no method calls are received within 5 seconds. Once a method call is received, the period is restored to its original value.

Default hibernation monitoring runs every 500ms.

## Changes Introduced in the New Code Generator
If you’re regenerating or adapting old components, here’s what has changed:

- Deprecated classes removed: `CommonBehavior`, `InnerModel`, `AGM`, `Monitors`, and `src/config.h`.
- Configuration parsing replaced with the new `ConfigLoader`, supporting both .`toml` and legacy configuration formats.
- Skeleton code split: `generated` (non-editable) and `src` (editable).
- Component period is now configurable in the configuration file.
- State machine integrated with predefined states: `Initialize`, `Compute`, `Emergency`, and `Restore`.
- With the `dsr` option, you generate `G` in the GenericWorker, making the viewer independent. If you want to use the `dsrviewer`, you will need the `Qt GUI (QMainWindow)` and the `dsr` option enabled in the **CDSL**.
- Strings in the legacy config now need to be enclosed in quotes (`""`).

## Adapting Old Components
To adapt older components to the new structure:

1. **Add** `Period.Compute` and `Period.Emergency` and swap Endpoints and Proxies with their names in the `etc/config` file.
2. **Merge** the new `src/CMakeLists.txt` and the old `CMakeListsSpecific` files.
3. **Modify** `specificworker.h`:
    - Add the `HIBERNATION_ENABLED` flag.
    - Update the constructor signature.
    - Replace `setParams` with state definitions (`Initialize`, `Compute`, etc.).
4. **Modify** `specificworker.cpp`:
    - Refactor the constructor entirely.
    - Move `setParams` logic to the `initialize` state using `ConfigLoader.get<>()`.
    - Remove the old timer and period logic and replace it with `getPeriod()` and `setPeriod()`.
    - Add the new function state `Emergency`, and `Restore`.
    - Add the following code to the implements and publish functions:
        ```C++
        #ifdef HIBERNATION_ENABLED
            hibernation = true;
        #endif
        ```
5. **Update Configuration Strings**, ensure all strings in the `config` under legacy are enclosed in quotes (`""`), as required by the new structure.
6. **Using DSR**, if you use the DSR option, note that `G` is generated in `GenericWorker`, making the viewer independent. However, to use the `dsrviewer`, you must integrate a `Qt GUI (QMainWindow)` and enable the `dsr` option in the **CDSL**. 
7. **Installing toml++**, to use the new .toml configuration format, install the toml++ library:
```bash
mkdir ~/software 2> /dev/null; git clone https://github.com/marzer/tomlplusplus.git ~/software/tomlplusplus
cd ~/software/tomlplusplus && cmake -B build && sudo make install -C build -j12 && cd -
```
8. **Installing qt6 Dependencies**
```bash
sudo apt install qt6-base-dev qt6-declarative-dev qt6-scxml-dev libqt6statemachineqml6 libqt6statemachine6

mkdir ~/software 2> /dev/null; git clone https://github.com/GillesDebunne/libQGLViewer.git ~/software/libQGLViewer
cd ~/software/libQGLViewer && qmake6 *.pro && make -j12 && sudo make install && sudo ldconfig && cd -
```
9. **Generated Code**, When the component is generated, a `generated` folder is created containing non-editable files. You can delete everything in the `src` directory except for:
- `src/specificworker.h`
- `src/specificworker.cpp`
- `src/CMakeLists.txt`
- `src/mainUI.ui`
- `README.md`
- `etc/config`
- `etc/config.toml`
- Your Clases...