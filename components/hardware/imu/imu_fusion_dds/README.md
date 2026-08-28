# imu_fusion_dds

Publica la pose del robot por `FullPoseEstimationPub` (ICE) y por un plano
DDS zero-copy adicional. Tiene **dos modos de funcionamiento**, seleccionados
por `Simulation` en el config, con la **misma interfaz de salida** en ambos
para que el resto de la arquitectura conmute entre robot real y simulación
sin más cambio que ese valor:

- **`Simulation = false` (robot real)**: fusiona el IMU Phidget con la
  odometría de la base (`SVD48VBase`) mediante un ESKF (Error-State Kalman
  Filter).
- **`Simulation = true` (Webots, vía `webots-bridge`)**: sin IMU ni ESKF —
  `webots-bridge` ya da posición y velocidad *ground truth*
  (`Supervisor.getPosition()`/`getVelocity()`, sin ruido de sensor real que
  fusionar) por `FullPoseEstimationPub`; este componente las convierte y
  republica directamente (ver `_publish_passthrough` en `specificworker.py`).

Componente RoboComp puro, sin grafo DSR: no depende de `pydsr` ni escribe/lee
del grafo. Es la conversión a componente del antiguo agente DSR `imu_fusion`
(la versión con DSR y corrección de deriva vía la pose de `room_concept` se
conserva intacta en `agents/imu_fusion` / `agents/imu_fusion_dds`, por si hace
falta retomar esa vía más adelante).

## Dependencias

**Robot real** (`Simulation = false`):
- Paquete Python `Phidget22` (`pip install Phidget22`) + drivers nativos de
  Phidgets instalados en el sistema (ver
  [docs de Phidgets para Linux](https://www.phidgets.com/docs/OS_-_Linux)).
- El IMU Phidget Spatial Precision 3/3/3 físicamente conectado.
- Un publicador de `FullPoseEstimationPub` con la odometría de la base
  (`SVD48VBase`) corriendo y accesible (topic `Endpoints.FullPoseEstimationPubPrefix`,
  por defecto `"base"`).

**Simulación** (`Simulation = true`):
- Ningún driver ni librería de IMU — no se usa `Phidget22` ni el interfaz
  `IMU`/`imu_proxy` en absoluto en este modo (el `requires IMU` del `.cdsl`
  ha quedado sin uso tras pasar al passthrough; ver nota más abajo).
- `webots-bridge` corriendo, con Webots simulando el robot, publicando
  `FullPoseEstimationPub` con posición absoluta (metros) y velocidad de
  cuerpo ya calculada (`adv`/`side`/`rot`) en el mismo topic que se
  suscribe este componente (ver `Endpoints.FullPoseEstimationPubPrefix` en
  `etc/config_webots` — debe coincidir con el prefijo que use
  `webots-bridge` para publicar, no necesariamente `"base"`).

**Ambos modos**, opcional:
- Módulo nativo `pose_dds_native` (FastDDS + pybind11, ver
  `src/dds_publisher.h/.cpp`, `src/pose_dds_bindings.cpp`, `src/CMakeLists.txt`)
  para el plano DDS zero-copy (`PublishDDS = true`). Si no está compilado, el
  componente sigue funcionando igual, solo sin ese plano adicional (aviso en
  el log al arrancar).

## Qué hace y qué no

- **Sí**: publica el pose del robot (fusionado con ESKF en robot real,
  ground-truth relayado en simulación). Ambas publicaciones son
  **diferenciales** (deltas/velocidades desde el último ciclo), no una pose
  absoluta:
  - **ICE** (`FullPoseEstimationPub`, topic propio — ver
    `Proxies.FullPoseEstimationPubPrefix`): mismo convenio de ejes y campos
    que `SVD48VBase` (mm, rad, deltas + velocidades) **en ambos modos**. Hoy
    lo consume `room_concept`, que solo usa `pose.adv/side/rot` (velocidades).
  - **DDS zero-copy** (`pose_dds_native`, opcional — `PublishDDS=true`):
    velocidades (mundo y cuerpo) + incertidumbre (sigma), en SI. Los campos
    `x/y/yaw` (posición absoluta) **sí se publican de verdad en simulación**
    (es la pose ground-truth real de Webots, sin deriva) pero se publican a
    `0.0` en el robot real: al quitar DSR se perdió la única fuente de
    corrección de deriva (la pose de `room_concept`), así que ahí no hay
    ancla fiable para una posición absoluta — publicar una posición
    "acumulada sin límite de deriva" induciría a error a cualquier
    consumidor. En simulación no aplica esa limitación (sigma se publica a
    0 también, por confianza perfecta: es ground truth, no una estimación).
- **No**: en robot real, no corrige deriva con ninguna referencia externa
  (room_concept, SLAM, GPS...). El estado interno del ESKF
  (`state["pos"]`/`state["yaw"]`) es dead-reckoning puro (odom+IMU) y deriva
  sin límite con el tiempo — es normal y esperado, no es un bug.

## Robot real: ESKF + Phidget

Lee el Phidget Spatial directamente en proceso (`src/phidget.py`,
`PhidgetIMU`), a su tasa nativa (~250 Hz, hilo del propio driver). Aplica la
rotación de montaje `R_body_imu = diag(1,-1,-1)` (el Phidget está montado con
Z hacia abajo). Fusiona con la odometría de la base mediante el ESKF
(`src/ekf.py`): `predict()` por cada muestra IMU, `update_odom()` por cada
mensaje de odometría (gateado por distancia de Mahalanobis).

### Máquina de estados (solo robot real)

`WAITING_IMU → COLLECTING_STATIC → RUNNING ⇄ DEGRADED / FAULT`: espera
conexión del Phidget, recoge una ventana en reposo para inicializar el ESKF
(`IMU.static_collect_s`), y fusiona en `RUNNING`. `DEGRADED` si no llega
odometría por `Fusion.odom_timeout_s`; `FAULT` si no llega IMU por
`IMU.timeout_s` (se recupera solo si el IMU vuelve a enviar datos). En
simulación no existe esta máquina de estados: `compute()` no hace nada (todo
es event-driven desde el callback de suscripción, ver
`FullPoseEstimationPub_newFullPose`/`_publish_passthrough`).

### Depuración: grabación y visualización en vivo (solo robot real)

`Fusion.enable_recording` / `Fusion.enable_view` activan, respectivamente,
volcado a CSV (`/tmp/traj_*.csv`, `/tmp/odom_updates_*.csv`,
`/tmp/imu_samples_*.csv`) y una ventana `pyqtgraph` en un proceso aparte
(`src/plot_viewer.py`). Ambos acumulan la trayectoria integrada por el ESKF
**exclusivamente para depuración/validación visual** — para comparar la
fusión contra la odometría pura de la base (dead-reckoning sin IMU) y contra
las incertidumbres/sesgos internos del filtro. No representa una
localización absoluta fiable (ver sección anterior).

En simulación estos dos flags se fuerzan a `False` en el propio código,
independientemente de lo que diga `etc/config_webots`: todo el
recording/plot está atado a campos del estado del ESKF (sesgos, NIS,
covarianzas...) que no existen en el modo passthrough. Para depurar en
simulación, `_publish_passthrough` imprime un log de diagnóstico ~1 Hz
(`Sim: pos=... yaw=... | adv=... side=... rot=...`).

## Simulación: passthrough (sin ESKF)

`_publish_passthrough` (llamado desde `FullPoseEstimationPub_newFullPose`
cuando `Simulation=true`) convierte cada mensaje de `webots-bridge` al mismo
formato de salida que usa el ESKF del robot real (mismos campos `diff`/
`state` que consume `_publish_fused_odom`), sin pasar por ningún filtro:

- `pose.adv`/`side`/`rot` (velocidad de cuerpo ya calculada por Webots,
  ground truth) → delta ICE y velocidad, usando `dt` desde el mensaje
  anterior.
- `pose.rz` (yaw absoluto real, normalizado a `[-π,π]`) → orientación
  usada para rotar la velocidad de cuerpo a frame mundo, y pose absoluta en
  el plano DDS.
- `pose.x`/`y` (posición absoluta real, metros) → pose absoluta en el plano
  DDS.

**Nota sobre `requires IMU`**: el `.cdsl` sigue declarando `requires IMU`
(y `Proxies.IMU` en el config) de una implementación anterior que emulaba el
IMU pidiendo `getAcceleration()`/`getAngularVel()` a un simulador — se
abandonó ese enfoque en favor del passthrough directo de pose/velocidad, así
que ese proxy ha quedado **sin uso** en ambos modos. Pendiente de limpieza:
quitar `requires IMU` del `.cdsl` y regenerar (`robocompdsl` + `cmake`).

## Interfaces (`imu_fusion_dds.cdsl`)

- `implements MediaPlaneDDS` — `getMediaDescriptor()`: descriptor JSON del
  plano DDS, para que `robot_concept` lo relaye al grafo (si se usa DSR en
  otro punto de la arquitectura; este componente en sí no lo necesita).
- `subscribesTo FullPoseEstimationPub` — en robot real, odometría de
  `SVD48VBase` (entrada al ESKF); en simulación, pose/velocidad ground-truth
  de `webots-bridge` (entrada al passthrough).
- `publishes FullPoseEstimationPub` — pose diferencial (ver arriba), topic
  propio, mismo formato en ambos modos.
- `requires IMU` — **sin uso actualmente** en ningún modo (ver nota arriba).

## Configuration parameters

Dos ficheros de ejemplo en `etc/`: `config` (robot real, `Simulation=false`)
y `config_webots` (simulación, `Simulation=true`) — idénticos salvo ese
valor. Ajustar `Endpoints.FullPoseEstimationPubPrefix` en `config_webots` si
`webots-bridge` publica con un prefijo de topic distinto al configurado.

- `Simulation`: `false` = ESKF + Phidget real; `true` = passthrough de
  `webots-bridge`.
- `IMU.*`: parámetros del driver Phidget e inicialización estática
  (`interval_ms`, `static_collect_s`, `static_omega_thr`, `timeout_s`,
  `invert_yaw_rate`) — **solo aplican en robot real**.
- `ESKF.*`: ruidos del filtro (Allan Variance) y ruido adaptativo
  estático/dinámico — **solo aplican en robot real**.
- `Fusion.*`: timeouts, tamaño de cola de odometría, flags de depuración
  (`enable_recording`, `enable_view`, forzados a `False` en simulación),
  `invert_odom_yaw_rate` — timeouts/cola solo aplican en robot real.
- `PublishDDS` / `DDS.*`: activa y configura el plano DDS zero-copy
  (dominio, topic, QoS) — aplica en ambos modos.

## Starting the component

To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <imu_fusion_dds's path>
```
```
cp etc/config config          # robot real
# o bien:
cp etc/config_webots config   # simulación (Webots + webots-bridge)
```

After editing the new config file we can run the component:

```
bin/imu_fusion_dds config
```
