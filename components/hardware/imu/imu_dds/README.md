# imu_dds

Bridges the IMU from the RoboComp **ICE** world onto the CORTEX **zero-copy DDS media
plane**, so IMU samples reach consumers out-of-band instead of through the DSR graph.

```
 IMU.Source = "ice"                         IMU.Source = "phidget"
 ──────────────────                         ──────────────────────
 webots-bridge / p3bot-bridge               Phidget Spatial on USB
        │ ICE getDataImu()                         │ phidget22 (driver thread)
        ▼                                          ▼
   read_sample_ice()                       PhidgetImu::read()
        └──────────────► ImuSample ◄───────────────┘
                             │            (SI units, one struct)
                             ▼
                      ImuDDSPublisher
                             │  FastDDS  domain 7, "rc/imu/data", ImuFrame
                             ▼
        consumers (room_concept ImuIngestor, ...) attach via the descriptor

    imu_dds ── ICE MediaPlaneDDS::getMediaDescriptor() ──▶ robot_concept ──▶ DSR "imu" node
                                                            (relays the descriptor)
```

## Two sources, one stream

`IMU.Source` picks where samples are **read**; it changes nothing about what is
**published**. Both paths fill the same `ImuSample` (SI units, `src/imu_sample.h`) and go
through the same `ImuDDSPublisher::publish()`, so the bytes on `rc/imu/data` are identical
either way — a consumer cannot tell which source is running. Swapping simulation for real
hardware is this one line, and nothing downstream changes.

| `IMU.Source` | Reads from | Needs |
|---|---|---|
| `"ice"` (default) | `RoboCompIMU::getDataImu()` on `Proxies.IMU` | an IMU component (webots-bridge, p3bot-bridge, phidgetimu…) |
| `"phidget"` | a real Phidget Spatial via `phidget22` (`src/phidget_imu.{h,cpp}`) | `libphidget22-dev` at build time, a device on USB at run time |

The Phidget reader is push-driven: the driver delivers samples on its own thread at the
configured `DataInterval`, `PhidgetImu` keeps the newest one under a mutex, and `read()`
reports `false` when nothing new arrived — so `compute()` stays a plain polling loop and
never blocks on the device. It converts the driver's **g** and **deg/s** to m/s² and rad/s
on the way in, maps the `PUNK_DBL` (1e300) "unmeasurable" sentinel to NaN, and re-anchors
the device clock (which counts from attach, not epoch) onto wall-epoch ms so
`ImuFrame.stamp_ms` keeps its contract while inter-sample deltas stay the device's own.

Phidget support is **optional at build time**: if `phidget22.h` is absent, `phidget_imu.cpp`
is not compiled and `IMU.Source = "phidget"` refuses at startup with the apt line to fix it,
rather than breaking the build on every simulation box.

It publishes the **same topic, domain and frame type** that `robot_concept` used to bridge
itself (`Media.imu_topic = "rc/imu/data"`, domain 7, `ImuFrame.v1`), so a consumer cannot
tell the two producers apart. That is the point: with `imu_dds` running, the consumer's code
is identical in simulation and on real hardware.

The component is **not** a DSR agent and never joins the graph. It only serves its media
descriptor over ICE; `robot_concept` is what relays that JSON onto the `imu` node.

## How it paces itself

The IMU is far faster than any sensible fixed `Period.Compute` (~115 Hz in Webots). So
`Period.Compute` is only the **idle / source-down ceiling**: `compute()` measures the source's
own period from its sample stamps, drives the real period down to ~half of it, and drops
samples whose stamp it already published. Pacing on our own loop timing instead would be
circular and would spiral the rate downwards.

## ⚠️ One producer per topic

`robot_concept` bridges the IMU over ICE onto `rc/imu/data` whenever it has not adopted an
external producer. Running `imu_dds` on that same topic **at the same time** puts two
producers on one topic and a consumer cannot tell which sample it got — the failure looks
like an intermittently wrong IMU, not a duplicate. Set `Media.imu_source = "dds"` in
`robot_concept`'s config (or give it a `MediaPlaneDDS` proxy to this component so `"auto"`
negotiates) before enabling this one.

## Dependencies
The following dependencies are required to build and run imu_dds. Ensure they are installed and properly configured on your system before proceeding:
- **eProsima Fast DDS** + **Fast CDR** (`find_package(fastdds)` / `fastcdr`)
- **libphidget22-dev** — OPTIONAL, only for `IMU.Source = "phidget"`:
  `sudo apt install libphidget22-dev`. Without it the component builds and runs fine on the
  ICE source; CMake says so at configure time.
- `active_inference/common/media_transport` — the shared `rc::media` publisher and the
  generated `ImuFrame` IDL support. Pulled in by source from `src/CMakeLists.txt`; nothing
  to install, but the `active_inference` tree must be checked out beside this one.
- An IMU source implementing `RoboCompIMU` (webots-bridge, p3bot-bridge, phidgetimu, ...).

## Configuration parameters

| Parameter | Meaning |
|---|---|
| `Proxies.IMU` | The ICE IMU source. `10007` on every bridge (webots-bridge, p3bot-bridge, shadow `imu`). |
| `Endpoints.MediaPlaneDDS` | Descriptor query port (`11891`; siblings use zed 12002, ricoh 10099, helios 11890, bpearl 11889). |
| `PublishDDS` | Master gate. `false` ⇒ no plane is created and the descriptor is empty, which tells `robot_concept` to keep bridging. |
| `DDS.Domain` | `7`, the CORTEX media domain — deliberately not the DSR domain 0, so media churn cannot perturb cortex resync. |
| `DDS.Topic` | `"rc/imu/data"`. Must match `robot_concept`'s `Media.imu_topic`. |
| `DDS.HistoryDepth` | KEEP_LAST depth. |
| `DDS.SharedMemoryOnly` | SHM transport for same-board consumers. |
| `DDS.DataSharing` | Zero-copy loans. Leave `false` (churn-safe) — see `media_transport.h`. |
| `Period.Compute` | Idle/source-down **ceiling** only; see above. |
| `IMU.Source` | `"ice"` or `"phidget"` — where samples are read. Same published stream either way. |
| `Phidget.DataIntervalMs` | Sampling period asked of the device (8 = 125 Hz); clamped up to its `MinDataInterval`. |
| `Phidget.Serial` / `Phidget.HubPort` | Device selection; `-1` = any. Set only with several Phidgets on the bus. |
| `Phidget.OpenTimeoutMs` | Attach wait at startup. Timing out is not fatal — it keeps retrying, so plugging in later works. |
| `Phidget.UseAHRS` | Orientation from the on-board AHRS (quaternion → rpy). Off ⇒ `rpy` stays 0 rather than guessed. |
| `Phidget.GyroVar` / `Phidget.AccVar` | Nominal per-sample variances (SI²) published with the data; negative = "unknown". |

## Starting the component
To avoid modifying the config file directly in the repository, you can copy it to the component's home directory. This prevents changes from being overridden by future `git pull` commands:

```bash
cd <imu_dds's path> 
cp etc/config etc/yourConfig
```

After editing the new config file we can run the component:

```bash
cmake -B build && make -C build -j12 # Compile the component
bin/imu_dds etc/yourConfig # Execute the component
```
-----
-----
# Developer Notes
This section explains how to work with the generated code of imu_dds, including what can be modified and how to use key features.
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
