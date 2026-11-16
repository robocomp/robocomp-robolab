# Lidar3D
RoboComp driver for **RoboSense 3D LiDARs** (Helios and Bpearl). It reads the point cloud, transforms it
into the robot frame and serves it through the `Lidar3D` interface (full cloud, angular sectors,
2D-distance threshold, colour cloud and points projected onto a 360º camera image).

It works with both:
- **Real lidar**: through the RoboSense SDK [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver) (UDP MSOP/DIFOP packets).
- **Simulator** (Webots): reads the cloud from another `Lidar3D` proxy (the simulator bridge) and applies the same processing.

```
 RoboSense lidar --UDP--> rs_driver ─┐
                                     ├─> robot-frame transform ─> sort by phi ─> buffer ─> Lidar3D interface
 Webots (Lidar3D proxy) ─────────────┘
```

## Dependencies
- RoboComp (with the new code generator, `ConfigLoader`, toml++ and Qt6, see the *Developer Notes* below).
- OpenCV 4, Eigen3, OpenMP, cppitertools.
- [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver) — **optional**, only needed for the real lidar:
  ```bash
  sudo apt install libpcap-dev
  mkdir ~/software 2> /dev/null; git clone https://github.com/RoboSense-LiDAR/rs_driver.git ~/software/rs_driver
  cd ~/software/rs_driver && cmake -B build && sudo make install -C build -j12 && cd -
  ```

### Building with or without rs_driver
`rs_driver` is detected automatically at configure time:

| Situation | Result |
|---|---|
| `rs_driver` installed | Full build: real lidar **and** simulator (`lidar3D: rs_driver found ...`) |
| `rs_driver` not installed | **Simulator-only** build, with a CMake warning (`lidar3D: SIMULATOR-ONLY build`) |
| `-DLIDAR3D_SIMULATOR_ONLY=ON` | Forces the simulator-only build even if `rs_driver` is installed |

A simulator-only binary refuses to start with `simulator = false` and prints an explicit error,
instead of failing silently.

```bash
cmake -B build && make -C build -j12                              # auto-detect
cmake -B build -DLIDAR3D_SIMULATOR_ONLY=ON && make -C build -j12  # force simulator-only
```
> The binary is always written to `bin/Lidar3D`, whatever the build folder.

## Configuration
Ready-made configs in `etc/`:

| File | Lidar | Mode |
|---|---|---|
| `config_helios_jetson`, `config_helios_flip` | Helios | Real |
| `config_pearl_jetson` | Bpearl | Real |
| `config_helios_webots`, `config_P3bot_helios_webots` | Helios | Simulator |
| `config_pearl_webots` | Bpearl | Simulator |
| `config_*_webots_choca` | Helios / Bpearl | Simulator (user-specific paths) |

### Parameters
| Parameter | Description |
|---|---|
| `Endpoints.Lidar3D` | Port where this component serves `Lidar3D` (by convention 11990 Helios, 11989 Bpearl). |
| `Proxies.Lidar3D` | Source `Lidar3D` proxy, only used when `simulator = true` (Webots bridge, usually port 11988). |
| `lidar_model` | `0` = Helios (`RSHELIOS`), `1` = Bpearl (`RSBP`). In simulation it selects the `"helios"` / `"bpearl"` name requested from the proxy. |
| `simulator` | `true`: read from the proxy. `false`: real lidar through `rs_driver`. |
| `dest_pc_ip_addr` | IP of the computer receiving the lidar packets (real lidar only). |
| `msop_port`, `difop_port` | Lidar UDP ports (real lidar only, see below). |
| `rx`, `ry`, `rz` | Lidar→robot rotation (rad, XYZ). Use `.` as decimal separator. |
| `tx`, `ty`, `tz` | Lidar→robot translation (mm). |
| `mount_file`, `mount_key` | *Optional.* DSR model (`shadow.json`) and node name (`helios`, `bpearl`) to read the extrinsic from. |
| `Period.Compute`, `Period.Emergency` | State periods (ms). |

#### Extrinsics: `mount_file` vs `rx..tz`
- If `mount_file` is set, the lidar→robot transform is read from the `RT` edge pointing to the node
  `mount_key` in the DSR model (rotation in rad, translation in metres → converted to mm). This is the
  single source of geometry and must match `lidar3d_dds`.
- If reading that file fails for **any** reason (missing file, invalid JSON, node not found, no `RT`
  edge...) the component **aborts** with the cause, it does **not** fall back to `rx..tz`.
- Remove `mount_file` from the config to use the `rx..tz` values on purpose.

### Real lidar network setup
Set the IP of the receiving computer both in the config (`dest_pc_ip_addr`) and in the lidar, opening
the lidar IP in a browser (default `192.168.1.200`), section *Setting*. The ports depend on the model:

| Lidar | MSOP | DIFOP |
|---|---|---|
| Helios | 6699 | 7788 |
| Bpearl | 6698 | 7787 |

## Interface `Lidar3D`
Points are returned in the robot frame (mm), sorted by `phi` (`atan2(x, y)`, rad, in `[-π, π]`).

| Method | Description |
|---|---|
| `getLidarData(name, start, len, decimationDegreeFactor)` | Points in the angular sector `[start, start+len]` (rad, handles the ±π wrap). `len >= 2π` returns the full cloud. `decimationDegreeFactor` keeps one ray every N degrees (`1` = no decimation). |
| `getLidarDataWithThreshold2d(name, distance, decimationDegreeFactor)` | Points whose 2D distance to the robot is below `distance` (mm). |
| `getLidarDataProyectedInImage(name)` | Full cloud. |
| `getLidarDataArrayProyectedInImage(name)` | Points projected onto the 360º (equirectangular) camera image, Helios only. |
| `getColorCloudData()` | Full cloud as `int16` X/Y/Z arrays (colour fixed to white). |
| `getLidarDataByCategory(...)` | Not implemented (returns empty). |

The `name` parameter is ignored: the component serves a single lidar.

Every method returns empty data until the component has finished initialising.

## Starting the component
To avoid modifying the config files in the repository directly, copy one and edit the copy. This
prevents changes from being overwritten by future `git pull` commands:

```bash
cd <Lidar3D's path>
cp etc/config_helios_jetson etc/yourConfig   # or the *_webots one for simulation
```

Then build and run:

```bash
cmake -B build && make -C build -j12 # Compile the component
bin/Lidar3D etc/yourConfig           # Execute the component
```

In simulation, start Webots and its `Lidar3D` bridge (the one in `Proxies.Lidar3D`) first.

## Notes
- `HIBERNATION_ENABLED` is defined in `specificworker.h`: with no calls for 5 s the component slows
  down its period, and recovers on the next call.
- The camera projection (`lidar2cam`) uses hard-coded extrinsics/resolution in `initialize()`, with
  different values for simulator and real robot.

-----
-----
# Developer Notes
This section explains how to work with the generated code of Lidar3D, including what can be modified and how to use key features.
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
