# Lidar3D
A brief introduction to the component. Describe its purpose, functionality, and any specific features here.
```
<YOUR BRIEFING>
```

## Dependencies
The following dependencies are required to build and run Lidar3D. Ensure they are installed and properly configured on your system before proceeding:
    - Install repo [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)
    - Install libopen3d-dev

## Run
Set ip of the running computer in config and in lidar, by accessing the lidar ip in a browser, section Setting.
    
The difference between lidars is the ports:
    Helios
    - MSOP = 6699
    - DIFOP = 7788
    Pearl
    - MSOP = 6698
    - DIFOP = 7787

## Starting the component
To avoid modifying the config file directly in the repository, you can copy it to the component's home directory. This prevents changes from being overridden by future `git pull` commands:

```bash
cd <Lidar3D's path> 
cp etc/config etc/yourConfig
```

After editing the new config file we can run the component:

```bash
cmake -B build && make -C build -j12 # Compile the component
bin/Lidar3D etc/yourConfig # Execute the component
```

## Robot-body self-filter (removing "stale" points on the robot)

The raw scan contains returns off the robot's own body, base, wheels, arm, cables and
mounts. Because they are rigidly attached, they **move with the robot** and, if not removed
at the source, poison downstream navigation. This component removes them **once**, in
`compute()` (`src/specificworker.cpp`), upstream of every output (the image projection, the
Ice buffer and the DDS media plane), so consumers never see them. All the logic lives in
`src/mesh_filter.cpp` (`MeshFilter::filter`) and is enabled per-config with `MeshFilter.enabled = true`.

### How a point is decided (in order)

Each point is transformed from the **device frame** into the **robot/body frame** using the
static mount (`body <- sensor`) read from `shadow.json` (`mount_file` / `mount_key`), then a
point is **kept** only if it passes *all* of these; it is **dropped** if it fails any:

1. **Vertical band** — keep only `Floor.z < z < Top.z` (mm, robot frame). Drops the ground
   plane and anything above the ceiling.
2. **Body footprint disc** — drop if the point is within `Footprint.radius` (m) of the robot
   Z-axis, inside `[Footprint.z_min, Footprint.z_max]` (mm). A tight, full-height cylinder
   around the body.
3. **Near-floor skirt** — drop if within the wider `Skirt.radius` (m), but only inside the
   *low* band `[Skirt.z_min, Skirt.z_max]` (mm). Catches near-floor self/ground-reflection
   returns that sit past the body footprint, **without** hiding upright obstacles (their
   returns above `Skirt.z_max` survive).
4. **Mesh proximity** — drop if within `Dilate` (m) of the robot STL surface (`shadow.stl`,
   Embree, 6-ray sphere query).

> A point is removed if it lands in **either** disc **or** the mesh shell. The discs and the
> mesh are complementary: the mesh is precise but only covers what is modelled; the discs are
> blunt but cover what the mesh does not.

### Why the discs exist (and are not redundant with the mesh)

`shadow.stl` models only the **central column**: measured extent is
`X ±0.239 m, Y ±0.230 m, Z 0.003 → 1.309 m` — a footprint of only ~24 cm radius. It does
**not** include the wider wheeled base/skirt, the arm, sensor mounts, handles or cabling.
Returns off those parts sit **more than `Dilate` (5 cm)** from the modelled surface, so the
mesh test misses them and they survive. The footprint disc and the near-floor skirt exist to
remove exactly those un-modelled, rigidly-attached returns. They are **navigation-safe**:
nothing inside the robot's own footprint can be a real external obstacle.

### Configuration keys

Set per sensor in `etc/config_*` (both `[Section]`-TOML and flat `Section.key` forms work).
Current Webots values:

| Key                | helios | bpearl | Unit | Meaning |
|--------------------|:------:|:------:|:----:|---------|
| `MeshFilter.enabled` | true | true | bool | master switch for the whole self-filter |
| `Floor.z`          | 110    | 110    | mm   | drop at/below this height (ground) |
| `Top.z`            | 2100   | 2100   | mm   | drop at/above this height (ceiling) |
| `Dilate`           | 0.05   | 0.05   | m    | mesh self-radius (6-ray `tfar`) |
| `Footprint.radius` | 0.55   | 0.55   | m    | full-height body disc (`0` disables) |
| `Footprint.z_min`  | 110    | 110    | mm   | footprint disc lower bound |
| `Footprint.z_max`  | 1350   | 1350   | mm   | footprint disc upper bound (top of body) |
| `Skirt.radius`     | 0.75   | 0.75   | m    | wider near-floor disc (`0` disables) |
| `Skirt.z_min`      | 110    | 110    | mm   | skirt lower bound |
| `Skirt.z_max`      | 600    | 350    | mm   | skirt upper bound (keep low for safety) |

On start each instance prints its resolved values, e.g.:

```
[MeshFilter] robot='Shadow' bounds Min(-0.239,-0.230,0.003) Max(0.239,0.230,1.309) floor_z=110 top_z=2100 dilate=0.050 footprint_r=0.550m z[110,1350] skirt_r=0.750m z[110,600]
```

### Tuning when the real robot behaves differently

The Webots values are a starting point; the real robot's base, arm pose and reflectivity
differ. Watch the scan in a cenital viewer and adjust — **config only, no rebuild**, just
restart the instance so it reloads:

| Symptom | Knob |
|---------|------|
| Stray point **near the robot, near the floor**, just outside the removed area | raise `Skirt.radius` (reach further out) or `Skirt.z_max` (reach higher) |
| Stray point **near the robot at body height** (not near floor) | raise `Footprint.radius`. Costs navigation clearance — it blanks a full-height cylinder, so prefer the skirt for near-floor strays |
| Stray point **sits in the gap** (past `Footprint.radius` and above `Skirt.z_max`) | raise `Skirt.z_max` until it is covered — this was the fix for the helios rear-left stray (`350 → 600`) |
| **Real obstacles disappear** close to the robot | you over-blanked: lower `Footprint.radius` / `Skirt.radius`, or lower `Skirt.z_max` so upright obstacles reappear above it |
| Thin robot part (cable/antenna) still returns, hugging the body | raise `Dilate` a little (grows the mesh shell) before widening a disc |
| Ground plane / low ceiling leaks in | raise `Floor.z` / lower `Top.z` |

Keys are independent per sensor, so tune helios and bpearl separately. Set a disc's `radius`
to `0` to disable just that disc. Because the mount comes from `shadow.json`, a *global* shift
of all self-points (everything offset the same way) points to a wrong mount rather than a disc
size — fix the RT edge / `mount_file` instead of inflating the discs.

> **Note:** the self-filter is currently enabled only in the Webots configs. The real-robot
> configs (`config_helios_jetson`, `config_helios_flip`, `config_pearl_jetson`) do **not** set
> `MeshFilter.enabled`; add the block above (mesh + discs) there when moving to the real robot,
> re-measuring the discs against the physical base.

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
