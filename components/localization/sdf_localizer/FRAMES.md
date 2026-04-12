# Viewer3D – Session State Document

**Purpose:** Complete reference for resuming work on the Qt3D 3D viewer from a new chat session.  
**Last updated:** after all picking, lidar-pool, and furniture features were built and confirmed compiling.

---

## 1. Project Context

| Item | Value |
|---|---|
| Component root | `/home/pbustos/robocomp/components/beta-robotica-class/ainf_slamo/` |
| Binary | `bin/ainf_slamo` |
| Build command | `cmake --fresh -S . -B build_make -G "Unix Makefiles" && make -C build_make -j20` |
| Qt6-3D headers | `/usr/include/x86_64-linux-gnu/qt6/Qt3DExtras/` |
| CMake guard | `add_compile_definitions(HAS_QT3D)` – all viewer code wrapped in `#ifdef HAS_QT3D` |
| SVG layout file | `completo_unified_v2.svg`, layer **"Furniture"**, labels in Spanish (Inkscape) |

---

## 2. Source Files

| File | Role |
|---|---|
| `src/viewer_3d.h` | Class declarations – 175 lines |
| `src/viewer_3d.cpp` | Full implementation – 609 lines |
| `src/specificworker.h` | Declares `Viewer3D* viewer_3d_` + `draw_lidar_` toggle member |
| `src/specificworker.cpp` | Integration glue – wires slots to viewer calls |

---

## 3. Class Architecture

### 3.1 `rc::WebotsStyleCameraController`

Inherits `Qt3DExtras::QAbstractCameraController`.  Overrides `moveCamera()`.

| Gesture | Action | Axes used |
|---|---|---|
| Left drag | Orbit around view centre | `rxAxisValue`, `ryAxisValue` |
| Right drag | Pan (translate camera + view centre) | `rxAxisValue`, `ryAxisValue` |
| Scroll wheel | Zoom (dolly) | `tzAxisValue` |

**Key**: `tx/tyAxisValue` are keyboard-only; mouse deltas always come through `rx/ry`.

Exact sign conventions (final, confirmed working):
```cpp
// Orbit
cam->panAboutViewCenter ( state.rxAxisValue * lookSpeed() * dt);
cam->tiltAboutViewCenter(-state.ryAxisValue * lookSpeed() * dt);

// Pan
cam->translate(QVector3D(-state.rxAxisValue * linearSpeed() * dt,
                         -state.ryAxisValue * linearSpeed() * dt,
                          0.f),
               Qt3DRender::QCamera::TranslateViewCenter);

// Zoom
cam->translate(QVector3D(0.f, 0.f, state.tzAxisValue * linearSpeed() * dt),
               Qt3DRender::QCamera::DontTranslateViewCenter);
```
Camera constructed with: `linearSpeed = 20.f`, `lookSpeed = 180.f`.

---

### 3.2 `rc::Viewer3D`

```
QObject
  └─ Viewer3D
       ├─ Qt3DExtras::Qt3DWindow*          window_
       ├─ QWidget*                         container_    (embedded via createWindowContainer)
       ├─ Qt3DCore::QEntity*               root_entity_
       ├─ Qt3DCore::QTransform*            robot_transform_
       ├─ QQuaternion                      robot_base_rot_
       ├─ Qt3DCore::QEntity*               floor_entity_
       ├─ vector<Qt3DCore::QEntity*>       wall_entities_
       ├─ vector<Qt3DCore::QEntity*>       obstacle_entities_
       ├─ vector<Qt3DCore::QEntity*>       furniture_entities_
       ├─ vector<Qt3DCore::QTransform*>    lidar_hi_xf_   [500]
       ├─ vector<Qt3DCore::QTransform*>    lidar_lo_xf_   [250]
       ├─ QLabel*                          pick_label_
       ├─ QTimer*                          pick_timer_
       ├─ float                            wall_height_  = 2.5f
       └─ float                            robot_half_h_ = 0.15f
```

**Signal:** `void objectPicked(const QString& name)` — emitted on Shift+Right-click.

**FurnitureItem struct:**
```cpp
struct FurnitureItem {
    std::string     label;     // Inkscape layer label (Spanish OK)
    Eigen::Vector2f centroid;  // room (x, y) in metres
    Eigen::Vector2f size;      // AABB (width_x, depth_z) in metres
};
```

---

## 4. Coordinate Convention

### Architectures
1. **RoboComp / 2D Planner Setup:**
   * Z-Up, Right-Handed Cartesian
   * `X+`: Right
   * `Y+`: Forward
   * `Z+`: Up
   * Note: Zero degrees pointing strictly along the `Y+` (Forward) axis. Positive $\theta$ refers to counter-clockwise rotation.

2. **Qt3D Default Setup:**
   * Y-Up, Right-Handed Cartesian
   * `X+`: Right
   * `Y+`: Up
   * `Z+`: Forward

### Robot Body-Frame Velocity Convention

The robot's body frame follows the same Y+ = Forward convention:

| Body-frame axis | Direction | OmniRobot API arg | ControlCommand field |
|---|---|---|---|
| **X** (lateral) | Right | `advx` (1st arg) | `adv_x` |
| **Y** (forward) | Forward | `advz` (2nd arg) | `adv_y` |
| **θ** (angular) | **CCW positive** | `rot` (3rd arg) | `rot` |

### Unified Rotation Convention: CCW+ Everywhere

All internal buffers, integrators, the planner, and all hardware boundaries use **CCW+** (standard math).
After the Webots proto fix, all sources and sinks are natively CCW+ — **no sign conversions anywhere**.

| Hardware boundary | Raw convention | Conversion |
|---|---|---|
| **Joystick** (input) | CCW+ | Stored directly (after Webots proto fix) |
| **OmniRobot API** (output) | CCW+ | Sent directly (after Webots proto fix) |
| **FullPoseEstimation** (input) | CCW+ | Stored directly (after Webots proto fix) |

| Internal component | Convention | Notes |
|---|---|---|
| **Velocity buffer** (`VelocityCommand.rot`) | **CCW+** | Joystick/planner stored directly |
| **Odometry buffer** (`OdometryReading.rot`) | **CCW+** | FullPoseEstimation stored directly |
| **Planner** (`ControlCommand.rot`) | **CCW+** | `atan2(-x_body, y_body)` gives CCW+ |
| **All integrators** | **CCW+** | `dtheta = rot * dt` — no negation anywhere |
| **forward_project_pose** | **CCW+** | `v_rot = odom->rot` or `vel->rot` — no negation |

⚠️ **Critical rule**: No sign conversions needed — all boundaries are natively CCW+ after the Webots proto fix.

**Forward direction in world frame** for a robot at heading θ:
- `forward = (-sin θ, cos θ)` — **NOT** `(cos θ, sin θ)` because θ=0 faces Y+.
- `right   = ( cos θ, sin θ)`

**World-to-body transform** (inverse rotation by θ):
```
body_x =  cos θ · world_x + sin θ · world_y   (lateral component)
body_y = -sin θ · world_x + cos θ · world_y   (forward component)
```

**Body-to-world transform** (rollout kinematics):
```
dx = (adv_x · cos θ − adv_y · sin θ) · dt
dy = (adv_x · sin θ + adv_y · cos θ) · dt
dθ = rot · dt
```

**Target heading** (desired θ so robot faces target with Y+ axis):
```
target_heading = atan2(-to_target.x(), to_target.y())
```
⚠️ **Common bug**: using `atan2(y, x)` assumes θ=0→X+ and produces a permanent π/2 heading error → robot spins without advancing.

### Project Implementation (Room 2D → Qt3D 3D)

**Mapping:** Room 2D `(x, y)` → Qt3D 3D `(−x, height, y)`

- Qt3D Y axis = up (height)
- Room Y maps to Qt3D Z
- Room X is **negated** (`−x`) everywhere — this was a global fix for a scene X-flip artefact
- Consequently, while heading naturally turns counter-clockwise in RoboComp, to maintain the same visual alignment while translating over negated X, the global **Wall/Robot rotation angle signs are evaluated positively** without inversion `qRadiansToDegrees(+theta_rad)`.

This convention is explicitly applied in:
- `build_floor` → `translation(-cx, 0, cy)`
- `rebuild_walls` → `translation(-mid.x, h/2, mid.y)`, `rotationY(+angle_rad_deg)`
- `update_robot_pose` → `translation(-x, robot_half_h_, y)`
- `update_obstacles` → `translation(-cx, h/2, cy)`
- `update_furniture` → `translation(-centroid.x, 0, centroid.y)`
- `update_lidar_points` → `QVector3D(-wx, p.z(), wz)`

---

## 5. Lighting

```cpp
// Directional light (warm sun from upper-left)
QDirectionalLight  colour=(255,252,240)  intensity=0.55  dir=(-0.4,-1.0,-0.3).normalised

// Point ambient fill (soft from above, no attenuation)
QPointLight        colour=(200,205,215)  intensity=0.25
                   constantAttenuation=1, linear=0, quadratic=0
                   position=(0, 30, 0)
```

Background clear colour: `QColor(28, 28, 38)` (dark blue-grey).

---

## 6. Materials (colours)

| Object | Diffuse (R,G,B) | Ambient | Shininess |
|---|---|---|---|
| Floor | (65,62,56) | (38,36,32) | 0 + specular=(0,0,0) |
| Walls | (248,248,245) | (180,180,178) | 2 |
| Robot | (50,110,220) | (20,45,95) | 60, spec=(180,200,255) |
| Obstacles | (240,110,25) | (100,44,10) | 40 |
| Table | (160,100,55) | (70,44,24) | 20 |
| Bench | (130,85,45) | (55,36,19) | 15 |
| Pot (pot_only) | (195,155,95) | (90,70,40) | 10 |
| Tree (tree_only) | (55,130,45) | (25,60,20) | 5 |
| Vitrina proxy | (180,200,220) | (70,80,90) | 40 |
| Lidar HELIOS (hi) | (50,230,50) green | (20,100,20) | 0 |
| Lidar BPEARL (lo) | (50,220,220) cyan | (20,90,90) | 0 |

---

## 7. Mesh Inventory

All meshes live in `meshes/` (component root). All are **Z-up**, **base centred at origin**.

| File | Dimensions | Notes |
|---|---|---|
| `shadow.obj` | 12 MB robot | Faces +Z; needs `−90°X * 180°Y` base correction |
| `table.obj` | 1.2 × 0.8 × 0.75 m | 40v 30f, 4 box legs |
| `bench.obj` | 1.5 × 0.4 × 0.52 m seat | 56v 42f, I-supports |
| `pot_only.obj` | r_bot=0.30 r_top=0.42 h=0.80 m | 98v 120f |
| `tree_only.obj` | trunk r=0.09, canopy r_max=1.15, h=3.6 m | 246v 348f, 3-tier canopy |

**Mesh path resolution** (used throughout cpp):
```cpp
const QString root = QDir(QCoreApplication::applicationDirPath() + "/..").absolutePath();
// e.g. root + "/meshes/shadow.obj"
```

**Robot base rotation** correction (Z-up mesh → Y-up Qt3D, facing correct direction):
```cpp
robot_base_rot_ = QQuaternion::fromAxisAndAngle(0,1,0, 180.f)   // 180°Y (after testing 90°/-90° offsets finding inversion fix necessary)
                * QQuaternion::fromAxisAndAngle(1,0,0, -90.f);  // -90°X
```

**Furniture base rotation** (all furniture meshes are also Z-up):
```cpp
const QQuaternion base_rot = QQuaternion::fromAxisAndAngle(1,0,0, -90.f);  // -90°X only
// benches additionally:
QQuaternion::fromAxisAndAngle(0,1,0, 90.f) * base_rot
```

---

## 8. Object Picking System

```
QPickingSettings (on window_->renderSettings())
  setPickMethod(TrianglePicking)      ← REQUIRED; without this pressed() never fires
  setPickResultMode(NearestPick)
  setFaceOrientationPickingMode(FrontAndBackFace)
```

`attach_picker(entity, name)` adds a `QObjectPicker` and connects `pressed`:
```cpp
if (ev->button()    == Qt3DRender::QPickEvent::RightButton &&
    ev->modifiers() &  static_cast<int>(Qt::ShiftModifier))
{
    emit objectPicked(name);
    // show pick_label_ centered bottom of container_ for 3 s
}
```

Entities with pickers: Robot, each Wall N, each Obstacle N, and all furniture entities.

**Overlay label**: `QLabel` styled `rgba(0,0,0,170)` semi-transparent dark, 14px bold white, shown for 3 s.

---

## 9. Lidar Point Pool

Pre-allocated at construction time via `init_lidar_pool()`:

| Pool | Count | Radius | Colour |
|---|---|---|---|
| `lidar_hi_xf_` (HELIOS) | 500 | 0.04 m | green (50,230,50) |
| `lidar_lo_xf_` (BPEARL) | 250 | 0.04 m | cyan (50,220,220) |

Spheres: `rings=4, slices=6` (minimal geometry), one shared mesh+material per layer.  
Hidden position: `(0, −500, 0)`.

`update_lidar_points(pts_hi, pts_lo, rx, ry, rtheta)`:
1. Rotates robot-frame `(x,y,z)` to world frame using `(cos_t, sin_t)`
2. Applies X-negation: `QVector3D(-wx, p.z(), wz)`
3. Strides through pts if count > pool size (uniform subsampling)
4. Parks remaining spheres back at hidden position

---

## 10. Integration in `specificworker.cpp`

```cpp
// initialize()
viewer_3d_ = new rc::Viewer3D(splitter, 2.5f);
splitter->addWidget(viewer_3d_->container_widget());
connect(viewer_3d_, &rc::Viewer3D::objectPicked,
        this, [](const QString& n){ qInfo() << "Picked:" << n; });

// display_robot()
viewer_3d_->update_robot_pose(x, y, theta);

// draw_room_polygon()
viewer_3d_->rebuild_walls(room_polygon_);

// draw_temp_obstacles()
viewer_3d_->update_obstacles( /* vector<vector<Eigen::Vector2f>> */ );

// draw_furniture()  – builds FurnitureItem list from SVG polygons
//   computes per-polygon AABB in room metres, passes label+centroid+size
viewer_3d_->update_furniture(items);

// after draw_lidar_points() – gated by draw_lidar_ bool
if (draw_lidar_)
    viewer_3d_->update_lidar_points(pts_hi, pts_lo, rpx, rpy, rpt);
```

**SVG label matching** (case-insensitive `.contains()`):

| Substring | Mesh used |
|---|---|
| `"mesa"` or `"table"` | `table.obj` (scaled to SVG AABB) |
| `"banco"` or `"bench"` | `bench.obj` (90°Y extra rotation) |
| `"maceta"`, `"plant"`, or `"pot"` | `pot_only.obj` + `tree_only.obj` (two entities) |
| `"vitrina"`, `"cabinet"`, `"shelf"` | `bench.obj` proxy (glass-blue colour) |

---

## 11. Known Issues / Resolutions

| Problem | Fix applied |
|---|---|
| `QObjectPicker::pressed` never fires | Set `TrianglePicking` on `renderSettings()->pickingSettings()` |
| Wrong button comparison | Use `Qt3DRender::QPickEvent::RightButton` (not `Qt::RightButton`) |
| Wrong modifier comparison | Use `static_cast<int>(Qt::ShiftModifier)` (not `QPickEvent::ShiftModifier`) |
| MOC compile error (`Q_SIGNALS` swallowing public methods) | Add `public:` section after the `Q_SIGNALS:` block |
| Scene mirrored left-right | Negate X in all world placements; flip wall rotation sign |
| White specular hotspot on dark floor | `specular=(0,0,0)`, `shininess=0` |
| Robot leans / faces wrong direction | `robot_base_rot_ = 180°Y * −90°X` |
| Pan not working with mouse | `tx/tyAxisValue` are keyboard-only; mouse Δ goes to `rx/ry` |
| Furniture not found in SVG | Labels are Spanish (`mesa`, `banco`, `maceta`); old matcher was English |
| Mesh file not found at runtime | Use absolute path from `QCoreApplication::applicationDirPath() + "/.."` |

---

## 12. Remaining / Future Work

- [ ] **Dedicated vitrina mesh**: `bench.obj` is a proxy; a proper glass cabinet mesh would look better
- [ ] **Trajectory ribbon**: draw robot path as a 3D line or flat ribbon on the floor
- [ ] **Click-to-navigate**: cast a ray from a mouse click to the floor plane, emit a navigation target pose
- [ ] **Furniture rotation**: SVG polygons carry orientation angles; currently furniture is always axis-aligned
- [ ] **Room topology**: doors/windows are in the SVG but not yet rendered as wall gaps
- [ ] **Shadow / AO**: consider `Qt3DRender::QRenderCapture` or a dedicated render pass for visual quality

---

## 13. Build State

Last confirmed clean build: `[100%] Built target ainf_slamo` with all features:
- Qt3D viewer with camera controller ✅
- Robot + walls + floor + obstacles ✅
- Furniture (all 4 types) ✅
- Lidar pool (HELIOS 500 + BPEARL 250) ✅
- Object picking (TrianglePicking, Shift+RightClick) ✅
