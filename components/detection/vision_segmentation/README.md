# vision_segmentation

`vision_segmentation` is a RoboComp component that reads RGBD data, runs YOLO segmentation, projects segmented regions to 3D points, and publishes the results through the `ImageSegmentation` ICE interface.

## What the component does

- Consumes frames from `CameraRGBDSimple`.
- Runs YOLO segmentation (`ultralytics`) over the RGB image.
- Builds segmented objects with:
	- `label`
	- `score`
	- `imagePolygon` (2D contour points)
	- `points3D` (depth-projected 3D points)
- Serves image/objects through ICE methods.
- Optionally renders:
	- left panel: RGB + overlays
	- right panel: Qt3D segmented point cloud

## Implemented ICE interface

Module: `RoboCompImageSegmentation`

Main data types:

- `Point2D { int x; int y; }`
- `Point3D { float x; float y; float z; }`
- `SegmentedObject { string label; float score; Polygon imagePolygon; PointCloud points3D; }`
- `TImage { compressed, cameraID, width, height, depth, focalx, focaly, alivetime, period, image }`
- `TDepth { compressed, cameraID, width, height, focalx, focaly, alivetime, period, depthFactor, depth }`
- `TData { ObjectList objects; TImage image; TDepth depth; long timestamp; }`

Implemented methods:

- `ObjectList getSegmentedObjects(bool points3d)`
- `TImage getImage()`
- `TDepth getDepth()`
- `TData getAll(bool points3d)`

## Requirements

### System

- Linux
- RoboComp installed
- ICE runtime available
- CMake + compiler toolchain

### Python

- Python 3.10+
- `numpy`
- `opencv-python`
- `ultralytics`
- `PySide6`
- `rich`

Install Python dependencies:

```bash
pip install numpy opencv-python ultralytics PySide6 rich
```

## Build

From component root:

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## Configuration

Default config file: `etc/config`

Relevant parameters:

- `Proxies.CameraRGBDSimple`: source RGBD endpoint.
- `Endpoints.ImageSegmentation`: endpoint exposed by this component.
- `Display`: `True`/`False` to enable GUI rendering.
- `ProxyThread`: `True`/`False` to read camera frames in a background thread.
- `Period.Compute`: compute loop period (ms).

Example:

```ini
Proxies.CameraRGBDSimple = "camerargbdsimple:tcp -h localhost -p 10096"
Endpoints.ImageSegmentation = "tcp -p 14111"
Display = False
ProxyThread = True
Period.Compute = 50
```

## Run

Recommended:

```bash
cp etc/config config
bin/vision_segmentation config
```

## Notes

- With `Display=False`, rendering work is skipped to maximize throughput.
- The component prints FPS periodically.
- `points3d=False` in `getAll`/`getSegmentedObjects` returns segmented objects with empty `points3D` for lower CPU cost.
- In `ProxyThread=True` mode, camera reads are done only by the background thread to avoid parallel proxy access.
- Snapshot publication (`objects`, `image`, `depth`, `timestamp`) is protected with a lock to avoid concurrent read/write races.

## Coordinate frames

This component uses three coordinate frames for 3D points:

### 1) Camera frame (depth projection input)

From depth intrinsics projection:

- `x_cam`: right
- `y_cam`: up
- `z_cam`: forward (depth)

Projection equations:

- `x_cam = (u - cx) * z / fx`
- `y_cam = (cy - v) * z / fy`
- `z_cam = z`

### 2) RoboComp frame (published `Point3D`)

Published segmented object points (`SegmentedObject.points3D`) are converted to RoboComp frame:

- `X+`: right
- `Y+`: forward
- `Z+`: up

Conversion from camera frame:

- `x_robocomp = x_cam`
- `y_robocomp = z_cam`
- `z_robocomp = y_cam`

### 3) Qt3D frame (visualization only)

For on-screen point cloud rendering, RoboComp points are remapped to Qt3D frame:

- `+X`: right
- `+Y`: up
- `+Z`: forward (view-center direction convention)

Current remap used in display path:

- `x_qt = -x_robocomp`
- `y_qt = z_robocomp`
- `z_qt = y_robocomp`

Important:

- The sign flip in `x_qt` is applied only for Qt3D drawing (to match the current viewer orientation).
- ICE output (`Point3D`) remains in RoboComp frame.

## Troubleshooting

- **Low FPS (expected around 20Hz but getting less)**
	- Set `Display=False` to disable GUI overhead.
	- Try `ProxyThread=False` and compare performance.
	- Use a lighter segmentation model checkpoint if needed.
	- Use `points3d=False` from clients when 3D points are not needed.

- **Intermittent segfault / random crash**
	- Ensure all clients and this component are regenerated/restarted against the same `ImageSegmentation` interface version.
	- Keep a single process reading the camera proxy in threaded mode (`ProxyThread=True` is already implemented this way).
	- If instability persists, test with `Display=False` to rule out Qt3D/GUI driver issues.

- **Proxy errors / no input frames**
	- Verify `Proxies.CameraRGBDSimple` endpoint in `etc/config`.
	- Ensure the camera component is running and reachable.
	- Check ICE network settings and port conflicts.

- **Model not loading**
	- Confirm `ultralytics` is installed in the active Python environment.
	- Ensure model weights are accessible from the runtime environment.

- **No overlays or empty segmented objects**
	- Ensure the input image has valid content and size.
	- If no clients are requesting image/object data, some optional overlay work may be skipped for performance.

- **Qt / display errors**
	- If running headless, keep `Display=False`.
	- Ensure `PySide6` and Qt3D packages are installed when display is enabled.
