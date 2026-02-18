# CameraSimple Streaming Component

`CameraSimple` is a Python-implemented component that acts as a driver for USB/V4L2 cameras and provides real-time video streaming with GPU acceleration. This component integrates:

- **RoboComp ICE interface**: `CameraSimple.idsl` interface to serve images to other components
- **Real-time streaming**: RTSP/WebRTC/HLS transmission via MediaMTX
- **GPU acceleration**: H264 encoding with NVIDIA NVENC (nvh264enc) or CPU fallback (x264enc)
- **Low latency**: Optimized pipeline for real-time robotic applications

## System Architecture

```
┌─────────────────┐
│  USB Camera     │
│  (/dev/video)   │
└────────┬────────┘
         │ V4L2
         ▼
┌─────────────────────────────────────┐
│  CameraSimple Component (Python)    │
│  ┌───────────────────────────────┐  │
│  │  cv2.VideoCapture (V4L2)      │  │
│  │  ├─ MJPEG decode              │  │
│  │  └─ Frame capture (640x480)   │  │
│  └──────────┬────────────────────┘  │
│             │ Raw BGR frames        │
│             ▼                        │
│  ┌───────────────────────────────┐  │
│  │  GStreamer Pipeline (subprocess)│ │
│  │  fdsrc → rawvideoparse        │  │
│  │       → videoconvert          │  │
│  │       → nvh264enc (GPU)       │  │
│  │       → h264parse → flvmux    │  │
│  │       → rtmpsink              │  │
│  └──────────┬────────────────────┘  │
│             │ H264/RTMP             │
└─────────────┼─────────────────────┘
              │
              ▼
┌─────────────────────────────────────┐
│  MediaMTX Server                    │
│  ┌───────────────────────────────┐  │
│  │  RTMP Input :1935             │  │
│  │  ├→ RTSP Output :8554         │  │
│  │  ├→ WebRTC Output :8889       │  │
│  │  └→ HLS Output :8888          │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Clients            │
    │  - Browsers         │
    │  - VLC/FFplay       │
    │  - Other components │
    └─────────────────────┘
```

## System Dependencies

### Required packages

```bash
# GStreamer and plugins
sudo apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-rtsp

# For NVIDIA GPU (optional but recommended)
sudo apt-get install -y \
    gstreamer1.0-plugins-nvenc \
    nvidia-cuda-toolkit

# Network and diagnostic tools
sudo apt-get install -y \
    net-tools \
    ffmpeg \
    v4l-utils
```

### Python packages

```bash
pip install opencv-python PySide6
```

### MediaMTX

The `mediamtx` binary is already included in the repository. If you need to update it:

```bash
# Download latest version from https://github.com/bluenviron/mediamtx/releases
wget https://github.com/bluenviron/mediamtx/releases/download/vX.X.X/mediamtx_vX.X.X_linux_amd64.tar.gz
tar -xzf mediamtx_*.tar.gz
mv mediamtx ~/robocomp/components/robocomp-robolab/components/hardware/camera/camera_streamer/
chmod +x mediamtx
```

## GStreamer Pipeline

The component automatically detects the platform and configures the optimal pipeline:

### Pipeline NVIDIA GPU (PC/Workstation)

```bash
fdsrc fd=0 ! 
rawvideoparse format=bgr width=640 height=480 framerate=30/1 ! 
queue max-size-buffers=1 leaky=downstream ! 
videoconvert ! video/x-raw,format=I420 ! 
nvh264enc preset=low-latency-hq rc-mode=cbr bitrate=5000 gop-size=30 ! 
h264parse ! 
flvmux streamable=true ! 
rtmpsink location=rtmp://localhost:1935/camera sync=false
```

**Features:**
- **fdsrc fd=0**: Reads raw frames from stdin
- **rawvideoparse**: Parses raw BGR data to structured video
- **nvh264enc**: NVIDIA hardware H264 encoding
  - `preset=low-latency-hq`: Low latency, high quality
  - `rc-mode=cbr`: Constant bitrate (5 Mbps)
  - `gop-size=30`: Keyframe every second (30fps)
- **flvmux**: FLV multiplexing for RTMP
- **rtmpsink**: RTMP output to MediaMTX

### Pipeline Jetson (Orin/Xavier)

```bash
fdsrc fd=0 ! 
rawvideoparse format=bgr width=640 height=480 framerate=30/1 ! 
queue max-size-buffers=1 leaky=downstream ! 
videoconvert ! video/x-raw,format=I420 ! 
nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! 
nvv4l2h264enc preset-level=1 control-rate=1 bitrate=5000000 
  iframeinterval=30 insert-sps-pps=true maxperf-enable=true ! 
h264parse ! 
flvmux streamable=true ! 
rtmpsink location=rtmp://localhost:1935/camera sync=false
```

**Jetson-specific features:**
- **nvvidconv**: Conversion with NVMM memory (optimized for Jetson)
- **nvv4l2h264enc**: V4L2 encoder for Jetson
- **maxperf-enable=true**: Maximum performance

### Pipeline CPU Fallback

```bash
fdsrc fd=0 ! 
rawvideoparse format=bgr width=640 height=480 framerate=30/1 ! 
queue max-size-buffers=1 leaky=downstream ! 
videoconvert n-threads=4 ! video/x-raw,format=I420 ! 
x264enc tune=zerolatency speed-preset=ultrafast bitrate=5000 key-int-max=30 ! 
h264parse ! 
flvmux streamable=true ! 
rtmpsink location=rtmp://localhost:1935/camera sync=false
```

**CPU features:**
- **x264enc**: Software encoding
- **tune=zerolatency**: Optimized for minimum latency
- **speed-preset=ultrafast**: Maximum encoding speed

## Compilation and Installation

This component assumes you already have RoboComp core library installed according to the [official guide](https://github.com/robocomp/robocomp).

### 1. Compile the component

```bash
cd ~/robocomp/components/robocomp-robolab/components/hardware/camera/camera_streamer/
cmake .
make
```

### 2. Verify dependencies

```bash
# Check available camera
v4l2-ctl --list-devices

# Check GStreamer
gst-launch-1.0 --version

# Check NVIDIA encoder (if you have GPU)
gst-inspect-1.0 nvh264enc

# Check available ports
ss -tuln | grep -E '(8554|8889|8888|1935)'
```

## Configuration

### Configuration file: `etc/config`

```ini
# ========================================
# RoboComp ICE Configuration
# ========================================
CameraSimple.Endpoints=tcp -p 10195
CommonBehavior.Endpoints=tcp -p 10196

Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0

# ========================================
# Camera Configuration
# ========================================
[Camera]
# V4L2 camera device
Device=/dev/video1

# Capture resolution
Width=640
Height=480

# Target FPS
FPS=30

# ========================================
# Display Configuration
# ========================================
[Display]
# Show local preview window (0=No, 1=Yes)
Enabled=0

# ========================================
# Streaming Configuration
# ========================================
[Stream]
# Enable streaming (0=No, 1=Yes)
Enabled=1

# MediaMTX host
Host=localhost

# MediaMTX RTMP port
Port=1935

# Stream path
Path=camera

# ========================================
# Compute Period
# ========================================
[Period]
# Compute period in ms (33ms = 30fps approx)
Compute=33
```

### MediaMTX configuration: `mediamtx.yml`

```yaml
# Basic configuration - already included in repository
# See mediamtx.yml for complete details

paths:
  camera:
    # Publish from any source
    publishUser: ""
    publishPass: ""
    publishIPs: []
    
    # No authentication for reading
    readUser: ""
    readPass: ""
    readIPs: []
```

## System Usage

### Automatic startup (recommended)

The `start_system.sh` script manages the complete system lifecycle:

```bash
# Start everything (MediaMTX + CameraSimple)
./start_system.sh

# Stop everything
./start_system.sh --stop

# Restart
./start_system.sh --stop && ./start_system.sh
```

**The script automatically:**
- Detects platform (NVIDIA GPU, Jetson, CPU)
- Verifies dependencies (GStreamer, camera, ports)
- Starts MediaMTX in background
- Starts CameraSimple with correct configuration
- Shows access URLs and useful commands
- Saves PIDs for easy management

### Manual startup

If you prefer to start components separately:

```bash
# 1. Start MediaMTX
./mediamtx &
MEDIAMTX_PID=$!

# 2. Start CameraSimple
cd ~/robocomp/components/robocomp-robolab/components/hardware/camera/camera_streamer
bin/camera_streamer etc/config

# 3. Stop
kill $MEDIAMTX_PID
pkill -f camera_streamer
```

### Streaming verification

Use the included verification script:

```bash
./check_stream.sh
```

**Expected output:**
```
+---------------------------------------------------+
|     STREAMING VERIFICATION                        |
+---------------------------------------------------+

Checking MediaMTX... Active
Checking CameraSimple... Active
Checking ports... All open
Checking RTMP stream... Publishing
Testing RTSP stream... Valid stream
  Resolution: 640x480
  FPS: 30/1

+---------------------------------------------------+
|     STREAMING ACTIVE AND ACCESSIBLE               |
+---------------------------------------------------+
```

## Stream Access

### Access from local network

By default, MediaMTX listens on all interfaces (0.0.0.0), so the stream is accessible from any device on your local network without additional configuration. Simply use your machine's IP address:

```bash
# Find your IP address
ip addr show | grep "inet " | grep -v 127.0.0.1

# Access URLs (replace <YOUR_IP> with your actual IP)
http://<YOUR_IP>:8889/camera  # WebRTC
rtsp://<YOUR_IP>:8554/camera  # RTSP
http://<YOUR_IP>:8888/camera  # HLS
```

### Access from external networks (Internet)

To access the stream from outside your local network, you need to:

#### 1. Configure your firewall

Open the required ports in your system firewall:

```bash
# For Ubuntu/Debian with ufw
sudo ufw allow 8554/tcp comment 'MediaMTX RTSP'
sudo ufw allow 8889/tcp comment 'MediaMTX WebRTC HTTP'
sudo ufw allow 8888/tcp comment 'MediaMTX HLS'
sudo ufw allow 1935/tcp comment 'MediaMTX RTMP'
sudo ufw allow 8189/udp comment 'MediaMTX WebRTC ICE'

# For systems with firewalld
sudo firewall-cmd --permanent --add-port=8554/tcp
sudo firewall-cmd --permanent --add-port=8889/tcp
sudo firewall-cmd --permanent --add-port=8888/tcp
sudo firewall-cmd --permanent --add-port=1935/tcp
sudo firewall-cmd --permanent --add-port=8189/udp
sudo firewall-cmd --reload

# For systems with iptables
sudo iptables -A INPUT -p tcp --dport 8554 -j ACCEPT
sudo iptables -A INPUT -p tcp --dport 8889 -j ACCEPT
sudo iptables -A INPUT -p tcp --dport 8888 -j ACCEPT
sudo iptables -A INPUT -p tcp --dport 1935 -j ACCEPT
sudo iptables -A INPUT -p udp --dport 8189 -j ACCEPT
```

#### 2. Configure port forwarding in your router

Access your router's administration panel (usually http://192.168.1.1 or http://192.168.0.1) and configure port forwarding:

| Service | Protocol | External Port | Internal IP | Internal Port |
|---------|----------|---------------|-------------|---------------|
| RTSP    | TCP      | 8554          | <YOUR_PC_IP> | 8554          |
| WebRTC HTTP | TCP  | 8889          | <YOUR_PC_IP> | 8889          |
| HLS     | TCP      | 8888          | <YOUR_PC_IP> | 8888          |
| RTMP    | TCP      | 1935          | <YOUR_PC_IP> | 1935          |
| WebRTC ICE | UDP   | 8189          | <YOUR_PC_IP> | 8189          |

#### 3. Find your public IP address

```bash
curl ifconfig.me
# or
curl icanhazip.com
```

#### 4. Access from external network

Once configured, you can access the stream using your public IP:

```bash
# WebRTC (best for browser access)
http://<YOUR_PUBLIC_IP>:8889/camera

# RTSP (best for VLC/media players)
rtsp://<YOUR_PUBLIC_IP>:8554/camera

# HLS (browser alternative)
http://<YOUR_PUBLIC_IP>:8888/camera
```

### Port reference table

| Port | Protocol | Service | Purpose | Required for External Access |
|------|----------|---------|---------|-------------------------------|
| 8554 | TCP | RTSP | Video streaming via RTSP | Yes (if using RTSP) |
| 8889 | TCP | HTTP | WebRTC signaling | Yes (if using WebRTC) |
| 8888 | TCP | HTTP | HLS streaming | Yes (if using HLS) |
| 1935 | TCP | RTMP | Internal video input | No (only local) |
| 8189 | UDP | WebRTC ICE | WebRTC media transport | Yes (if using WebRTC) |
| 10195 | TCP | ICE | RoboComp CameraSimple interface | Only if other components need it |

### Security considerations

**Important:** Exposing streaming services to the Internet has security implications:

1. **No authentication by default**: MediaMTX is configured without authentication. Anyone with your IP can access the stream.

2. **Enable authentication** (recommended for public access):

Edit `mediamtx.yml`:
```yaml
paths:
  camera:
    # Require authentication for reading
    readUser: "your_username"
    readPass: "your_password"
    
    # URLs become:
    # rtsp://your_username:your_password@<IP>:8554/camera
```

3. **Use HTTPS/TLS** for production environments:
   - Configure MediaMTX with SSL certificates
   - Use a reverse proxy (nginx, Apache) with Let's Encrypt

4. **Alternative: Use VPN or SSH tunneling** instead of direct port exposure:
```bash
# SSH tunnel example (from remote machine)
ssh -L 8889:localhost:8889 user@<YOUR_PUBLIC_IP>
# Then access: http://localhost:8889/camera
```

5. **Monitor access logs**:
```bash
# Check who is accessing your stream
tail -f /tmp/mediamtx.log | grep "session"
```

### WebRTC (Browser) - **Recommended for low latency**

```bash
# Web browser
http://localhost:8889/camera
http://<LOCAL_IP>:8889/camera

# Example
xdg-open http://192.168.5.13:8889/camera
```

**Advantages:**
- Ultra low latency (<100ms)
- Works in modern browsers without plugins
- Mobile compatible

### RTSP (VLC/FFplay)

```bash
# VLC Media Player
vlc rtsp://localhost:8554/camera
vlc rtsp://<LOCAL_IP>:8554/camera

# FFplay (low latency)
ffplay -fflags nobuffer -flags low_delay rtsp://localhost:8554/camera

# GStreamer
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/camera latency=0 ! decodebin ! autovideosink
```

**Advantages:**
- Standard streaming protocol
- Compatible with most players
- Ideal for recording

### HLS (Browser/HTTP)

```bash
# Web browser
http://localhost:8888/camera
http://<LOCAL_IP>:8888/camera
```

**Advantages:**
- Works in any browser
- Easy distribution via HTTP
- Higher latency (3-10 seconds)

## Logs and Diagnostics

### View real-time logs

```bash
# MediaMTX
tail -f /tmp/mediamtx.log

# CameraSimple
tail -f /tmp/camerasimple.log
```

### Expected logs - MediaMTX

```
INF [RTMP] [conn 127.0.0.1:43906] is publishing to path 'camera', 1 track (H264)
INF [WebRTC] [session 17b7b968] is reading from path 'camera', 1 track (H264)
```

### Expected logs - CameraSimple

```
Opening camera: /dev/video1
Resolution: 640x480
FPS configured: 30
Platform detected: nvidia
GStreamer pipeline: fdsrc fd=0 ! rawvideoparse...
Streaming pipeline initialized successfully (PID: 12345)
[13:36:11] CameraSimple adapter created in port tcp -p 10195
```

### Problem diagnostics

#### Stream not accessible

```bash
# 1. Verify MediaMTX is receiving data
grep "is publishing" /tmp/mediamtx.log

# 2. Verify GStreamer pipeline
ps aux | grep gst-launch

# 3. Check ports
ss -tuln | grep -E '(8554|8889|8888|1935)'

# 4. Test stream with ffprobe
ffprobe -v quiet -print-format json -show_streams rtsp://localhost:8554/camera
```

#### Camera not detected

```bash
# List V4L2 devices
v4l2-ctl --list-devices

# View supported formats
v4l2-ctl -d /dev/video1 --list-formats-ext

# Test direct capture
ffplay -f v4l2 -i /dev/video1
```

#### GPU not available

```bash
# Check NVIDIA GPU
nvidia-smi

# Check GStreamer encoder
gst-inspect-1.0 nvh264enc

# Alternative: Force CPU encoding
# Component automatically falls back to x264enc
```

## Optimization and Performance

### For minimum latency

```ini
# etc/config
[Period]
Compute=16  # ~60fps processing

[Camera]
FPS=60
```

```yaml
# mediamtx.yml
paths:
  camera:
    # Disable recording (reduces overhead)
    recordPath: ""
```

### For maximum quality

```python
# In specificworker.py, modify bitrate:
# bitrate=10000  # 10 Mbps instead of 5 Mbps
```

### Reduce CPU usage

```ini
# etc/config
[Display]
Enabled=0  # Disable local preview

[Camera]
FPS=15  # Reduce FPS if real-time not needed
```

## Troubleshooting

### Error: "Could not open streaming pipeline"

**Cause:** OpenCV without GStreamer support or invalid pipeline

**Solution:** Component uses subprocess for GStreamer directly, doesn't require OpenCV with GStreamer

### Error: "no one is publishing to path 'camera'"

**Cause:** CameraSimple is not sending data to MediaMTX

**Solutions:**
1. Verify GStreamer process is active: `ps aux | grep gst-launch`
2. Check logs: `tail -f /tmp/camerasimple.log`
3. Restart: `./start_system.sh --stop && ./start_system.sh`

### Error: "Cannot open camera /dev/videoX"

**Cause:** Camera not available or insufficient permissions

**Solutions:**
```bash
# Check available cameras
ls -la /dev/video*

# Add permissions
sudo usermod -a -G video $USER
# Logout and login to apply changes

# Try another camera
v4l2-ctl --list-devices
# Edit etc/config with correct device
```

### High latency in WebRTC

**Solutions:**
1. Check local network (use ethernet cable instead of WiFi)
2. Reduce resolution/FPS
3. Verify no buffering: check MediaMTX logs

### High CPU usage

**Solutions:**
1. Verify GPU encoder is used: `grep "Platform detected" /tmp/camerasimple.log`
2. Reduce FPS or resolution
3. Disable local display: `[Display] Enabled=0`

## References

- **RoboComp**: https://github.com/robocomp/robocomp
- **MediaMTX**: https://github.com/bluenviron/mediamtx
- **GStreamer**: https://gstreamer.freedesktop.org/
- **CameraSimple Interface**: `robocomp/interfaces/IDSLs/CameraSimple.idsl`

## License

This component is part of RoboComp and is distributed under GNU General Public License v3.0.
