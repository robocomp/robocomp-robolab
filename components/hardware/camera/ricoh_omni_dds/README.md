# ricoh_omni_dds - Ricoh Theta Z1 360° Camera Component

RoboComp component for Ricoh Theta Z1 omnidirectional camera with low-latency WebRTC/RTSP streaming support.

## Prerequisites

- **NVIDIA GPU required** for video decoding (without it, the image appears green)
- Ricoh Theta Z1 in Live Streaming mode (hold Mode button while powering on)
- Ubuntu 20.04+ / Linux with GStreamer 1.x

---

## Complete Installation

### 1. System Dependencies

```bash
sudo apt update
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libusb-1.0-0-dev \
    autopoint \
    gtk-doc-tools \
    libgtk-3-dev \
```

### 2. NVIDIA Decoding

#### On PC with NVIDIA GPU:
```bash
# Install NVIDIA Video Codec SDK (v12.0.16 or higher)
# Download from: https://developer.nvidia.com/nvidia-video-codec-sdk

# Compile GStreamer NVDEC plugin:
# Follow: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200
```

#### On Jetson (Xavier/Orin):
No additional installation required, comes integrated with JetPack.

### 3. Ricoh Theta Driver (libuvc-theta + gstthetauvc)

```bash
# Installation directory
mkdir -p ~/software && cd ~/software

# 1. Uninstall standard libuvc if installed
sudo apt remove libuvc-dev 2>/dev/null

# 2. Install libuvc-theta
git clone https://github.com/ricohapi/libuvc-theta.git
cd libuvc-theta
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~/software

# 3. Install gstthetauvc
git clone https://github.com/nickel110/gstthetauvc.git
cd gstthetauvc/thetauvc
make
```

### 4. Configure GStreamer Plugin Path

```bash
# On standard PC:
echo 'export GST_PLUGIN_PATH=$HOME/software/gstthetauvc/thetauvc:$GST_PLUGIN_PATH' >> ~/.bashrc

# On Jetson:
echo 'export GST_PLUGIN_PATH=$HOME/software/gstthetauvc/thetauvc:/usr/lib/aarch64-linux-gnu/gstreamer-1.0:$GST_PLUGIN_PATH' >> ~/.bashrc

source ~/.bashrc
```

### 5. Verify Installation

```bash
# Verify plugin
gst-inspect-1.0 thetauvcsrc

# Direct video test (PC with GPU):
gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! queue ! glimagesink sync=false

# Direct video test (Jetson):
gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! nvv4l2decoder ! nvvidconv ! queue ! glimagesink sync=false

# Video test (CPU, slower):
gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! decodebin ! queue ! autovideosink sync=false
```

### 6. Install MediaMTX (Streaming Server)

```bash
cd /path/to/ricoh_omni_dds
./install_mediamtx.sh

# Or manually:
wget https://github.com/bluenviron/mediamtx/releases/download/v1.5.1/mediamtx_v1.5.1_linux_amd64.tar.gz
tar -xzf mediamtx_v1.5.1_linux_amd64.tar.gz
chmod +x mediamtx
```

---

## Quick Start

### Option 1: Automated Script (Recommended)

```bash
cd /path/to/ricoh_omni_dds

# Start entire system (MediaMTX + ricoh_omni_dds)
./start_system.sh

# Stop the system
./start_system.sh --stop
```

### Option 2: Manual Start

```bash
cd /path/to/ricoh_omni_dds

# Terminal 1: Start MediaMTX server
./mediamtx mediamtx.yml

# Terminal 2: Start ricoh_omni_dds component
bin/ricoh_omni_dds etc/config
```

---

## Stream Access

Once the system is started:

| Protocol | Local URL | Local Network URL |
|----------|-----------|-------------------|
| **WebRTC** (low latency) | http://localhost:8889/theta | http://192.168.5.13:8889/theta |
| **RTSP** | rtsp://localhost:8554/theta | rtsp://192.168.5.13:8554/theta |
| **HLS** | http://localhost:8888/theta | http://192.168.5.13:8888/theta |

### Included Web Viewer

Open `viewer_lowlatency.html` in your browser to view the WebRTC stream with latency statistics.

### Play with VLC

```bash
vlc rtsp://192.168.5.13:8554/theta
```

---

## Processing Pipeline Architecture

### Video Capture Pipeline

The component uses GStreamer pipelines to capture and decode video from the Ricoh Theta Z1 camera:

#### Standard PC (NVIDIA GPU):
```
┌─────────────┐   ┌─────────┐   ┌──────────┐   ┌─────────────┐   ┌────────────┐   ┌─────────┐
│ thetauvcsrc │──▶│ h264parse│──▶│ nvh264sldec│──▶│ videoconvert│──▶│ video/x-raw│──▶│ appsink │
│  mode=2K    │   │          │   │  (NVIDIA) │   │  n-threads=4│   │ format=BGR │   │  OpenCV │
└─────────────┘   └─────────┘   └──────────┘   └─────────────┘   └────────────┘   └─────────┘
     USB 3.0          H.264          Hardware        Multi-thread      BGR format      Max FPS
   Live Stream      Parsing          Decoding         Conversion       Conversion      (drop old)
```

#### NVIDIA Jetson (Xavier/Orin):
```
┌─────────────┐   ┌─────────┐   ┌──────────────┐   ┌──────────┐   ┌─────────────┐   ┌─────────┐
│ thetauvcsrc │──▶│ h264parse│──▶│ nvv4l2decoder│──▶│ nvvidconv│──▶│ videoconvert│──▶│ appsink │
│  mode=2K    │   │          │   │   (Jetson)   │   │ (HW accel)│   │  n-threads=4│   │  OpenCV │
└─────────────┘   └─────────┘   └──────────────┘   └──────────┘   └─────────────┘   └─────────┘
     USB 3.0          H.264            Hardware        Hardware       Multi-thread      Max FPS
   Live Stream      Parsing            Decoding       Conversion      Conversion       (drop old)
```

**Pipeline Parameters:**
- `mode=2K`: 3840x1920 @ 30fps equirectangular format
- `max-buffers=1`: Keep only latest frame (minimum latency)
- `drop=true`: Discard old frames if processing is slow
- `sync=false`: No synchronization, maximum throughput
- `n-threads=4`: Parallel color conversion

### Streaming Pipeline (MediaMTX Integration)

When streaming is enabled, a parallel pipeline publishes frames to MediaMTX server:

```
┌──────────────┐   ┌─────────────┐   ┌──────────┐   ┌────────┐   ┌──────────┐   ┌─────────┐
│   OpenCV     │──▶│ videoconvert│──▶│  x264enc │──▶│ flvmux │──▶│ rtmpsink │──▶│ MediaMTX│
│   cv::Mat    │   │ to I420     │   │ (H.264)  │   │        │   │   RTMP   │   │  Server │
└──────────────┘   └─────────────┘   └──────────┘   └────────┘   └──────────┘   └─────────┘
    BGR frames         Color           Software      FLV format      RTMP to       Multi-proto
   from capture      Conversion        Encoding      Container      localhost:     streaming
                                    (ultrafast,                      1935/theta
                                    zerolatency)
                                    5000 kbps
```

### MediaMTX Multi-Protocol Distribution

MediaMTX receives the RTMP stream and redistributes it via multiple protocols:

```
                                    ┌─────────────────────────────────────────┐
                                    │          MediaMTX Server                │
┌──────────────┐                    │                                         │
│  ricoh_omni_dds   │                    │  ┌───────────────────────────────────┐ │
│  Component   │──RTMP:1935/theta──▶│  │         RTMP Ingest               │ │
└──────────────┘                    │  └───────────────────────────────────┘ │
                                    │                 │                       │
                                    │                 ▼                       │
                                    │  ┌───────────────────────────────────┐ │
                                    │  │      Internal Re-encoding         │ │
                                    │  └───────────────────────────────────┘ │
                                    │         │       │       │       │      │
                                    │    ┌────┘       │       │       └────┐ │
                                    │    ▼            ▼       ▼            ▼ │
                                    │ ┌──────┐   ┌──────┐ ┌─────┐    ┌─────┐│
                                    │ │ RTSP │   │WebRTC│ │ HLS │    │RTMP ││
                                    │ │:8554 │   │:8889 │ │:8888│    │:1935││
                                    │ └──────┘   └──────┘ └─────┘    └─────┘│
                                    └─────────────────────────────────────────┘
                                           │         │       │           │
                                           ▼         ▼       ▼           ▼
                                       VLC/FFmpeg  Browser  Safari   External
                                        Players    WebRTC   HLS      Clients
```

**Streaming Characteristics:**
- **Input**: RTMP @ 1935/theta (from component)
- **RTSP**: Low-latency, good for VLC/FFmpeg players
- **WebRTC**: Ultra low-latency (<500ms), browser-compatible
- **HLS**: High compatibility, higher latency (3-10s)
- **Bitrate**: 5000 kbps (configurable via pipeline)
- **Encoding**: H.264 baseline profile, ultrafast preset

### Complete System Data Flow

```
 Camera          Component              MediaMTX               Clients
┌──────┐       ┌──────────┐           ┌──────────┐         ┌──────────┐
│Ricoh │──USB─▶│ GStreamer│           │          │         │ Browser  │
│Theta │       │  Capture │           │          │◀─WebRTC─│  Viewer  │
│  Z1  │       │ Pipeline │           │          │         └──────────┘
└──────┘       │          │           │          │
               │    ↓     │           │          │         ┌──────────┐
               │ OpenCV   │──RTMP────▶│  Multi-  │◀─RTSP──│   VLC    │
               │ Mat BGR  │           │ Protocol │         │  Player  │
               │          │           │  Server  │         └──────────┘
               │    ↓     │           │          │
               │   ICE    │           │          │         ┌──────────┐
               │Interface │           │          │◀──HLS──│  Safari  │
               └──────────┘           └──────────┘         └──────────┘
                    ↓
              RoboComp Clients
            (CameraRGBSimple)
```

---

## Configuration

### Configuration File (`etc/config`)

```ini
# Streaming enabled
streaming = 1
stream_host = localhost
stream_port = 8554
stream_path = theta

# Local display (0=disabled, 1=enabled)
display = 0

# Temporal offset for synchronization
time_offset = 0

# Image compression on ICE interface
compressed = 0
```

### MediaMTX Configuration (`mediamtx.yml`)

```yaml
# Change IP for external access
webrtcAdditionalHosts: ['YOUR_LOCAL_IP']

# Ports (default)
rtspAddress: 0.0.0.0:8554
rtmpAddress: 0.0.0.0:1935
webrtcAddress: 0.0.0.0:8889
hlsAddress: 0.0.0.0:8888
```

---

## Firewall Configuration

To allow external access to the streaming server, open the required ports:

```bash
# Using UFW (Ubuntu):
sudo ufw allow 8554/tcp   # RTSP
sudo ufw allow 8889/tcp   # WebRTC HTTP
sudo ufw allow 8889/udp   # WebRTC UDP
sudo ufw allow 8888/tcp   # HLS
sudo ufw allow 1935/tcp   # RTMP
sudo ufw allow 8189/udp   # WebRTC local UDP
sudo ufw reload

# Using iptables:
sudo iptables -A INPUT -p tcp --dport 8554 -j ACCEPT   # RTSP
sudo iptables -A INPUT -p tcp --dport 8889 -j ACCEPT   # WebRTC HTTP
sudo iptables -A INPUT -p udp --dport 8889 -j ACCEPT   # WebRTC UDP
sudo iptables -A INPUT -p tcp --dport 8888 -j ACCEPT   # HLS
sudo iptables -A INPUT -p tcp --dport 1935 -j ACCEPT   # RTMP
sudo iptables -A INPUT -p udp --dport 8189 -j ACCEPT   # WebRTC local UDP

# Save iptables rules (Debian/Ubuntu):
sudo iptables-save | sudo tee /etc/iptables/rules.v4
```

### Port Summary

| Port | Protocol | Service |
|------|----------|---------|
| 8554 | TCP | RTSP streaming |
| 8889 | TCP/UDP | WebRTC signaling and media |
| 8888 | TCP | HLS streaming |
| 1935 | TCP | RTMP ingest |
| 8189 | UDP | WebRTC local UDP |

---

## Troubleshooting

### Green Image
- Verify NVIDIA GPU is being used for decoding
- Check NVIDIA drivers: `nvidia-smi`

### Camera Not Detected
```bash
# Check USB
lsusb | grep -i ricoh

# Verify camera is in Live Streaming mode
# (blue LED blinking)
```

### GStreamer Plugin Not Found
```bash
# Check GST_PLUGIN_PATH
echo $GST_PLUGIN_PATH
gst-inspect-1.0 thetauvcsrc
```

### Stream Not Connecting
```bash
# Check MediaMTX is running
ps aux | grep mediamtx

# Check ports
netstat -tuln | grep -E '8554|8889|1935'

# View logs
tail -f /tmp/mediamtx.log
tail -f /tmp/ricoh_omni_dds.log
```

---

## Component Files

| File | Description |
|------|-------------|
| `start_system.sh` | Script to start/stop the entire system |
| `mediamtx.yml` | MediaMTX server configuration |
| `viewer_lowlatency.html` | WebRTC viewer with statistics |
| `etc/config` | Component configuration |
| `bin/ricoh_omni_dds` | Component executable |

---

## License

MIT License - RoboComp / RoboLab
