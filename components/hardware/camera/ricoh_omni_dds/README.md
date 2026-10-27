# ricoh_omni_dds - Ricoh Theta Z1 360° Camera Component

RoboComp component for Ricoh Theta Z1 omnidirectional camera with low-latency WebRTC/RTSP streaming support.

## Prerequisites

- Ricoh Theta Z1 in Live Streaming mode (hold Mode button while powering on)
- Ubuntu 20.04+ / Linux with GStreamer 1.x
- NVIDIA GPU *optional* — `use_gpu = false` (CPU decode, `avdec_h264`) is the
  current default and the better-tested path on real hardware; `use_gpu = true`
  runs hardware decode in an isolated helper process (see
  [GPU Decode Architecture](#gpu-decode-architecture) below) and adds
  measurable latency (see [Latency: CPU vs GPU](#latency-cpu-vs-gpu)) — pick it
  for the CPU headroom, not for lower latency.

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

> **Known upstream bug**: the vendored `libuvc-theta` sources have shipped, at
> least at the time of writing, with a missing semicolon in
> `src/device.c` (`static char h264guid[]={...}` with no trailing `;`), which
> only surfaces as a build error if you ever enable `-DENABLE_UVC_DEBUGGING=ON`
> to debug a connection issue. Fix it (add the `;`) before rebuilding with
> debugging on. Separately: if `libuvc`/`libuvc-theta` was ever installed
> before on the machine, always re-run `sudo make install && sudo ldconfig`
> after any rebuild — a stale `/usr/local/lib/libuvc.so.0` that doesn't match
> the current source is a real, silent failure mode (see
> [Camera Won't Connect](#camera-wont-connect--diagnostic-checklist) below).

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

### Option 1: Single Command (Recommended)

The component manages MediaMTX itself when running against real hardware
(`simulator = false`): it launches `./mediamtx mediamtx.yml` as a child process
during `initialize()` and stops it on shutdown. No separate terminal or script
needed.

```bash
cd /path/to/ricoh_omni_dds
bin/ricoh_omni_dds etc/config_shadow_real.toml
```

If a MediaMTX instance is already running (e.g. started manually or left over
from a previous run), the component detects it and skips launching a new one.
Set `launch_mediamtx = false` in the config to always manage MediaMTX
separately instead.

### Option 2: Automated Script (legacy)

```bash
cd /path/to/ricoh_omni_dds

# Start entire system (MediaMTX + ricoh_omni_dds)
./start_system.sh

# Stop the system
./start_system.sh --stop
```

### Option 3: Manual Start

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

#### Standard PC, CPU decode (`use_gpu = false`, current default):
```
┌─────────────┐   ┌───────────────────┐   ┌─────────────┐   ┌────────────┐   ┌─────────┐
│ thetauvcsrc │──▶│    avdec_h264     │──▶│ videoconvert│──▶│ video/x-raw│──▶│ appsink │
│  mode=2K    │   │  max-threads=4    │   │  n-threads=4│   │ format=BGR │   │  OpenCV │
└─────────────┘   └───────────────────┘   └─────────────┘   └────────────┘   └─────────┘
     USB 3.0        Software decode,          Multi-thread      BGR format      Max FPS
   Live Stream      thread count pinned         Conversion       Conversion      (drop old)
```
No `h264parse`: `avdec_h264` accepts `thetauvcsrc`'s raw byte-stream directly.
`h264parse` was removed from this path deliberately — see
[Known Issues & Fixes](#known-issues--fixes-2026-08).
`max-threads`/`n-threads` are pinned (`decode_threads`/`n-threads=4` config,
not left at the GStreamer default of `0`=auto) because on a high-core-count
machine "one thread per core" alone can push CPU into the hundreds of percent
— see the same section.

#### Standard PC, GPU decode (`use_gpu = true`), isolated in a helper process:
```
 Helper process (gst-launch-1.0 --no-fault)              Main component process
┌─────────────┐ ┌─────────┐ ┌──────────┐ ┌───────────┐   ┌──────────┐ ┌────────────┐ ┌─────────┐
│ thetauvcsrc │▶│h264parse│▶│ nvh264dec│▶│ gldownload│──▶│ shmsink  │ │  shmsrc    │▶│ appsink │
│  mode=2K    │ │         │ │ (NVIDIA) │ │ (GPU→CPU) │   │ socket   │▶│ (discovered│ │  OpenCV │
└─────────────┘ └─────────┘ └──────────┘ └───────────┘   └──────────┘ │ caps, see  │ └─────────┘
                                                          shared memory│ below)     │
                                                                       └────────────┘
```
Why isolated: `h264parse` (needed for `nvh264dec`'s caps negotiation) carries
a reproducible upstream segfault — see
[Known Issues & Fixes](#known-issues--fixes-2026-08). Running it in its own
`gst-launch-1.0` process means a crash there only kills the helper, which the
component detects and respawns, instead of taking the whole component (ICE
adapters, DDS publisher) down with it. Full architecture, caveats and
required flags in [GPU Decode Architecture](#gpu-decode-architecture).

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
- `mode=2K`: 1920x960 @ 29.97fps (`30000/1001`) equirectangular format —
  confirmed from the negotiated caps on real hardware (shadow-hp,
  2026-08-13); despite the name, it is **not** 3840x1920.
- `max-buffers=1`: Keep only latest frame (minimum latency)
- `drop=true`: Discard old frames if processing is slow
- `sync=false`: No synchronization, maximum throughput
- `avdec_h264 max-threads` / `videoconvert n-threads`: pinned via
  `decode_threads` / `n-threads=4` — see
  [Known Issues & Fixes](#known-issues--fixes-2026-08); GStreamer's default
  (`0`=auto, one thread per CPU core) is *not* safe to leave as-is on a
  high-core-count machine.

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
`x264enc threads` is pinned via `encode_threads` (default `2`) for the same
reason as decode above — GStreamer's `0`=auto default spawns one encoder
thread per CPU core, and under `tune=zerolatency` more threads mostly just
adds latency rather than helping throughput at this bitrate/resolution.

This pipeline's `cv::VideoWriter::write()` calls have no way to report a push
failure back to this component (no exception, no return value on the
GStreamer backend) — see
[Known Issues & Fixes](#known-issues--fixes-2026-08) for what that caused and
how it's mitigated.

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

## GPU Decode Architecture

When `use_gpu = true`, decode does **not** happen inside this component's own
process. Instead:

1. `launch_gpu_helper()` spawns `gst-launch-1.0 --no-fault -v <gpu pipeline>`
   as a detached child process (same pattern as the MediaMTX auto-launch).
   The pipeline ends in `shmsink socket-path=<gpu_shm_socket>` instead of an
   `appsink` — GStreamer's real shared-memory IPC, not a custom protocol.
2. `discover_gpu_caps()` polls the helper's own `-v` log
   (`/tmp/ricoh_gpu_helper.log`) for the negotiated `video/x-raw` caps on its
   `shmsink` pad (format, width, height, framerate) and builds a matching
   `shmsrc socket-path=... ! video/x-raw,format=BGR,width=W,height=H,... !
   appsink` pipeline for this process to open with `cv::VideoCapture`.
   **Why**: `shmsrc` does not reliably relay the sender's caps over the shm
   protocol in this GStreamer version — with no capsfilter, `appsink` samples
   arrive with no caps at all; with a `format`-only filter, OpenCV's
   `gst_video_info_from_caps()` fails ("can't handle unknown layout") because
   width/height are required. The real resolution isn't fixed in config
   (depends on what the camera/decoder negotiate), so it has to be discovered
   at runtime rather than hardcoded.
3. `gpu_helper_alive()` (`kill(pid, 0)`) and `reconnect_camera()` detect a
   dead helper and respawn it, rediscovering caps each time.

**Why isolate it at all**: `nvh264dec` requires `h264parse` for caps
negotiation (`alignment=au`) — without it, GStreamer refuses to link the
pipeline. `h264parse` carries the segfault described in
[Known Issues & Fixes](#known-issues--fixes-2026-08), so this is the only way
to use GPU decode without a crash there taking the whole component down.

**Required flag — `--no-fault`**: `gst-launch-1.0` installs its *own* SIGSEGV
handler by default, which tries to exec `gdb` and then spins forever waiting
for a debugger to attach instead of dying. Without `--no-fault`, a crashed
helper stays "alive" as far as `kill(pid, 0)` is concerned, so it never gets
respawned and the stream hangs forever instead of self-healing. **Confirmed
live on shadow-hp, 2026-08-13** — this is not a theoretical concern, it
reproduced on the very first real segfault after the isolation was deployed.

**Bonus side effect**: the helper negotiates `nvh264dec`'s GL/CUDA context
correctly because it's a real GStreamer app pumping its own bus
(`gst-launch-1.0`), unlike `cv::VideoCapture`, which never answers the
`NEED_CONTEXT` bus messages `nvh264dec`/`gldownload` send and hangs
indefinitely. This incidentally fixes the GPU hang described in
[Step 5 of the troubleshooting checklist](#step-5--gpugl-pipeline-hangs-forever-nvh264dec--gldownload)
below, which used to require falling back to `use_gpu = false` on shadow-hp.

### Latency: CPU vs GPU

Measured with GStreamer's built-in `latency` tracer
(`GST_TRACERS=latency`) on standalone `thetauvcsrc → fakesink` pipelines
(shadow-hp, 2026-08-13, ~10s / 230-270 samples each). This isolates decode
latency only — it does **not** include the extra shared-memory hop between
the GPU helper and this component in the real, isolated architecture above,
so real end-to-end GPU-mode latency is somewhat higher than shown:

| | CPU (`avdec_h264`) | GPU (`nvh264dec`+`gldownload`) | Δ |
|---|---|---|---|
| Mean | 27.1 ms | 48.0 ms | **+20.9 ms** |
| Median (p50) | 27.4 ms | 45.1 ms | +17.7 ms |
| p95 | 41.6 ms | 66.2 ms | +24.6 ms |
| Min | 7.2 ms | 20.8 ms | +13.6 ms |
| Max | 69.5 ms | 154.1 ms | +84.6 ms |

GPU decode is **not** a latency win here — the `gldownload` GPU→CPU transfer
and NVDEC's internal queueing cost more than they save. Pick `use_gpu = true`
for CPU headroom (frees a core's worth of `avdec_h264` decode work), not for
responsiveness.

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

### Real Hardware Config (`etc/config_shadow_real.toml`)

Same as above, plus MediaMTX auto-launch settings and the ICE
endpoint/proxy for `Camera360RGB`. This is the config to use for a
standalone start with `bin/ricoh_omni_dds etc/config_shadow_real.toml`:

```toml
simulator = false

launch_mediamtx  = true            # component starts/stops MediaMTX itself
mediamtx_bin     = "./mediamtx"
mediamtx_config  = "mediamtx.yml"

[Proxies]
Camera360RGB = "camera360rgb:tcp -h localhost -p 10097"  # unused (simulator=false), but required by ConfigLoader

[Endpoints]
Camera360RGB = "tcp -p 10097"  # interface this component serves to its clients
```

### GPU / Thread-Count Parameters

All optional, all have safe defaults — only set these if you need to override
per machine (core count, GPU plugin build differ between shadow-hp and
p3bot; **the component's source is kept identical across machines, only
config differs** — see [Known Issues & Fixes](#known-issues--fixes-2026-08)):

```toml
use_gpu         = false      # true = isolated GPU helper (see GPU Decode Architecture)
gpu_decoder     = "nvh264dec"  # or "nvdec" — varies by nvcodec plugin build
gpu_downloader  = "gldownload"  # or "cudadownload" (no display needed)
gpu_shm_socket  = "/tmp/ricoh_omni_dds_gpu.sock"
gpu_shm_size    = 100000000    # ~100MB shm ring buffer

decode_threads  = 4          # avdec_h264 max-threads (CPU path)
encode_threads  = 2          # x264enc threads (RTMP streaming path)
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

## Known Issues & Fixes (2026-08)

Summary of a debugging round on shadow-hp covering intermittent segfaults and
CPU spikes on real hardware. Full write-up with kernel-log evidence, thread
counts, and root-cause traces:
`debug_ricoh_omni_dds.md` in the `SHADOW_UPDATE` notebook repo.

| Symptom | Root cause | Fix |
|---|---|---|
| Intermittent `segmentation fault`, whole component dies | Upstream bug in `GstBaseParse::gst_base_parse_set_latency` (`libgstbase-1.0`), reached via `h264parse`, triggered by `xhci_hcd` USB anomalies on this camera | **CPU path**: removed `h264parse` — `avdec_h264` accepts unparsed byte-stream directly. **GPU path**: `h264parse` is mandatory there (caps negotiation), so it's isolated in a separate `gst-launch-1.0` process instead — see [GPU Decode Architecture](#gpu-decode-architecture) |
| CPU spikes to ~300% | (1) `avdec_h264`/`x264enc` default to `0`=auto threads = one thread per CPU core; (2) `stream_writer` (RTMP) leaked a full duplicate encode pipeline + threads on every reconnect (no `.release()` before reopening, plus a `static` error counter that never reset) | Pinned `decode_threads`/`encode_threads`; defensive `.release()` before reopen; error counter moved to a per-session member; adaptive backoff (2s→30s) on repeated reconnects instead of a fixed 2s retry forever |
| GPU helper crashes but stream hangs forever instead of recovering | `gst-launch-1.0` installs its own SIGSEGV handler by default and spins waiting for `gdb` instead of dying — the process stays "alive" per `kill(pid,0)`, so nothing ever respawns it | Launch the helper with `--no-fault` |
| RTMP stream freezes on a stale/black frame after a network blip, never recovers | `cv::VideoWriter::write()` (GStreamer backend) has no way to report a push failure to application code — no exception, no return value. mediamtx logs the disconnect (`i/o timeout`/`EOF`) but this process never sees it | Unconditional periodic refresh (every 120s) of `stream_writer`, since there's no reliable failure signal to react to instead |
| `nvh264dec ! gldownload` hangs forever when opened via the component, but works fine standalone via `gst-launch-1.0` | `cv::VideoCapture`'s GStreamer backend never answers the `NEED_CONTEXT` bus messages `nvh264dec`/`gldownload` send for GL/CUDA context negotiation | Fixed as a side effect of the GPU subprocess isolation: the helper *is* a real `gst-launch-1.0` process pumping its own bus, so context negotiation works normally there |

**Standing rule**: the component's source must stay identical across
shadow-hp and p3bot — only `.toml` config may differ (core counts, GPU plugin
builds, and known-good `use_gpu` state per machine legitimately vary).

---

## Troubleshooting

### Camera Won't Connect — Diagnostic Checklist

If the component (or a plain `gst-launch-1.0` test) can't get an image out of
the Theta Z1, work through these steps **in order** — each one rules out a
real failure mode we've hit on actual robots, roughly from "cheapest to
check" to "most involved". Don't skip to GStreamer/GL debugging before
confirming steps 1–3; most "camera not detected" reports turn out to be one
of those.

| # | Symptom / check | Likely cause |
|---|---|---|
| 1 | `lsusb \| grep -i ricoh` shows nothing | USB cable/port/hub issue — not software. Try another port/cable first. |
| 2 | Camera enumerates but LED isn't blinking blue | Not in **Live Streaming mode** — see step 1 below |
| 3 | `lsusb -t` shows `Driver=uvcvideo` on the Theta's video interface | Kernel driver is holding the interface — see step 2 below |
| 4 | `gst-inspect-1.0 thetauvcsrc` fails / plugin not found | `GST_PLUGIN_PATH` not set — see "GStreamer Plugin Not Found" below |
| 5 | Plain `fakesink` pipeline (step 4 below) fails with `Found 1 Theta(s), but none available` even though 1–3 are fine | Stale/mismatched `libuvc-theta` install — see step 3 below |
| 6 | Plain `fakesink` pipeline **works**, but the component hangs forever at `Connecting to Camera360RGB (attempt 1/10)` (no error, no timeout) | Historical GPU/GL context issue — see step 5 below (resolved by the subprocess isolation, kept for reference) |
| 7 | `use_gpu = true`: video freezes permanently after running fine for a while; `/tmp/ricoh_gpu_helper.log` ends in `Caught SIGSEGV ... Spinning` | Helper hit the `h264parse` segfault but didn't actually die — see [Known Issues & Fixes](#known-issues--fixes-2026-08); confirm the helper is launched with `--no-fault` |
| 8 | Video preview looks fine, but the RTMP/WebRTC stream in mediamtx is frozen on a stale frame; `mediamtx.log` shows `i/o timeout` or `EOF` on the RTMP connection | `cv::VideoWriter::write()` push failures are invisible to this process — see [Known Issues & Fixes](#known-issues--fixes-2026-08); the periodic refresh (120s) should self-heal — if it doesn't, check `stream_writer` is actually being reached in `compute()` |

#### Step 1 — Confirm Live Streaming mode
The Theta Z1 must be woken into **Live Streaming mode**, not its default
still-photo mode: hold the **Mode** button while powering on until the LED
blinks **blue**. A solid or off LED, or any other color, means `thetauvcsrc`
will never see a usable stream even though the camera enumerates fine over
USB.

#### Step 2 — Check whether `uvcvideo` (kernel driver) is holding the interface
Linux's built-in `uvcvideo` driver auto-binds to the Theta's video interface
as soon as it's plugged in. `libuvc-theta` needs **exclusive** USB access to
that same interface, so if `uvcvideo` has it, userspace access silently fails
(commonly surfacing as `Found 1 Theta(s), but none available`, sometimes with
`usbfs: process ... did not claim interface 1 before use` in `dmesg`/
`journalctl -k`).

```bash
# Check driver binding per interface (look for the Theta's entry, idVendor 05ca)
lsusb -t

# or, more explicitly, per-interface:
lsusb -v -d 05ca:2715 2>/dev/null | grep -E "Interface Descriptor|bInterfaceClass|Driver"
```

If `uvcvideo` shows up on the video interface, install a udev rule that
unbinds it automatically whenever the camera is plugged in (this is a
**machine-specific system file**, not part of this repo — every new robot
needs it installed once):

```bash
# /etc/udev/rules.d/99-theta.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="05ca", MODE="0666"
SUBSYSTEM=="usb", DRIVER=="uvcvideo", ATTRS{idVendor}=="05ca", ATTRS{idProduct}=="2715", RUN+="/bin/sh -c 'echo $kernel > /sys/bus/usb/drivers/uvcvideo/unbind'"
```

```bash
sudo cp 99-theta.rules /etc/udev/rules.d/99-theta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
# then unplug/replug the camera and re-check with `lsusb -t` — the video
# interface should now show no driver (or a blank `Driver=` field) instead
# of uvcvideo.
```

#### Step 3 — Rule out a stale/mismatched `libuvc-theta` install
Even with the camera in Live Streaming mode and `uvcvideo` unbound, a
previously-installed `libuvc.so` that doesn't match the currently checked-out
`libuvc-theta` source will fail to open the device — same
`Found 1 Theta(s), but none available` symptom, but with a completely
different root cause than steps 1–2. This is easy to miss because
`sudo make install` succeeding doesn't tell you whether an *older* copy was
already loaded by something, or whether the source itself had the semicolon
bug mentioned in the install section above and silently built a stale binary.

Rebuild and reinstall cleanly:
```bash
cd ~/software/libuvc-theta/build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig
```

If it's still failing, rebuild once with debugging on to see exactly where
`uvc_open()` fails (remember to fix the semicolon bug first if you hit a
build error):
```bash
cmake -DENABLE_UVC_DEBUGGING=ON .. && make -j$(nproc) && sudo make install && sudo ldconfig
# then re-run the fakesink test from step 4 and read the verbose UVC_DEBUG
# output — a failure inside uvc_open(), before any USBDEVFS_CLAIMINTERFACE
# ioctl, points at the library/build rather than USB permissions.
# Turn debugging back OFF and reinstall once fixed — it's noisy in production.
```

#### Step 4 — Isolate camera+driver from GStreamer/GL with a plain pipeline
Before blaming the component, always confirm the camera works with a minimal
GStreamer pipeline that has **no GPU/GL dependency**:
```bash
gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! avdec_h264 ! queue ! videoconvert ! video/x-raw,format=BGR ! fakesink
```
If this reaches `PLAYING` and keeps running (check with `Ctrl+C` after a few
seconds — it shouldn't have exited on its own), the camera/USB/driver/library
chain is entirely healthy. Any remaining problem is downstream, in the
GPU/GL decode path or in the component itself — proceed to step 5.

#### Step 5 — GPU/GL pipeline hangs forever (`nvh264dec ! gldownload`)
**Resolved as of the GPU subprocess isolation** (see
[GPU Decode Architecture](#gpu-decode-architecture)): the GPU pipeline now
runs inside a standalone `gst-launch-1.0` helper process rather than through
`cv::VideoCapture`, and `gst-launch-1.0` *does* correctly answer the
`NEED_CONTEXT` bus messages that used to hang it. Left below for reference —
still relevant if you're debugging the helper pipeline directly (e.g. testing
`gpu_helper_pipeline` by hand with `gst-launch-1.0`) or something regresses.

Original symptom, with `use_gpu = true` (or `orin = true`), before isolation:
the pipeline includes `gldownload`, which needs a **real, GPU-backed X
display** to create its EGL/GLX context. Symptoms:
- The component logs `Connecting to Camera360RGB (attempt 1/10)` and then
  just sits there — no error, no retry, no timeout.
- `Xlib: extension "DRI2" missing on display ":N"` appears in the log.

This happens when the component is launched against a display that doesn't
expose DRI2 (a bare `Xtigervnc`/VNC virtual display without GPU passthrough
is the case we've hit — it accepts a `DISPLAY` but has no real GL driver
behind it), or when no `DISPLAY` is set at all in the launch environment
(e.g. a plain non-interactive `ssh` session, `systemd`, or `screen`/`tmux`
started before the graphical session existed).

Checks:
```bash
echo $DISPLAY                 # empty? that's very likely the problem
glxinfo -B 2>&1 | head -20     # look for "direct rendering: Yes" and a real GPU renderer string
```

Workarounds:
- Launch the component from a terminal inside a session backed by the real
  GPU-driven X server (the physical console's Xorg, not a headless VNC
  display), so `DISPLAY` points at something with working DRI2/GLX.
- If that's not available yet on this robot, fall back to the CPU decode
  path, which needs no GL context at all — set in the config:
  ```toml
  use_gpu = false
  orin = false
  ```
  This uses `avdec_h264` instead of `nvh264dec ! gldownload` (see step 4's
  pipeline) — slower, but a reliable way to get the component running while
  the GPU/GL environment is sorted out separately.

---

### Green Image
- Verify NVIDIA GPU is being used for decoding
- Check NVIDIA drivers: `nvidia-smi`

### `error: XDG_RUNTIME_DIR is invalid or not set in the environment`
The GL-based decode pipeline (`gldownload`, used on PC/NVIDIA) needs an EGL
runtime dir. Some launch contexts (systemd, `screen`/`tmux` without a full
login session) don't export `XDG_RUNTIME_DIR`. The component now falls back
to a private dir under `/tmp` automatically when it's unset, so this is
harmless — but you can also set it explicitly before launching:
```bash
export XDG_RUNTIME_DIR=/run/user/$(id -u)
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
| `start_system.sh` | Legacy script to start/stop MediaMTX + component separately |
| `mediamtx.yml` | MediaMTX server configuration |
| `viewer_lowlatency.html` | WebRTC viewer with statistics |
| `etc/config` | Component configuration (manual MediaMTX) |
| `etc/config_shadow_real.toml` | Real hardware config; component launches MediaMTX itself |
| `bin/ricoh_omni_dds` | Component executable |

---

## License

MIT License - RoboComp / RoboLab
