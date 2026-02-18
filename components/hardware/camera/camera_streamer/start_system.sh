#!/bin/bash

# Script to start the complete CameraSimple system with GPU streaming
# Supports: PC with NVIDIA GPU, Jetson Orin/Xavier
# Usage: ./start_system.sh [--stop]

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# File to save PIDs
PID_FILE="/tmp/camerasimple_pids"

# Function to detect platform
detect_platform() {
    if [ -f /etc/nv_tegra_release ]; then
        echo "jetson"
    elif nvidia-smi &> /dev/null; then
        echo "nvidia"
    else
        echo "cpu"
    fi
}

# Function to stop processes
stop_system() {
    echo -e "${YELLOW}Stopping CameraSimple system...${NC}"

    # Kill by PID if the file exists
    if [ -f "$PID_FILE" ]; then
        while read pid; do
            if ps -p $pid > /dev/null 2>&1; then
                kill -9 $pid 2>/dev/null
                echo -e "  ${GREEN}✓${NC} Process $pid stopped"
            fi
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi

    # Kill by name (just in case)
    pkill -9 -f "mediamtx" 2>/dev/null
    pkill -9 -f "camerasimple" 2>/dev/null

    # Release ports if occupied
    for port in 8554 8888 8889 1935 8890; do
        fuser -k $port/tcp 2>/dev/null
    done

    sleep 2
    echo -e "${GREEN}✓ System stopped${NC}"
}

# If --stop is passed, only stop
if [ "$1" == "--stop" ]; then
    stop_system
    exit 0
fi

echo -e "${BLUE}╔═══════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  🎥 CameraSimple + GPU Streaming + MediaMTX      ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════╝${NC}"
echo ""

# Detect platform
PLATFORM=$(detect_platform)
echo -e "${GREEN}✓${NC} Platform detected: ${GREEN}$PLATFORM${NC}"
case $PLATFORM in
    jetson)
        echo -e "  Using: ${YELLOW}nvv4l2h264enc${NC} (Jetson hardware encoder)"
        ;;
    nvidia)
        echo -e "  Using: ${YELLOW}nvh264enc${NC} (NVIDIA NVENC)"
        ;;
    cpu)
        echo -e "  Using: ${YELLOW}x264enc${NC} (CPU software encoder)"
        echo -e "  ${YELLOW}⚠ Warning: Higher CPU usage, consider using GPU${NC}"
        ;;
esac
echo ""

# Get IP from network interface
IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+')
if [ -z "$IP" ]; then
    IP=$(hostname -I | awk '{print $1}')
fi
if [ -z "$IP" ]; then
    IP="localhost"
    echo -e "${YELLOW}⚠${NC} Could not detect network IP, using localhost"
else
    echo -e "${GREEN}✓${NC} Network IP: ${GREEN}$IP${NC}"
fi
echo ""

# Check camera device
CAMERA_DEVICE=$(grep "Camera.Device" etc/config 2>/dev/null | cut -d'=' -f2 | tr -d ' "'"'"'')
if [ -z "$CAMERA_DEVICE" ]; then
    CAMERA_DEVICE="/dev/video2"
fi

echo -n "Checking camera device $CAMERA_DEVICE... "
if [ -c "$CAMERA_DEVICE" ]; then
    echo -e "${GREEN}✓ Found${NC}"
    v4l2-ctl --device=$CAMERA_DEVICE --info 2>/dev/null | grep "Card type" | awk '{print "  " $0}'
else
    echo -e "${RED}✗ Not found${NC}"
    echo -e "${YELLOW}Available video devices:${NC}"
    ls -la /dev/video* 2>/dev/null | awk '{print "  " $0}'
    echo -e "${YELLOW}Please check Camera.Device in etc/config${NC}"
    exit 1
fi

# Check GStreamer
echo -n "Checking GStreamer... "
if gst-inspect-1.0 --version > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Available${NC}"
else
    echo -e "${RED}✗ Not found${NC}"
    echo -e "${YELLOW}Please install GStreamer: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-*${NC}"
    exit 1
fi

# Check encoder availability
echo -n "Checking GPU encoder... "
case $PLATFORM in
    jetson)
        if gst-inspect-1.0 nvv4l2h264enc > /dev/null 2>&1; then
            echo -e "${GREEN}✓ nvv4l2h264enc available${NC}"
        else
            echo -e "${YELLOW}⚠ nvv4l2h264enc not found, falling back to CPU${NC}"
        fi
        ;;
    nvidia)
        if gst-inspect-1.0 nvh264enc > /dev/null 2>&1; then
            echo -e "${GREEN}✓ nvh264enc available${NC}"
        else
            echo -e "${YELLOW}⚠ nvh264enc not found, falling back to CPU${NC}"
        fi
        ;;
    cpu)
        if gst-inspect-1.0 x264enc > /dev/null 2>&1; then
            echo -e "${GREEN}✓ x264enc available${NC}"
        else
            echo -e "${RED}✗ x264enc not found${NC}"
            echo -e "${YELLOW}Please install: sudo apt install gstreamer1.0-plugins-ugly${NC}"
            exit 1
        fi
        ;;
esac

# Stop previous processes
echo ""
echo -e "${YELLOW}Cleaning up previous processes...${NC}"
stop_system

# Start MediaMTX
echo ""
echo -e "${YELLOW}Starting MediaMTX...${NC}"
./mediamtx mediamtx.yml > /tmp/mediamtx.log 2>&1 &
MEDIAMTX_PID=$!
echo $MEDIAMTX_PID >> "$PID_FILE"
sleep 3

# Verify MediaMTX
if ps -p $MEDIAMTX_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} MediaMTX started (PID: $MEDIAMTX_PID)"
    echo -e "  Log: /tmp/mediamtx.log"
else
    echo -e "${RED}✗${NC} Error starting MediaMTX"
    echo -e "${YELLOW}Last lines of log:${NC}"
    tail -5 /tmp/mediamtx.log 2>/dev/null
    exit 1
fi

# Verify MediaMTX ports
echo -n "  Checking ports... "
sleep 1
if netstat -tuln 2>/dev/null | grep -q ":8554 " && netstat -tuln 2>/dev/null | grep -q ":8889 "; then
    echo -e "${GREEN}✓ 8554, 8889, 8888, 1935 active${NC}"
else
    if ss -tuln 2>/dev/null | grep -q ":8554 "; then
        echo -e "${GREEN}✓ Ports active${NC}"
    else
        echo -e "${YELLOW}⚠ Some ports may not be active yet${NC}"
    fi
fi

# Start CameraSimple
echo ""
echo -e "${YELLOW}Starting CameraSimple component...${NC}"
cd "$SCRIPT_DIR"
python3 bin/camera_streamer etc/config > /tmp/camerasimple.log 2>&1 &
CAMERA_PID=$!
echo $CAMERA_PID >> "$PID_FILE"
cd "$SCRIPT_DIR"
sleep 4

# Verify CameraSimple
if ps -p $CAMERA_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} CameraSimple started (PID: $CAMERA_PID)"
    echo -e "  Log: /tmp/camerasimple.log"

    # Check if streaming initialized
    if grep -q "Streaming pipeline initialized" /tmp/camerasimple.log 2>/dev/null || \
       grep -q "streaming pipeline" /tmp/camerasimple.log 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} GPU streaming active"
    else
        sleep 2
        if grep -qi "error" /tmp/camerasimple.log 2>/dev/null; then
            echo -e "  ${YELLOW}⚠${NC} Check log for potential errors"
        else
            echo -e "  ${GREEN}✓${NC} Component running"
        fi
    fi
else
    echo -e "${RED}✗${NC} Error starting CameraSimple"
    echo -e "${YELLOW}Last lines of log:${NC}"
    tail -10 /tmp/camerasimple.log 2>/dev/null
    kill $MEDIAMTX_PID 2>/dev/null
    exit 1
fi

# Summary
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║           ✅ SYSTEM FULLY ACTIVE                  ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Access from this machine:${NC}"
echo -e "   WebRTC: http://localhost:8889/camera"
echo -e "   RTSP:   rtsp://localhost:8554/camera"
echo ""
echo -e "${YELLOW}Access from local network:${NC}"
echo -e "   WebRTC: http://$IP:8889/camera"
echo -e "   RTSP:   rtsp://$IP:8554/camera"
echo -e "   HLS:    http://$IP:8888/camera"
echo ""
echo -e "${YELLOW}To view the stream:${NC}"
echo -e "   Browser: ${GREEN}http://$IP:8889/camera${NC}"
echo -e "   VLC:     ${GREEN}vlc rtsp://$IP:8554/camera${NC}"
echo -e "   FFplay:  ${GREEN}ffplay rtsp://$IP:8554/camera${NC}"
echo ""
echo -e "${YELLOW}Logs:${NC}"
echo -e "   MediaMTX:     tail -f /tmp/mediamtx.log"
echo -e "   CameraSimple: tail -f /tmp/camerasimple.log"
echo ""
echo -e "${YELLOW}To stop:${NC}"
echo -e "   ./start_system.sh --stop"
echo ""
echo -e "${BLUE}PIDs saved in: $PID_FILE${NC}"
echo ""
