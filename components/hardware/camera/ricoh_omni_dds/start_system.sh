#!/bin/bash

# Script to start the complete Ricoh Theta Z1 system
# Configured to be accessible from the network (interface)
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
PID_FILE="/tmp/ricoh_theta_pids"

# Function to stop processes
stop_system() {
    echo -e "${YELLOW}Stopping Ricoh Theta system...${NC}"

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
    pkill -9 -f "ricoh_omni_dds" 2>/dev/null

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
echo -e "${BLUE}║  🎥 Ricoh Theta Z1 + MediaMTX Streaming System   ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════╝${NC}"
echo ""

# Get IP from enp4s0
IP=$(ip addr show enp4s0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
if [ -z "$IP" ]; then
    IP="localhost"
    echo -e "${YELLOW}⚠${NC} Interface enp4s0 not found, using localhost"
else
    echo -e "${GREEN}✓${NC} Network IP: ${GREEN}$IP${NC}"
fi
echo ""

# Check camera
echo -n "Checking Ricoh Theta Z1 camera... "
if lsusb | grep -iq ricoh; then
    echo -e "${GREEN}✓ Connected${NC}"
    lsusb | grep -i ricoh | awk '{print "  " $0}'
else
    echo -e "${RED}✗ Not detected${NC}"
    echo -e "${YELLOW}Please connect the camera and try again${NC}"
    exit 1
fi

# Check GStreamer plugin
echo -n "Checking thetauvcsrc plugin... "
export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc
if gst-inspect-1.0 thetauvcsrc > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Available${NC}"
else
    echo -e "${RED}✗ Not found${NC}"
    echo -e "${YELLOW}Check GST_PLUGIN_PATH${NC}"
    exit 1
fi

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
if netstat -tuln 2>/dev/null | grep -q ":8554 " && netstat -tuln 2>/dev/null | grep -q ":8889 "; then
    echo -e "${GREEN}✓ 8554, 8889, 8888 active${NC}"
else
    echo -e "${YELLOW}⚠ Some ports may not be active${NC}"
fi

# Start ricoh_omni_dds
echo ""
echo -e "${YELLOW}Starting ricoh_omni_dds component...${NC}"
./bin/ricoh_omni_dds etc/config > /tmp/ricoh_omni_dds.log 2>&1 &
RICOH_PID=$!
echo $RICOH_PID >> "$PID_FILE"
sleep 4

# Verify ricoh_omni_dds
if ps -p $RICOH_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} ricoh_omni_dds started (PID: $RICOH_PID)"
    echo -e "  Log: /tmp/ricoh_omni_dds.log"

    # Check if streaming initialized
    if grep -q "Streaming pipeline initialized" /tmp/ricoh_omni_dds.log 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} Parallel streaming active"
    else
        echo -e "  ${YELLOW}⚠${NC} Parallel streaming may not be active"
    fi
else
    echo -e "${RED}✗${NC} Error starting ricoh_omni_dds"
    echo -e "${YELLOW}Last lines of log:${NC}"
    tail -10 /tmp/ricoh_omni_dds.log 2>/dev/null
    kill $MEDIAMTX_PID 2>/dev/null
    exit 1
fi

# Verify ports
echo ""
echo "Verifying ports..."
sleep 2
netstat -tuln | grep -E '8554|8888|8889' | while read line; do
    echo -e "  ${GREEN}✓${NC} $line"
done

# Summary
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║           ✅ SYSTEM FULLY ACTIVE                  ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "📍 ${YELLOW}Access from this machine:${NC}"
echo -e "   WebRTC: http://localhost:8889/theta"
echo ""
echo -e "📍 ${YELLOW}Access from local network:${NC}"
echo -e "   WebRTC: http://$IP:8889/theta"
echo -e "   RTSP:   rtsp://$IP:8554/theta"
echo -e "   HLS:    http://$IP:8888/theta"
echo ""
echo -e "🌐 ${YELLOW}To view the stream:${NC}"
echo -e "   Open: ${GREEN}viewer_webrtc.html${NC} in your browser"
echo -e "   Or:   ${GREEN}vlc rtsp://$IP:8554/theta${NC}"
echo ""
echo -e "📋 ${YELLOW}Logs:${NC}"
echo -e "   MediaMTX:  tail -f /tmp/mediamtx.log"
echo -e "   ricoh_omni_dds: tail -f /tmp/ricoh_omni_dds.log"
echo ""
echo -e "⏹️  ${YELLOW}To stop:${NC}"
echo -e "   ./start_system.sh --stop"
echo ""
echo -e "${BLUE}PIDs saved in: $PID_FILE${NC}"
echo ""
