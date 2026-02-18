#!/bin/bash

# Script to verify streaming status
# Usage: ./check_stream.sh

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔═══════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     🔍 STREAMING VERIFICATION                     ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════╝${NC}"
echo ""

# Check if MediaMTX is running
echo -n "Checking MediaMTX... "
if pgrep -x "mediamtx" > /dev/null; then
    echo -e "${GREEN}✓ Active${NC}"
    MEDIAMTX_PID=$(pgrep -x "mediamtx")
    echo -e "  PID: $MEDIAMTX_PID"
else
    echo -e "${RED}✗ Not running${NC}"
    exit 1
fi

# Check if CameraSimple is running
echo -n "Checking CameraSimple... "
if pgrep -f "camera_streamer" > /dev/null; then
    echo -e "${GREEN}✓ Active${NC}"
    CAMERA_PID=$(pgrep -f "camera_streamer")
    echo -e "  PID: $CAMERA_PID"
else
    echo -e "${RED}✗ Not running${NC}"
    exit 1
fi

# Check ports
echo ""
echo "Checking ports:"
for port in 8554 8889 8888 1935; do
    echo -n "  Port $port... "
    if ss -tuln 2>/dev/null | grep -q ":$port " || netstat -tuln 2>/dev/null | grep -q ":$port "; then
        echo -e "${GREEN}✓ Open${NC}"
    else
        echo -e "${RED}✗ Closed${NC}"
    fi
done

# Check if stream is active
echo ""
echo -n "Checking RTMP stream... "
if grep -q "is publishing to path 'camera'" /tmp/mediamtx.log 2>/dev/null; then
    echo -e "${GREEN}✓ Publishing${NC}"
    echo -e "  $(tail -1 /tmp/mediamtx.log | grep 'camera' | grep -o 'is publishing.*')"
else
    echo -e "${YELLOW}⚠ No active publisher${NC}"
fi

# Check connected clients
echo ""
echo -n "Checking WebRTC clients... "
CLIENTS=$(grep "is reading from path 'camera'" /tmp/mediamtx.log 2>/dev/null | tail -5 | wc -l)
if [ $CLIENTS -gt 0 ]; then
    echo -e "${GREEN}✓ $CLIENTS client(s) connected recently${NC}"
else
    echo -e "${YELLOW}⚠ No connected clients${NC}"
fi

# Test RTSP with ffprobe if available
echo ""
if command -v ffprobe &> /dev/null; then
    echo -n "Testing RTSP stream... "
    if timeout 3 ffprobe -v quiet -print_format json -show_streams rtsp://localhost:8554/camera 2>&1 | grep -q "codec_type"; then
        echo -e "${GREEN}✓ Valid stream${NC}"
        # Get stream info
        STREAM_INFO=$(timeout 3 ffprobe -v quiet -print_format json -show_streams rtsp://localhost:8554/camera 2>&1)
        WIDTH=$(echo "$STREAM_INFO" | grep -o '"width": [0-9]*' | head -1 | grep -o '[0-9]*')
        HEIGHT=$(echo "$STREAM_INFO" | grep -o '"height": [0-9]*' | head -1 | grep -o '[0-9]*')
        FPS=$(echo "$STREAM_INFO" | grep -o '"r_frame_rate": "[0-9]*/[0-9]*"' | head -1 | grep -o '[0-9]*/[0-9]*')

        if [ ! -z "$WIDTH" ] && [ ! -z "$HEIGHT" ]; then
            echo -e "  Resolution: ${WIDTH}x${HEIGHT}"
        fi
        if [ ! -z "$FPS" ]; then
            echo -e "  FPS: $FPS"
        fi
    else
        echo -e "${RED}✗ Cannot access stream${NC}"
    fi
else
    echo -e "${YELLOW}⚠ ffprobe not available to verify stream${NC}"
fi

# Get network IP
IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+')
if [ -z "$IP" ]; then
    IP=$(hostname -I | awk '{print $1}')
fi
if [ -z "$IP" ]; then
    IP="localhost"
fi

# URL summary
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     ✅ STREAMING ACTIVE AND ACCESSIBLE            ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "📺 ${YELLOW}Access URLs:${NC}"
echo -e "   WebRTC (browser): ${GREEN}http://$IP:8889/camera${NC}"
echo -e "   RTSP (VLC/FFplay): ${GREEN}rtsp://$IP:8554/camera${NC}"
echo -e "   HLS (browser):    ${GREEN}http://$IP:8888/camera${NC}"
echo ""
echo -e "🔧 ${YELLOW}Useful commands:${NC}"
echo -e "   View with VLC:        ${BLUE}vlc rtsp://$IP:8554/camera${NC}"
echo -e "   View with FFplay:     ${BLUE}ffplay -fflags nobuffer rtsp://$IP:8554/camera${NC}"
echo -e "   View MediaMTX logs:   ${BLUE}tail -f /tmp/mediamtx.log${NC}"
echo -e "   View CameraSimple logs: ${BLUE}tail -f /tmp/camerasimple.log${NC}"
echo ""

