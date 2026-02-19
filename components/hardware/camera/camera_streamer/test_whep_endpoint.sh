#!/bin/bash

# Test simple de endpoint WHEP
# Verifica que el servidor MediaMTX responde correctamente a peticiones WHEP

HOST=${1:-localhost}
PORT=${2:-8889}
PATH=${3:-camera}

WHEP_URL="http://${HOST}:${PORT}/${PATH}/whep"

echo "=========================================="
echo "  WHEP Endpoint Test"
echo "=========================================="
echo ""
echo "Testing URL: ${WHEP_URL}"
echo ""

# Test OPTIONS
echo "1. Testing OPTIONS (get ICE servers)..."
echo "   Request: OPTIONS ${WHEP_URL}"
echo ""

RESPONSE=$(curl -v -X OPTIONS "${WHEP_URL}" 2>&1)
HTTP_CODE=$(echo "$RESPONSE" | grep "< HTTP" | tail -1 | awk '{print $3}')

echo "   Response Code: ${HTTP_CODE}"

if [ "$HTTP_CODE" = "204" ] || [ "$HTTP_CODE" = "200" ]; then
    echo "   ✓ OPTIONS successful"

    # Extract CORS headers
    echo ""
    echo "   CORS Headers:"
    echo "$RESPONSE" | grep -i "< access-control" | sed 's/^/     /'

    # Extract Link header (ICE servers)
    echo ""
    echo "   ICE Servers:"
    LINK_HEADER=$(echo "$RESPONSE" | grep -i "< Link:" | sed 's/< Link: //')
    if [ ! -z "$LINK_HEADER" ]; then
        echo "     ${LINK_HEADER}"
    else
        echo "     (None configured - will use default STUN)"
    fi
else
    echo "   ✗ OPTIONS failed with code ${HTTP_CODE}"
    echo ""
    echo "   Full response:"
    echo "$RESPONSE"
    exit 1
fi

echo ""
echo "=========================================="
echo "  Endpoint is ready for WebRTC clients"
echo "=========================================="
echo ""
echo "To test from browser:"
echo "  1. Open http://${HOST}:${PORT}/viewer_webrtc.html"
echo "  2. Set Server: ${HOST}:${PORT}"
echo "  3. Set Path: ${PATH}"
echo "  4. Click Connect"
echo ""
echo "To verify session management:"
echo "  - Open browser console (F12)"
echo "  - Look for 'WHEP session URL' log"
echo "  - Disconnect and verify no 500 error"
echo ""

