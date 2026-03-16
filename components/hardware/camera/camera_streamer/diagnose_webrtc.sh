#!/bin/bash

# Script de diagnóstico de conectividad WebRTC/WHEP
# Diagnostica problemas de latencia de conexión ICE y URLs

HOST=${1:-localhost}
PORT=${2:-8889}
PATH=${3:-camera}

echo "=========================================="
echo "  Diagnóstico de Conectividad WebRTC"
echo "=========================================="
echo ""
echo "Servidor: ${HOST}:${PORT}"
echo "Path: ${PATH}"
echo ""

WHEP_URL="http://${HOST}:${PORT}/${PATH}/whep"

# Test 1: OPTIONS
echo "1. Testing OPTIONS request..."
echo "   URL: ${WHEP_URL}"
echo ""

OPTIONS_RESPONSE=$(curl -v -X OPTIONS "${WHEP_URL}" 2>&1)
OPTIONS_CODE=$(echo "$OPTIONS_RESPONSE" | grep "< HTTP" | tail -1 | awk '{print $3}')

echo "   Response: ${OPTIONS_CODE}"

if [ "$OPTIONS_CODE" = "204" ] || [ "$OPTIONS_CODE" = "200" ]; then
    echo "   ✓ OPTIONS OK"

    # ICE Servers
    LINK=$(echo "$OPTIONS_RESPONSE" | grep -i "< Link:" | sed 's/.*< Link: //')
    if [ ! -z "$LINK" ]; then
        echo "   ICE Servers: $LINK"
    else
        echo "   ICE Servers: (none - usando default STUN)"
    fi
else
    echo "   ✗ OPTIONS failed"
    exit 1
fi

echo ""

# Test 2: POST con SDP simulado
echo "2. Testing POST request (simulated SDP)..."
echo ""

# SDP offer simple para testing
SDP_OFFER="v=0
o=- 0 0 IN IP4 127.0.0.1
s=-
t=0 0
m=video 9 UDP/TLS/RTP/SAVPF 96
c=IN IP4 0.0.0.0
a=ice-ufrag:test
a=ice-pwd:testpassword1234567890123456
a=fingerprint:sha-256 00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00
a=setup:actpass
a=mid:0
a=recvonly
a=rtcp-mux
a=rtpmap:96 VP8/90000"

POST_RESPONSE=$(curl -v -X POST "${WHEP_URL}" \
    -H "Content-Type: application/sdp" \
    --data "$SDP_OFFER" 2>&1)

POST_CODE=$(echo "$POST_RESPONSE" | grep "< HTTP" | tail -1 | awk '{print $3}')

echo "   Response: ${POST_CODE}"

if [ "$POST_CODE" = "201" ] || [ "$POST_CODE" = "200" ]; then
    echo "   ✓ POST OK (session created)"

    # Extract Location header
    LOCATION=$(echo "$POST_RESPONSE" | grep -i "< Location:" | sed 's/.*< Location: //' | tr -d '\r')

    if [ ! -z "$LOCATION" ]; then
        echo "   Location header: ${LOCATION}"

        # Analyze Location format
        if [[ "$LOCATION" == http* ]]; then
            echo "   Format: Absolute URL"
            SESSION_URL="$LOCATION"
        elif [[ "$LOCATION" == /* ]]; then
            echo "   Format: Absolute path"
            SESSION_URL="http://${HOST}:${PORT}${LOCATION}"
        else
            echo "   Format: Relative path"
            SESSION_URL="${WHEP_URL}/${LOCATION}"
        fi

        echo "   Constructed DELETE URL: ${SESSION_URL}"
        echo ""

        # Test 3: DELETE
        echo "3. Testing DELETE request..."
        sleep 1

        DELETE_RESPONSE=$(curl -v -X DELETE "${SESSION_URL}" 2>&1)
        DELETE_CODE=$(echo "$DELETE_RESPONSE" | grep "< HTTP" | tail -1 | awk '{print $3}')

        echo "   Response: ${DELETE_CODE}"

        if [ "$DELETE_CODE" = "200" ] || [ "$DELETE_CODE" = "204" ]; then
            echo "   ✓ DELETE OK (session closed)"
        else
            echo "   ✗ DELETE failed"
            echo "   This may cause the 404 error you're seeing"
        fi
    else
        echo "   ✗ No Location header received!"
        echo "   This will cause 404 on DELETE"
    fi
else
    echo "   ✗ POST failed"
    echo ""
    echo "Full response:"
    echo "$POST_RESPONSE" | grep "< "
fi

echo ""
echo "=========================================="
echo "  Diagnóstico de Latencia ICE"
echo "=========================================="
echo ""

# Check network configuration
echo "IPs del servidor:"
ip -4 addr show 2>/dev/null | grep "inet " | grep -v 127.0.0.1 | awk '{print "  - " $2}'

echo ""
echo "Puertos WebRTC:"
netstat -tuln 2>/dev/null | grep -E ":(8889|8189)" | while read line; do
    echo "  $line"
done

echo ""
echo "=========================================="
echo "  Problemas Detectados y Soluciones"
echo "=========================================="
echo ""

# Check for common issues
echo "Verificando problemas comunes..."
echo ""

# Issue 1: IPv6 binding
if netstat -tuln 2>/dev/null | grep 8189 | grep -q ":::" && ! netstat -tuln 2>/dev/null | grep 8189 | grep -q "0.0.0.0"; then
    echo "⚠ PROBLEMA: Puerto 8189 solo en IPv6"
    echo "   Solución: En mediamtx.yml cambiar:"
    echo "   webrtcLocalUDPAddress: 0.0.0.0:8189"
    echo ""
fi

# Issue 2: ICE gathering timeout
echo "💡 OPTIMIZACIÓN: Reducir latencia ICE"
echo "   El cliente actual espera ICE gathering complete"
echo "   Solución: Usar Trickle ICE (no esperar gathering)"
echo "   → Ya implementado en el viewer actualizado"
echo ""

# Issue 3: Multiple IPs
NUM_IPS=$(ip -4 addr show 2>/dev/null | grep "inet " | grep -v 127.0.0.1 | wc -l)
if [ "$NUM_IPS" -gt 2 ]; then
    echo "⚠ AVISO: Servidor tiene $NUM_IPS IPs"
    echo "   Esto puede causar problemas de ICE"
    echo "   Solución: En mediamtx.yml listar solo IPs LAN:"
    echo "   webrtcAdditionalHosts:"
    ip -4 addr show 2>/dev/null | grep "inet " | grep -v 127.0.0.1 | grep "192.168" | awk '{print "     - \""$2"\""}' | cut -d'/' -f1
    echo ""
fi

echo "=========================================="
echo "  Tiempos Esperados"
echo "=========================================="
echo ""
echo "Con la configuración optimizada:"
echo "  - OPTIONS: < 50ms"
echo "  - POST: < 100ms"
echo "  - ICE checking → connected: < 1 segundo (LAN)"
echo "  - Inicio de video: < 2 segundos total"
echo ""
echo "Si tarda más de 5 segundos:"
echo "  1. Verificar que el stream esté publicando"
echo "  2. Verificar firewall en cliente y servidor"
echo "  3. Verificar que no hay múltiples IPs confundiendo ICE"
echo "  4. Usar trickle ICE (ya implementado)"
echo ""

