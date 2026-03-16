#!/bin/bash

# Script para reiniciar MediaMTX de forma segura

cd "$(dirname "$0")"

echo "=========================================="
echo "  Reiniciando MediaMTX"
echo "=========================================="
echo ""

# 1. Verificar si está corriendo
if pgrep -x mediamtx > /dev/null; then
    echo "✓ MediaMTX está corriendo, deteniendo..."
    pkill mediamtx
    sleep 2

    # Verificar que se detuvo
    if pgrep -x mediamtx > /dev/null; then
        echo "⚠ MediaMTX no se detuvo, usando kill -9..."
        pkill -9 mediamtx
        sleep 1
    fi
    echo "✓ MediaMTX detenido"
else
    echo "⚠ MediaMTX no estaba corriendo"
fi

echo ""

# 2. Verificar que el archivo de configuración existe
if [ ! -f "mediamtx.yml" ]; then
    echo "✗ Error: mediamtx.yml no encontrado"
    exit 1
fi

echo "✓ Configuración encontrada: mediamtx.yml"
echo ""

# 3. Mostrar cambios clave de configuración
echo "Configuración WebRTC:"
grep -A 2 "webrtcIPsFromInterfaces" mediamtx.yml 2>/dev/null || echo "  (no encontrado)"
grep "webrtcAdditionalHosts" mediamtx.yml 2>/dev/null || echo "  (no encontrado)"
grep "webrtcLocalUDPAddress" mediamtx.yml 2>/dev/null || echo "  (no encontrado)"
echo ""

# 4. Iniciar MediaMTX
echo "Iniciando MediaMTX..."
nohup ./mediamtx mediamtx.yml > /tmp/mediamtx.log 2>&1 &
MEDIAMTX_PID=$!

sleep 2

# 5. Verificar que arrancó
if pgrep -x mediamtx > /dev/null; then
    echo "✓ MediaMTX iniciado correctamente (PID: $(pgrep -x mediamtx))"
    echo ""

    # Verificar puertos
    echo "Puertos escuchando:"
    netstat -tuln 2>/dev/null | grep -E ":(8889|8189)" | head -4
    echo ""

    # Test rápido
    echo "Test de endpoint WHEP:"
    sleep 1
    RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" -X OPTIONS "http://localhost:8889/camera/whep" 2>/dev/null)
    if [ "$RESPONSE" = "204" ] || [ "$RESPONSE" = "200" ]; then
        echo "✓ Endpoint WHEP responde correctamente (HTTP $RESPONSE)"
    else
        echo "⚠ Endpoint WHEP no responde como esperado (HTTP $RESPONSE)"
        echo "  Verificar logs: tail -f /tmp/mediamtx.log"
    fi

    echo ""
    echo "=========================================="
    echo "  MediaMTX reiniciado exitosamente"
    echo "=========================================="
    echo ""
    echo "Logs: tail -f /tmp/mediamtx.log"
    echo "Viewer: http://localhost:8889/viewer_webrtc.html"
    echo ""
else
    echo "✗ Error: MediaMTX no pudo iniciar"
    echo ""
    echo "Últimas líneas del log:"
    tail -20 /tmp/mediamtx.log 2>/dev/null || echo "(log no disponible)"
    exit 1
fi

