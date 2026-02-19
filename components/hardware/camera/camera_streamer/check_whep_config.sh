#!/bin/bash

# Script de verificación de configuración MediaMTX para WHEP
# Verifica que la configuración y el servidor estén listos para clientes remotos

echo "=========================================="
echo "  MediaMTX WHEP Configuration Checker"
echo "=========================================="
echo ""

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Función para verificar
check() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1"
        return 1
    fi
}

# 1. Verificar que mediamtx está corriendo
echo -e "${BLUE}1. Verificando servicio MediaMTX...${NC}"
if pgrep -x "mediamtx" > /dev/null; then
    check "MediaMTX está corriendo"
else
    echo -e "${RED}✗${NC} MediaMTX no está corriendo"
    echo "   Iniciar con: ./mediamtx mediamtx.yml"
    exit 1
fi
echo ""

# 2. Verificar puertos
echo -e "${BLUE}2. Verificando puertos...${NC}"
netstat -tuln 2>/dev/null | grep -q ":8889" && check "Puerto 8889 (WebRTC/HTTP) abierto" || echo -e "${RED}✗${NC} Puerto 8889 no está escuchando"
netstat -tuln 2>/dev/null | grep -q ":8189" && check "Puerto 8189 (WebRTC/ICE) abierto" || echo -e "${YELLOW}⚠${NC} Puerto 8189 no está escuchando"
echo ""

# 3. Verificar archivo de configuración
echo -e "${BLUE}3. Verificando mediamtx.yml...${NC}"
if [ -f "mediamtx.yml" ]; then
    check "Archivo mediamtx.yml existe"

    # Verificar configuraciones clave
    grep -q "webrtc: yes" mediamtx.yml && check "WebRTC habilitado" || echo -e "${RED}✗${NC} WebRTC no habilitado"
    grep -q "webrtcAddress: 0.0.0.0:8889" mediamtx.yml && check "WebRTC escuchando en 0.0.0.0:8889" || echo -e "${YELLOW}⚠${NC} WebRTC address no configurado correctamente"
    grep -q "webrtcIPsFromInterfaces: yes" mediamtx.yml && check "webrtcIPsFromInterfaces habilitado" || echo -e "${YELLOW}⚠${NC} webrtcIPsFromInterfaces no habilitado"
    grep -q "webrtcAdditionalHosts" mediamtx.yml && check "webrtcAdditionalHosts configurado" || echo -e "${YELLOW}⚠${NC} webrtcAdditionalHosts no configurado"
else
    echo -e "${RED}✗${NC} Archivo mediamtx.yml no encontrado"
    exit 1
fi
echo ""

# 4. Verificar endpoint WHEP
echo -e "${BLUE}4. Verificando endpoint WHEP...${NC}"
HOST=${1:-localhost}
WHEP_URL="http://${HOST}:8889/camera/whep"

echo "   Probando: ${WHEP_URL}"
RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" -X OPTIONS "${WHEP_URL}" 2>/dev/null)

if [ "$RESPONSE" = "204" ] || [ "$RESPONSE" = "200" ]; then
    check "Endpoint WHEP responde (HTTP ${RESPONSE})"

    # Verificar headers CORS
    CORS=$(curl -s -I -X OPTIONS "${WHEP_URL}" 2>/dev/null | grep -i "access-control-allow-origin")
    if [ ! -z "$CORS" ]; then
        check "CORS configurado correctamente"
    else
        echo -e "${YELLOW}⚠${NC} Headers CORS no detectados"
    fi
else
    echo -e "${RED}✗${NC} Endpoint WHEP no responde (HTTP ${RESPONSE})"
    echo "   Verificar que el stream 'camera' esté publicando"
fi
echo ""

# 5. Verificar configuración de red
echo -e "${BLUE}5. Información de red...${NC}"
echo "   IPs locales:"
ip -4 addr show | grep inet | grep -v 127.0.0.1 | awk '{print "     - " $2}' | cut -d'/' -f1
echo ""

# 6. Recomendaciones
echo -e "${BLUE}6. Recomendaciones para clientes remotos:${NC}"
echo ""
echo "   Para que clientes remotos se conecten correctamente:"
echo "   1. Asegúrate de que el firewall permite:"
echo "      - TCP 8889 (WHEP endpoint)"
echo "      - UDP 8189 (WebRTC data)"
echo ""
echo "   2. Si el servidor está detrás de NAT, configurar en mediamtx.yml:"
echo "      webrtcICEHostNAT1To1IPs: ['<IP_PUBLICA>']"
echo ""
echo "   3. Usar el viewer actualizado que maneja sesiones WHEP correctamente"
echo ""

# 7. Test de stream (opcional)
echo -e "${BLUE}7. Verificando streams activos...${NC}"
STREAMS=$(curl -s "http://${HOST}:8889/v3/paths/list" 2>/dev/null | grep -o '"name":"[^"]*"' | cut -d'"' -f4)
if [ ! -z "$STREAMS" ]; then
    echo "   Streams disponibles:"
    echo "$STREAMS" | while read stream; do
        echo "     - ${stream}"
    done
else
    echo -e "${YELLOW}⚠${NC} No se detectaron streams activos"
    echo "   Iniciar stream con: python3 src/camerasimple.py etc/config"
fi
echo ""

echo "=========================================="
echo "  Verificación completada"
echo "=========================================="
echo ""
echo "Para probar desde un cliente remoto:"
echo "  1. Abrir http://${HOST}:8889/viewer_webrtc.html"
echo "  2. Configurar Server: ${HOST}:8889"
echo "  3. Path: camera"
echo "  4. Click Connect"
echo ""
echo "Ver logs de MediaMTX en tiempo real:"
echo "  tail -f /path/to/mediamtx.log"
echo ""

