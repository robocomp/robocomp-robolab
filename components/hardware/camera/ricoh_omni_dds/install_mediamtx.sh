#!/bin/bash

# Script de instalación de MediaMTX para streaming WebRTC
# MediaMTX es un servidor de streaming ligero y potente

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${BOLD}${BLUE}========================================${NC}"
echo -e "${BOLD}${BLUE}  Instalación de MediaMTX              ${NC}"
echo -e "${BOLD}${BLUE}========================================${NC}"
echo ""

# Detectar arquitectura
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    PLATFORM="linux_amd64"
elif [ "$ARCH" = "aarch64" ]; then
    PLATFORM="linux_arm64v8"
else
    echo -e "${RED}Arquitectura no soportada: $ARCH${NC}"
    exit 1
fi

echo -e "${YELLOW}Sistema detectado:${NC}"
echo "  Arquitectura: $ARCH"
echo "  Plataforma: $PLATFORM"
echo ""

# Versión de MediaMTX
VERSION="v1.5.1"
DOWNLOAD_URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}/mediamtx_${VERSION}_${PLATFORM}.tar.gz"
TEMP_FILE="/tmp/mediamtx.tar.gz"

echo -e "${BLUE}Descargando MediaMTX ${VERSION}...${NC}"
echo "  URL: $DOWNLOAD_URL"
echo ""

if ! wget -q --show-progress "$DOWNLOAD_URL" -O "$TEMP_FILE"; then
    echo -e "${RED}Error al descargar MediaMTX${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ Descarga completada${NC}"
echo ""

# Crear directorio temporal para extracción
EXTRACT_DIR="/tmp/mediamtx_install"
rm -rf "$EXTRACT_DIR"
mkdir -p "$EXTRACT_DIR"

echo -e "${BLUE}Extrayendo archivos...${NC}"
if ! tar -xzf "$TEMP_FILE" -C "$EXTRACT_DIR"; then
    echo -e "${RED}Error al extraer archivos${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Archivos extraídos${NC}"
echo ""

# Ofrecer opciones de instalación
echo -e "${YELLOW}Selecciona el tipo de instalación:${NC}"
echo "  1) Sistema (requiere sudo) - /usr/local/bin/mediamtx"
echo "  2) Local - ./mediamtx (en directorio actual)"
echo ""
read -p "Opción [2]: " INSTALL_OPTION
INSTALL_OPTION=${INSTALL_OPTION:-2}

if [ "$INSTALL_OPTION" = "1" ]; then
    echo ""
    echo -e "${BLUE}Instalando en /usr/local/bin...${NC}"

    if sudo mv "$EXTRACT_DIR/mediamtx" /usr/local/bin/mediamtx; then
        sudo chmod +x /usr/local/bin/mediamtx
        echo -e "${GREEN}✓ MediaMTX instalado en /usr/local/bin/mediamtx${NC}"
        MEDIAMTX_PATH="/usr/local/bin/mediamtx"
    else
        echo -e "${RED}Error al instalar en /usr/local/bin${NC}"
        exit 1
    fi
else
    echo ""
    echo -e "${BLUE}Instalando localmente...${NC}"

    cd "$(dirname "$0")"
    if mv "$EXTRACT_DIR/mediamtx" ./mediamtx; then
        chmod +x ./mediamtx
        echo -e "${GREEN}✓ MediaMTX instalado en $(pwd)/mediamtx${NC}"
        MEDIAMTX_PATH="./mediamtx"
    else
        echo -e "${RED}Error al instalar localmente${NC}"
        exit 1
    fi
fi

# Limpiar archivos temporales
rm -f "$TEMP_FILE"
rm -rf "$EXTRACT_DIR"

echo ""
echo -e "${BOLD}${GREEN}========================================${NC}"
echo -e "${BOLD}${GREEN}  Instalación Completada               ${NC}"
echo -e "${BOLD}${GREEN}========================================${NC}"
echo ""

# Verificar instalación
echo -e "${YELLOW}Verificando instalación...${NC}"
if $MEDIAMTX_PATH --version > /dev/null 2>&1; then
    VERSION_INFO=$($MEDIAMTX_PATH --version 2>&1 | head -1)
    echo -e "${GREEN}✓ MediaMTX instalado correctamente${NC}"
    echo "  $VERSION_INFO"
else
    echo -e "${RED}✗ Error al verificar instalación${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}Información de uso:${NC}"
echo ""
echo "  Para iniciar MediaMTX:"
if [ "$INSTALL_OPTION" = "1" ]; then
    echo "    mediamtx mediamtx.yml"
else
    echo "    ./mediamtx mediamtx.yml"
fi
echo ""
echo "  Para streaming WebRTC con Ricoh Theta Z1:"
echo "    ./start_webrtc_stream.sh"
echo ""
echo -e "${GREEN}Configuración lista para usar!${NC}"
