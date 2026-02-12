#!/bin/bash

# Script de instalación para drivers de Ricoh Theta Z1
# Para usar con RicohOmni component

set -e  # Salir si hay algún error

INSTALL_DIR="/home/robocomp/software"
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}======================================${NC}"
echo -e "${YELLOW}Instalación de Drivers Ricoh Theta Z1${NC}"
echo -e "${YELLOW}======================================${NC}"
echo ""

# Verificar que existe el directorio de software
if [ ! -d "$INSTALL_DIR" ]; then
    echo -e "${RED}Error: No existe el directorio $INSTALL_DIR${NC}"
    exit 1
fi

cd "$INSTALL_DIR"

# Paso 1: Instalar libuvc-theta
echo -e "${GREEN}[1/4] Instalando libuvc-theta...${NC}"
if [ -d "libuvc-theta" ]; then
    echo -e "${YELLOW}Directorio libuvc-theta ya existe. ¿Desea reinstalar? (s/n)${NC}"
    read -r response
    if [[ "$response" =~ ^([sS][iI]|[sS])$ ]]; then
        rm -rf libuvc-theta
    else
        echo "Saltando instalación de libuvc-theta..."
        cd libuvc-theta
    fi
fi

if [ ! -d "libuvc-theta" ]; then
    git clone https://github.com/ricohapi/libuvc-theta.git
    cd libuvc-theta

    # Corregir versión mínima de CMake en CMakeLists.txt
    echo -e "${YELLOW}Corrigiendo versión de CMake...${NC}"
    sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' CMakeLists.txt

    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    echo -e "${GREEN}✓ libuvc-theta instalado correctamente${NC}"
else
    cd libuvc-theta
    echo -e "${YELLOW}✓ libuvc-theta ya estaba instalado${NC}"
fi

cd "$INSTALL_DIR"

# Paso 2: Instalar gstthetauvc
echo ""
echo -e "${GREEN}[2/4] Instalando gstthetauvc...${NC}"
if [ -d "gstthetauvc" ]; then
    echo -e "${YELLOW}Directorio gstthetauvc ya existe. ¿Desea reinstalar? (s/n)${NC}"
    read -r response
    if [[ "$response" =~ ^([sS][iI]|[sS])$ ]]; then
        rm -rf gstthetauvc
    else
        echo "Saltando instalación de gstthetauvc..."
        cd gstthetauvc
    fi
fi

if [ ! -d "gstthetauvc" ]; then
    git clone https://github.com/nickel110/gstthetauvc.git
    cd gstthetauvc/thetauvc
    make -j$(nproc)
    echo -e "${GREEN}✓ gstthetauvc instalado correctamente${NC}"
else
    cd gstthetauvc/thetauvc
    echo -e "${YELLOW}✓ gstthetauvc ya estaba instalado${NC}"
fi

# Paso 3: Configurar GST_PLUGIN_PATH
echo ""
echo -e "${GREEN}[3/4] Configurando GST_PLUGIN_PATH...${NC}"
PLUGIN_PATH="$INSTALL_DIR/gstthetauvc/thetauvc"
BASHRC="$HOME/.bashrc"

if grep -q "GST_PLUGIN_PATH.*gstthetauvc" "$BASHRC"; then
    echo -e "${YELLOW}✓ GST_PLUGIN_PATH ya está configurado en .bashrc${NC}"
else
    echo "" >> "$BASHRC"
    echo "# Ricoh Theta Z1 GStreamer Plugin" >> "$BASHRC"
    echo "export GST_PLUGIN_PATH=$PLUGIN_PATH" >> "$BASHRC"
    echo -e "${GREEN}✓ GST_PLUGIN_PATH añadido a .bashrc${NC}"
fi

# Cargar la variable en la sesión actual
export GST_PLUGIN_PATH="$PLUGIN_PATH"

# Paso 4: Verificar instalación
echo ""
echo -e "${GREEN}[4/4] Verificando instalación...${NC}"
echo ""

# Verificar libuvc
if ldconfig -p | grep -q "libuvc"; then
    echo -e "${GREEN}✓ libuvc-theta encontrado en el sistema${NC}"
else
    echo -e "${RED}✗ libuvc-theta NO encontrado${NC}"
fi

# Verificar plugin
if gst-inspect-1.0 thetauvcsrc > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Plugin thetauvcsrc detectado correctamente${NC}"
    echo ""
    echo -e "${YELLOW}Información del plugin:${NC}"
    gst-inspect-1.0 thetauvcsrc | head -15
else
    echo -e "${RED}✗ Plugin thetauvcsrc NO detectado${NC}"
    echo -e "${YELLOW}Puede que necesites reiniciar la terminal o ejecutar:${NC}"
    echo -e "${YELLOW}  source ~/.bashrc${NC}"
fi

echo ""
echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Instalación completada${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo -e "${YELLOW}Pasos siguientes:${NC}"
echo "1. Reinicia la terminal o ejecuta: source ~/.bashrc"
echo "2. Conecta la cámara Ricoh Theta Z1 por USB"
echo "3. Prueba el pipeline con:"
echo "   gst-launch-1.0 thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! queue ! glimagesink sync=false"
echo "4. Ejecuta el componente RicohOmni"
echo ""
