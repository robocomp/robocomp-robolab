# ledStrip

Componente RoboComp que controla una tira de LED direccionables WS2812B e implementa la interfaz `LEDArray` (puerto 10991). Admite dos modos de salida, según el campo `LED.Interface` de `etc/config`:

| Modo | Placa | Salida | Biblioteca |
|------|-------|--------|------------|
| `GPIO` (por defecto) | Raspberry Pi | Pin de `LED.GPIO`, como `board.D18` | `adafruit-circuitpython-neopixel` |
| `SPI` | Orange Pi Zero 2W | MOSI de SPI1 | `adafruit-circuitpython-neopixel-spi` |

## Orange Pi Zero 2W

### Por qué SPI y no PWM

WS2812B necesita una señal de unos 800 kHz en la que cada bit tiene su propio ancho de pulso. El PWM por `sysfs` solo fija un periodo y un ciclo de trabajo, y no puede cambiarlos en cada bit. Por eso los pines PWM de la placa (7, 16, 32 y 33) **no sirven** para esta tira. `rpi_ws281x` tampoco es una opción, porque depende de los periféricos de la Raspberry.

El bus SPI genera la señal por hardware con DMA: cada bit de la tira se codifica como un byte SPI.

### Conexión

| Tira | Orange Pi Zero 2W |
|------|-------------------|
| DIN | **Pin 19** (PH7, SPI1 MOSI), con una resistencia en serie de 300-500 Ω |
| GND | Cualquier GND, por ejemplo el **pin 20**. Debe ser común con la fuente |
| 5V | **Fuente externa de 5 V**, no el pin de la placa |

- **Consumo:** cada LED puede llegar a 60 mA. Con 54 LED son unos 3,2 A en blanco al máximo y unos 0,65 A con el brillo de 0.2 que usa el componente.
- **Nivel lógico:** la placa da 3,3 V y WS2812B a 5 V espera al menos 3,5 V (0,7·VDD). Puede funcionar sin adaptador, pero no está garantizado. Si hay parpadeos o colores erróneos, añade un adaptador como el 74AHCT125.
- **Condensador:** se recomienda uno de unos 1000 µF entre 5 V y GND, cerca de la tira.
- **Pines reservados:** con SPI1 activo, los pines 19, 21, 23 y 26 no están disponibles como GPIO. Los pines 21, 23 y 26 se quedan sin conectar.

### Preparar el sistema

**1. Activar SPI1.** En `/boot/orangepiEnv.txt`, añade `spi1-cs1-spidev` a `overlays` y quita `ph-i2c4`, que ocupa los pines 19 y 23. Reinicia después:

```
overlays=ph-i2c1 ph-i2c2 ph-i2c3 pi-i2c0 pi-i2c1 pi-i2c2 spi1-cs1-spidev
```

> Al quitar `ph-i2c4` cambia la numeración de los buses: `i2c-4` pasa a ser el regulador de la placa (`axp313a`) y no se debe escribir en él. `i2c-1` (pines 3 y 5) no cambia.

**2. Dar permisos.** La regla de la imagen, `/etc/udev/rules.d/99-gpio.rules`, usa un grupo `gpio` que no existe, así que `/dev/gpiochip*` queda solo para root:

```bash
sudo groupadd -r gpio
sudo groupadd -r spi
sudo usermod -aG gpio,spi $USER
echo 'SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"' | sudo tee /etc/udev/rules.d/99-spidev.rules
sudo reboot
```

**3. Instalar Blinka y la biblioteca SPI.** Usa el fork [`alfiTH/Adafruit_Blinka`](https://github.com/alfiTH/Adafruit_Blinka) con la compatibilidad de la Zero 2W:

```bash
cd ~/software/Adafruit_Blinka
pip install .
pip install adafruit-circuitpython-neopixel-spi
```

Los cambios del fork son:

- `src/adafruit_blinka/board/orangepi/orangepizero2w.py`: mapa de pines de la Zero 2W.
- `src/board.py`: PlatformDetect identifica la Zero 2W como `ORANGE_PI_ZERO_2`, así que se distingue por el modelo del device tree. La Zero 2 tiene otro conector.
- `src/adafruit_blinka/microcontroller/allwinner/h616/pin.py`: usa `/dev/spidev1.1` cuando no existe `/dev/spidev1.0`.

### Compilar y arrancar

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/led/ledStrip
cmake -B build .
make -C build
QT_QPA_PLATFORM=offscreen bin/ledStrip etc/config
```

### Arrancar con la placa

`etc/robocomp-ledstrip.service` arranca el componente con la placa y lo reinicia 5 s después si falla. Añade los grupos `gpio` y `spi` solo a este servicio (`SupplementaryGroups=gpio spi`), aunque los grupos deben existir (paso 2). Sus rutas son las del usuario `orangepi` y del entorno `~/software/pip_venv`. Si cambian, edítalas antes de instalarlo.

Instálalo en la placa:

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/led/ledStrip
sudo cp etc/robocomp-ledstrip.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robocomp-ledstrip
```

| Acción | Comando |
|--------|---------|
| Ver el estado | `systemctl status robocomp-ledstrip` |
| Ver los logs en directo | `journalctl -u robocomp-ledstrip -f` |
| Reiniciar tras un cambio | `sudo systemctl restart robocomp-ledstrip` |
| Parar | `sudo systemctl stop robocomp-ledstrip` |
| Quitarlo del arranque | `sudo systemctl disable --now robocomp-ledstrip` |

Si modificas el `.service`, cópialo otra vez, ejecuta `sudo systemctl daemon-reload` y reinicia el servicio. Mientras el servicio esté activo, no arranques el componente a mano: fallará con `Address already in use` y los dos procesos competirían por el bus SPI. Páralo antes con `sudo systemctl stop robocomp-ledstrip`.

### Subir cambios desde otro equipo

Para editar en otro equipo y llevar los cambios a la placa, copia solo los ficheros modificados. Cambia `<ip-placa>` por la IP de la placa, que puedes ver en ella con `hostname -I`:

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/led/ledStrip
scp src/specificworker.py orangepi@<ip-placa>:~/robocomp/components/robocomp-robolab/components/hardware/led/ledStrip/src/
scp etc/config orangepi@<ip-placa>:~/robocomp/components/robocomp-robolab/components/hardware/led/ledStrip/etc/
ssh -t orangepi@<ip-placa> sudo systemctl restart robocomp-ledstrip
```

No copies `generated/`, `build/` ni `bin/`. La placa tiene los suyos, generados para su versión de Ice (3.7), y los de otro equipo pueden no funcionar en ella.

### Temporización

El reloj de SPI1 parte de 24 MHz. Pidiendo 6 o 6,4 MHz se midió la misma velocidad efectiva, de unos 5,7 MHz, así que el componente usa `frequency=6000000`, `bit0=0b11000000` y `bit1=0b11111000`:

| Bit | Alto | Bajo |
|-----|------|------|
| 0 | ~333 ns | ~1000 ns |
| 1 | ~833 ns | ~500 ns |

Son valores calculados, no medidos con osciloscopio.

**Límite de LED:** el `spidev` envía como máximo 4096 bytes por transferencia y cada LED ocupa 24, así que el límite es de unos 165. Para usar más, sube `spidev.bufsiz`.

### Problemas frecuentes

| Síntoma | Causa y solución |
|---------|------------------|
| `NotImplementedError: Board not supported` | Se está usando `neopixel` (modo GPIO), que en Blinka solo admite Raspberry. Usa `LED.Interface = "SPI"`. |
| `PermissionError` en `gpiod.Chip` o `/dev/spidev1.1` | Faltan los grupos del paso 2. Cierra sesión y vuelve a entrar. |
| `No such file or directory: /dev/spidev1.*` | El overlay no se ha cargado. Revisa el paso 1 y reinicia. |
| `import board` carga el mapa de la Zero 2 | Falta el fork de Blinka. Repite el paso 3. |
| El primer LED parpadea o los colores fallan | Comprueba el GND común y el nivel lógico de 3,3 V. |

## Raspberry Pi

```bash
pip3 install rpi_ws281x adafruit-circuitpython-neopixel
```

En `etc/config`, usa `LED.Interface = "GPIO"` e indica el pin en `LED.GPIO`, por ejemplo `18` para `board.D18`.

## Configuración

| Parámetro | Descripción |
|-----------|-------------|
| `Endpoints.LEDArray` | Puerto de la interfaz `LEDArray` |
| `LED.Interface` | `SPI` (Orange Pi Zero 2W) o `GPIO` (Raspberry Pi) |
| `LED.Number` | Número de LED de la tira |
| `LED.GPIO` | Pin de salida; solo se usa en modo `GPIO` |

Para no modificar el `config` del repositorio, puedes usar una copia:

```bash
cp etc/config config
bin/ledStrip config
```

## Interfaz

| Tipo | Interfaz | Rol |
|------|----------|-----|
| implements | `LEDArray` | `setLEDArray` cambia los LED indicados y devuelve `false` si algún índice está fuera de rango. `getLEDArray` devuelve los últimos valores enviados |
