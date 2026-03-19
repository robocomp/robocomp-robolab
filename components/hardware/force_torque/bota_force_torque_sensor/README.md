# bota_force_sensor

Componente RoboComp para el sensor de fuerza y torque **Bota Systems BFT-SENS-SER-M8**. Lee continuamente las seis componentes de fuerza y torque (Fx, Fy, Fz, Tx, Ty, Tz) a través de la interfaz serie RS-422/USB y las publica mediante la interfaz ICE `ForceTorqueSensor`.

Características principales:
- Arranque robusto con hasta 5 reintentos automáticos y reset por software entre intentos
- Detección automática de datos no calibrados mediante sanity check de rango físico
- Sincronización automática al stream binario del sensor (búsqueda de cabecera `0xAA` + CRC16 X25)
- Tare software mediante promediado de N frames
- Publicación de datos calibrados en N y Nm
- Visualizador en tiempo real integrado (matplotlib + PySide6)

---

## Dependencias

```bash
sudo apt install libftdi-dev
```

### Configuración del dispositivo USB (imprescindible)

El sensor usa un chip FTDI como adaptador USB→RS-422. Por defecto el driver acumula datos durante **16 ms** antes de entregarlos al sistema operativo, lo que provoca que lleguen varios frames agrupados. Es imprescindible reducir este valor a **1 ms**.

> **⚠ IMPORTANTE:** Sin este ajuste el componente puede fallar al sincronizarse con el sensor o leer datos incorrectos de forma intermitente.

#### Regla udev permanente (recomendado)

Crear `/etc/udev/rules.d/99-bota-ft-sensor.rules` con el siguiente contenido. Sustituir el valor de `serial` por el del dispositivo real:

```
SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="0403", \
  ATTRS{idProduct}=="6001", \
  ATTRS{serial}=="AV0KXDYP", \
  MODE="0666", \
  SYMLINK+="bota_ft_sensor", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
```

Aplicar sin reiniciar:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Para obtener el serial del dispositivo FTDI:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep serial
```

#### Verificación del latency timer

```bash
# Debe devolver 1
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Cambiarlo manualmente si fuera necesario
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

---

## Parámetros de comunicación serie

| Parámetro  | Valor                                                                   |
|------------|-------------------------------------------------------------------------|
| Baudrate   | 460800 bps (por defecto del sensor)                                     |
| Data bits  | 8                                                                       |
| Stop bits  | 1                                                                       |
| Parity     | none                                                                    |
| Flow ctrl  | none                                                                    |
| Frame size | 37 bytes (1 header + 2 status + 24 F/T + 4 timestamp + 4 temp + 2 CRC) |
| Header     | `0xAA` (primer byte del frame)                                          |
| Checksum   | CRC16 X25                                                               |

> **ℹ NOTA:** El sensor puede arrancar ocasionalmente sin la matriz de calibración activa, enviando datos ADC crudos en lugar de N/Nm. El bit 3 del campo `status` del frame indica este estado. El componente detecta esta situación mediante el sanity check y reintenta el arranque automáticamente.

---

## Configuración

Copiar el fichero de configuración para evitar que `git pull` lo sobreescriba:

```bash
cp etc/config etc/myConfig
```

Parámetros relevantes en `etc/config`:

| Parámetro                     | Valor por defecto                                    |
|-------------------------------|------------------------------------------------------|
| `device`                      | `"/dev/bota_ft_sensor"` (o `/dev/ttyUSB0` sin udev) |
| `Period.Compute`              | `10` (ms entre lecturas, ~100 Hz)                    |
| `Period.Emergency`            | `500` (ms en estado de emergencia)                   |
| `Endpoints.ForceTorqueSensor` | `tcp -p 10005`                                       |

---

## Compilación y ejecución

```bash
cmake -B build && make -C build -j$(nproc)
bin/bota_force_sensor etc/myConfig
```

Salida esperada en un arranque normal:

```
[BotaFTSensor] Intento 1/5...
[BotaFTSensor] Sincronizado, sensor listo.
[BotaFTSensor] Opened /dev/bota_ft_sensor (fd=13).
[BotaFTSensor] Tare completado. Offsets: Fx=0.1234 Fy=... Fz=... Tx=... Ty=... Tz=...
```

Cuando el componente detecta un arranque incorrecto y reintenta automáticamente:

```
[BotaFTSensor] Intento 1/5...
[BotaFTSensor] Sincronizado, sensor listo.
[BotaFTSensor] Intento 2/5...
[BotaFTSensor] Sincronizado, sensor listo.
[BotaFTSensor] Opened /dev/bota_ft_sensor (fd=13).
```

---

## Interfaz ICE

El componente publica en el puerto configurado (por defecto `tcp -p 10005`) mediante la interfaz `ForceTorqueSensor`. El método `getSensorData()` devuelve:

| Campo         | Descripción                               |
|---------------|-------------------------------------------|
| `fx, fy, fz`  | Componentes de fuerza en N                |
| `tx, ty, tz`  | Componentes de torque en Nm               |
| `temperature` | Temperatura del sensor en °C              |
| `timestamp`   | Timestamp interno del sensor en µs        |
| `valid`       | `true` si el frame es válido y calibrado  |

---

## Solución de problemas

| Síntoma | Solución |
|---------|----------|
| Valores absurdos (millones de N) | El sensor arrancó sin calibración. El componente lo detecta y reintenta automáticamente. Si ocurre de forma sistemática, revisar la conexión USB y que el latency timer sea 1. |
| `ERROR: no se pudo arrancar tras 5 intentos` | Verificar que el sensor está encendido y que `/dev/ttyUSB0` o `/dev/bota_ft_sensor` existen. El componente pasará a estado Emergency y reintentará periódicamente. |
| `Permission denied` al abrir el puerto | Comprobar que la regla udev está activa con `MODE=0666`. Alternativa: `sudo usermod -aG dialout $USER` y reloguear. |
| Datos entrecortados / CRC errors frecuentes | Verificar que el latency timer es 1: `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`. Si es 16, la regla udev no está activa. |
| Sensor no encontrado en `/dev/bota_ft_sensor` | La regla udev no está cargada. Usar `/dev/ttyUSB0` en `etc/config` o recargar: `sudo udevadm control --reload-rules && sudo udevadm trigger` |

---
---

# Developer Notes

## Editable Files

- `etc/*` — ficheros de configuración
- `src/*` — lógica del componente
- `src/BotaFTSensor.h` / `src/BotaFTSensor.cpp` — wrapper del sensor (no generado por RoboComp)
- `README.md` — esta documentación

> **⚠ ATENCIÓN:** La carpeta `generated/` contiene ficheros autogenerados por RoboComp. No editar directamente; serán sobreescritos en el próximo `cmake`.

## Arquitectura de BotaFTSensor

Wrapper C++ sobre `BotaForceTorqueSensorComm` (clase abstracta del fabricante). La clase interna `Impl` implementa los métodos virtuales puros `serialReadBytes()` y `serialAvailable()` usando el file descriptor del puerto serie.

### Flujo de `open()`

El método implementa un bucle de hasta 5 intentos:

1. `openPort()` — abre el dispositivo con `VMIN=1`, `VTIME=0` y activa el modo low-latency del chip FTDI
2. Si intento > 1: `softwareReset()` — envía `'I'` (reset software, válido desde cualquier estado), espera 1.5 s, hace flush, envía `'R'` para forzar modo RUN y espera 0.5 s
3. `flushStartup()` — lee byte a byte hasta encontrar `0xAA` con CRC válido; cambia a `VMIN=37` al sincronizar
4. `sanityCheck()` — lee 10 frames y verifica fuerzas < 500 N y torques < 50 Nm; requiere 8/10 válidos
5. Si el sanity check pasa: abierto correctamente. Si falla: cierra y reintenta.

### Estados RoboComp

| Estado     | Descripción                                                           |
|------------|-----------------------------------------------------------------------|
| Initialize | Abre el puerto con reintentos automáticos y realiza tare inicial      |
| Compute    | Lee un frame cada `Period.Compute` ms y actualiza los datos publicados |
| Emergency  | Se activa si `open()` falla tras 5 intentos; reintenta periódicamente |
| Restore    | Vuelve a Compute en cuanto el puerto se abre correctamente            |

## ConfigLoader

```cpp
// Lectura de parámetros en initialize()
std::string device = this->configLoader.get<std::string>("device");
int period         = this->configLoader.get<int>("Period.Compute");
```

## Instalación de dependencias Qt6

```bash
sudo apt install qt6-base-dev qt6-declarative-dev qt6-scxml-dev \
                 libqt6statemachineqml6 libqt6statemachine6
```
