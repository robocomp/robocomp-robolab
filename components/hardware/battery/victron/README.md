# victron

Componente RoboComp que se conecta por puerto serie a un cargador/controlador **Victron** mediante el protocolo **VE.Direct** (ver `VE.Direct-Protocol.pdf` incluido en este directorio) y publica el estado de la batería a través de dos interfaces Ice:

- **`BatteryStatus`**: estado completo de la batería (voltaje, corriente, consumo acumulado, porcentaje y estado: `Charging` / `Charged` / `Disconnected`).
- **`Power`**: lectura genérica de potencia (`PowerData`: nombre, voltaje, corriente, potencia, timestamp), pensada para ser consumida por componentes agregadores como [`powerMonitor`](../powerMonitor).

Internamente usa el módulo local `src/vedirect.py` (requiere `pyserial`) para parsear las tramas VE.Direct que llegan por el puerto serie configurado, a una frecuencia marcada por `Period.Compute`. Además lanza un hilo en segundo plano (`warning_function`) que reproduce avisos sonoros (`audio/Please_charge_me.mp3`, `audio/Hunger.mp3` vía `ffplay`) cuando la batería baja del 25 % o del 15 % mientras se está descargando.

## Dependencias

```bash
pip3 install pyserial
```

`ffplay` (paquete `ffmpeg`) debe estar instalado en el sistema si se quieren los avisos sonoros de batería baja.

## Interfaces

| Tipo | Interfaz | Rol |
|------|----------|-----|
| implements | `BatteryStatus` | Expone `getBatteryState()` con el estado completo de la batería |
| implements | `Power` | Expone `getPower(name)` / `getPowers()` con la lectura de potencia (fuente `"battery"`) |

## Configuration parameters

Como cualquier otro componente de RoboComp, *victron* necesita un fichero de configuración para arrancar. En `etc/config` se encuentra el fichero de ejemplo:

```ini
# Endpoints de las interfaces implementadas
Endpoints.BatteryStatus = "tcp -p 14498"
Endpoints.Power = "tcp -p 15002"

# Puerto serie donde está conectado el Victron (protocolo VE.Direct)
serialPort = "/dev/ttyUSB0"

# Umbrales de la batería (en voltios), usados para calcular el porcentaje
vMaxCharging = 54.0
vMax = 52.6
vMin = 45.0

Period.Compute = 100
Period.Emergency = 500

Component.Debug.Verbose = true

Ice.Warn.Connections = "0"
Ice.Trace.Network = "0"
Ice.Trace.Protocol = "0"
Ice.MessageSizeMax = "2097151"
```

- `serialPort`: dispositivo serie del Victron (en Linux suele aparecer como `/dev/ttyUSB0` o, si se usa una regla `udev`, un alias fijo como `/dev/Victron`).
- `vMax` / `vMin` / `vMaxCharging`: voltajes de referencia usados por `vedirect.calc_percentage()` para estimar el porcentaje de batería según si está cargando o descargándose.
- `Endpoints.Power`: puerto por el que otros componentes (p. ej. `powerMonitor`) acceden a la lectura de potencia de este Victron. Debe coincidir con el valor configurado como `Proxies.Power`/`Proxies.PowerN` en el componente que lo consuma.

## Starting the component

Para evitar modificar el *config* del repositorio, se puede copiar a un fichero local que no se vea afectado por futuros `git pull`:

```bash
cd <ruta de victron>
cp etc/config config
```

Tras editar el nuevo fichero de configuración, arrancamos el componente:

```bash
bin/victron config
```

## Referencias

- [VE.Direct protocol parsing (foro Victron Community)](https://community.victronenergy.com/questions/29816/how-to-parse-vedirect-protocol.html)
- `VE.Direct-Protocol.pdf` (incluido en este directorio): especificación oficial del protocolo.
