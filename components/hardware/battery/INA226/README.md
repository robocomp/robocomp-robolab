# INA226

Componente RoboComp que lee uno o varios sensores de corriente/voltaje **INA226** (I2C) y expone sus lecturas a través de dos interfaces Ice:

- **`BatteryStatus`**: estado de la batería principal (voltaje, corriente, potencia y porcentaje estimado a partir de los límites `[Battery]`), tomado del sensor cuyo nombre en `[ID]` sea `battery`.
- **`Power`**: lectura de potencia de **todos** los sensores configurados en `[ID]` (`PowerData` por cada uno: nombre, voltaje, corriente, potencia, timestamp), pensada para ser consumida por componentes agregadores como [`powerMonitor`](../powerMonitor).

Cada sensor declarado en la sección `[ID]` del fichero de configuración se instancia como un objeto `INA226` independiente (biblioteca `ina226`), identificado por su dirección I2C. Si ninguno de los nombres configurados es `"battery"`, `BatteryStatus_getBatteryState()` no tendrá sensor de batería asociado y lo indicará por consola.

## Dependencias

Instalar la librería `ina226`:

```bash
cd ~/software && git clone https://github.com/e71828/pi_ina226.git && pip install ./pi_ina226 && cd -
```

El usuario que ejecute el componente debe tener permisos de acceso al bus I2C (grupo `i2c` en Linux, o ejecutar con privilegios suficientes).

## Interfaces

| Tipo | Interfaz | Rol |
|------|----------|-----|
| implements | `BatteryStatus` | Expone `getBatteryState()` con el estado de la batería principal (sensor `"battery"`) |
| implements | `Power` | Expone `getPower(name)` / `getPowers()` con la lectura de potencia de cada sensor INA226 configurado |

## Configuration parameters

Como cualquier otro componente de RoboComp, *INA226* necesita un fichero de configuración para arrancar. En `etc/config.toml` se encuentra el fichero de ejemplo:

```toml
# Endpoints de las interfaces implementadas
[Endpoints]
BatteryStatus = "tcp -p 16987"
Power = "tcp -p 15001"

[Period]
Compute = 100
Emergency = 500

[Component]
Debug.Verbose = true

[Ice]
Warn.Connections = "0"
Trace.Network = "0"
Trace.Protocol = "0"
MessageSizeMax = "20004800"

# Límites de voltaje de la batería (en voltios), usados para calcular el porcentaje
[Battery]
max = 28.8
min = 19.90

# Sensores INA226 a instanciar: nombre -> dirección I2C (hexadecimal)
# El sensor llamado "battery" (si existe) es el que alimenta BatteryStatus_getBatteryState()
[ID]
battery = "0x4F"
24Aux = "0x50"
```

- `[ID]`: cada entrada `nombre = "0xNN"` crea un sensor `INA226(busnum=1, address=0xNN, max_expected_amps=20, shunt_ohms=0.002)`. El nombre es el identificador usado en `Power_getPower(name)` y en la respuesta de `Power_getPowers()`.
- `[Battery].max` / `[Battery].min`: voltajes de referencia para calcular el porcentaje de batería (`calc_percentage`); solo se usan si hay un sensor `battery` configurado.
- `Endpoints.Power`: puerto por el que otros componentes (p. ej. `powerMonitor`) acceden a las lecturas de potencia. Debe coincidir con el `Proxies.Power`/`Proxies.PowerN` configurado en el componente que lo consuma.

## Starting the component

Para evitar modificar el *config* del repositorio, se puede copiar a un fichero local que no se vea afectado por futuros `git pull`:

```bash
cd <ruta de INA226>
cp etc/config.toml config
```

Tras editar el nuevo fichero de configuración, arrancamos el componente:

```bash
bin/INA226 config
```
