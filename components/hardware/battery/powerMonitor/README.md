# powerMonitor

Componente RoboComp que agrega las lecturas de potencia de varias fuentes (hasta 3) a través de la interfaz **`Power`**, las vuelca a una base de datos de series temporales **InfluxDB** a un ritmo de **10 Hz** (`Period.Compute = 100` ms), y las sirve de nuevo hacia fuera implementando la propia interfaz `Power` como agregador. Las métricas se pueden visualizar con **Grafana**.

```
victron (Power, :15002) ─┐
INA226  (Power, :15001) ─┼─► powerMonitor ──► InfluxDB (:8086) ──► Grafana (:3000)
<tercera fuente>  (Power2) ─┘        (Power, :15003)
```

En cada ciclo de `compute()`, el componente llama a `getPowers()` sobre cada uno de los 3 proxies `Power` requeridos (`Power`, `Power1`, `Power2`); si una fuente no responde (proxy inalcanzable), se registra el error por consola y se continúa con las demás sin interrumpir el ciclo. Cada `PowerData` recibido se escribe en InfluxDB como un punto de la medida `power` (tag `source` = nombre de la fuente; campos `voltage`, `current`, `power`; timestamp = el del propio dato, en milisegundos) y se guarda en caché para responder a `Power_getPower()` / `Power_getPowers()`.

## Instalación y ejecución paso a paso

Probado en una Orange Pi Zero 2W (Ubuntu 24.04, arm64, Ice 3.7.10) con un INA226 como fuente. Los comandos se ejecutan en la máquina donde correrá el componente y dan por hecho que RoboComp ya está instalado, con `$ROBOCOMP` definido, y que el entorno de Python de RoboComp está activo.

> **Seguridad**: las credenciales de InfluxDB (token, usuario, contraseña) **nunca** se guardan en `etc/config` ni en el código. Solo viven en `docker/.env`, que está en `.gitignore`. El componente las carga al arrancar con `python-dotenv`.

### 1. Instalar Docker

Si `docker compose version` ya funciona, pasa directamente a activar el servicio. Si no, instálalo desde los repositorios de Ubuntu:

```bash
sudo apt update
sudo apt install docker.io docker-compose-v2
```

Activa el servicio para que los contenedores vuelvan a arrancar tras un reinicio. Sin esto, puede que el demonio no arranque solo y `restart: unless-stopped` no tenga efecto:

```bash
sudo systemctl enable --now docker
```

Opcionalmente, añade tu usuario a los grupos `docker` e `i2c` para no usar `sudo`. Cierra sesión y vuelve a entrar para que se aplique. El grupo `i2c` solo hace falta en la máquina que lee el INA226:

```bash
sudo usermod -aG docker,i2c $USER
```

Si no lo haces, antepón `sudo` a los comandos `docker` de esta guía.

### 2. Instalar las dependencias de Python

```bash
pip install influxdb-client python-dotenv
```

### 3. Compilar el componente

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/battery/powerMonitor
cmake -B build .
make -C build
```

Esto genera `generated/Power.ice` y el enlace `bin/powerMonitor`.

### 4. Crear las credenciales

Crea `docker/.env` a partir de la plantilla, con contraseña y token aleatorios:

```bash
cd docker
sed -e "s/^INFLUXDB_PASSWORD=.*/INFLUXDB_PASSWORD=$(openssl rand -hex 16)/" \
    -e "s/^INFLUXDB_TOKEN=.*/INFLUXDB_TOKEN=$(openssl rand -hex 32)/" \
    .env.example > .env
chmod 600 .env
```

Las variables que define son:

| Variable | Uso |
|----------|-----|
| `INFLUXDB_USER`, `INFLUXDB_PASSWORD` | Acceso a la interfaz web de InfluxDB |
| `INFLUXDB_ORG`, `INFLUXDB_BUCKET` | Organización y bucket donde se escriben los datos |
| `INFLUXDB_RETENTION` | Tiempo que se conservan los datos (por defecto `15d`) |
| `INFLUXDB_TOKEN` | Token que usan el componente y Grafana |
| `INFLUXDB_URL` | Dirección de InfluxDB para el componente |

> InfluxDB solo aplica estos valores la primera vez que arranca con `docker/influxdb/data` vacío. Cambiar `.env` después no modifica la base de datos ya creada.

### 5. Preparar las carpetas de datos

Grafana se ejecuta con el UID 472 y debe poder escribir en su carpeta. Hazlo antes del primer arranque:

```bash
mkdir -p grafana/data influxdb/data
sudo chown -R 472:472 grafana/data
```

### 6. Arrancar InfluxDB y Grafana

```bash
docker compose up -d
docker compose ps
```

La primera vez descarga las imágenes y tarda unos minutos. Grafana puede tardar además un par de minutos en responder en una placa pequeña. Comprueba que los dos servicios funcionan:

```bash
curl -s localhost:8086/health      # debe contener "status":"pass"
curl -s localhost:3000/api/health  # debe contener "database": "ok"
```

Servicios desplegados:

- **InfluxDB 2.7**: `http://<ip>:8086`, con los datos en `docker/influxdb/data`.
- **Grafana**: `http://<ip>:3000`, con los datos en `docker/grafana/data`. El usuario y la contraseña iniciales son `admin`/`admin`, y pide cambiarla al entrar por la web.

### 7. Conectar Grafana con InfluxDB

En Grafana, ve a **Connections → Data sources → Add data source → InfluxDB** y rellena:

| Campo | Valor |
|-------|-------|
| Query language | `Flux` |
| URL | `http://influxdb:8086` |
| Organization | `INFLUXDB_ORG` de `docker/.env` |
| Token | `INFLUXDB_TOKEN` de `docker/.env` |
| Default Bucket | `INFLUXDB_BUCKET` de `docker/.env` |

Pulsa **Save & test**. Debe indicar que la datasource funciona. Para crear una gráfica, añade un panel con esta consulta, cambiando `powers` si usas otro bucket:

```flux
from(bucket: "powers")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r._measurement == "power" and r._field == "current")
  |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)
```

Los campos disponibles son `voltage` (V), `current` (A) y `power` (W), y la etiqueta `source` es el nombre del sensor.

### 8. Arrancar las fuentes de potencia

Cada fuente debe estar en marcha para que `powerMonitor` pueda leerla. Por ejemplo, para el INA226, en otro terminal:

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/battery/INA226
cmake -B build . && make -C build
bin/INA226 etc/config.toml
```

Consulta el README de cada fuente para sus dependencias y su cableado. Para que arranquen solas con la placa, instálalas como servicios (`robocomp-ina226`, `robocomp-victron`) siguiendo la sección *Arrancar con la placa* de su README.

### 9. Arrancar powerMonitor

Revisa los puertos de `etc/config` (ver [parámetros de configuración](#configuration-parameters)). Para no modificar el fichero del repositorio, puedes usar una copia:

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/battery/powerMonitor
cp etc/config config
bin/powerMonitor config
```

En una sesión SSH o una máquina sin pantalla, indica a Qt que no use ninguna:

```bash
QT_QPA_PLATFORM=offscreen bin/powerMonitor config
```

### 10. Comprobar que se escriben datos

Con 10 Hz deben aparecer unos 100 puntos por campo y sensor en 10 segundos:

```bash
cd docker
. ./.env
curl -s -X POST "localhost:8086/api/v2/query?org=$INFLUXDB_ORG" \
  -H "Authorization: Token $INFLUXDB_TOKEN" \
  -H "Content-Type: application/vnd.flux" \
  -H "Accept: application/csv" \
  --data "from(bucket:\"$INFLUXDB_BUCKET\") |> range(start:-10s) |> filter(fn:(r)=>r._measurement==\"power\") |> group(columns:[\"source\",\"_field\"]) |> count()"
```

### 11. Arrancar con la placa

`etc/robocomp-powermonitor.service` arranca el componente con la placa y lo reinicia 5 s después si falla. Arranca después de Docker y de los servicios `robocomp-ina226` y `robocomp-victron`, aunque no exige que estén en marcha: si falta una fuente, solo registra que no la alcanza. Sus rutas son las del usuario `orangepi` y del entorno `~/software/pip_venv`. Si cambian, edítalas antes de instalarlo.

Instálalo en la placa:

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/battery/powerMonitor
sudo cp etc/robocomp-powermonitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robocomp-powermonitor
```

| Acción | Comando |
|--------|---------|
| Ver el estado | `systemctl status robocomp-powermonitor` |
| Ver los logs en directo | `journalctl -u robocomp-powermonitor -f` |
| Reiniciar tras un cambio | `sudo systemctl restart robocomp-powermonitor` |
| Parar | `sudo systemctl stop robocomp-powermonitor` |
| Quitarlo del arranque | `sudo systemctl disable --now robocomp-powermonitor` |

Si modificas el `.service`, cópialo otra vez, ejecuta `sudo systemctl daemon-reload` y reinicia el servicio. Mientras el servicio esté activo, no arranques el componente a mano: fallará con `Address already in use`. Páralo antes con `sudo systemctl stop robocomp-powermonitor`.

### 12. Subir cambios desde otro equipo

Para editar en otro equipo y llevar los cambios a la placa, copia solo los ficheros modificados. Cambia `<ip-placa>` por la IP de la placa, que puedes ver en ella con `hostname -I`:

```bash
cd $ROBOCOMP/components/robocomp-robolab/components/hardware/battery/powerMonitor
scp src/specificworker.py orangepi@<ip-placa>:~/robocomp/components/robocomp-robolab/components/hardware/battery/powerMonitor/src/
scp etc/config orangepi@<ip-placa>:~/robocomp/components/robocomp-robolab/components/hardware/battery/powerMonitor/etc/
ssh -t orangepi@<ip-placa> sudo systemctl restart robocomp-powermonitor
```

No copies `generated/`, `build/` ni `bin/`. La placa tiene los suyos, generados para su versión de Ice (3.7), y los de otro equipo pueden no funcionar en ella.

### Parar todo

Si los componentes están instalados como servicios (pasos 11 y 12):

```bash
sudo systemctl stop robocomp-powermonitor robocomp-ina226 robocomp-victron
cd docker && docker compose down   # los datos se conservan en docker/*/data
```

Si los arrancaste a mano, ten en cuenta que cambian el nombre de su proceso, así que `pkill -f bin/powerMonitor` no los encuentra:

```bash
pkill -x powerMonitor
pkill -x INA226
pkill -x victron
```

### Problemas frecuentes

| Síntoma | Causa y solución |
|---------|------------------|
| `[Power] unreachable: ... Connection refused` | Esa fuente no está en marcha. El ciclo continúa con las demás. Es lo esperado en `Power2` con `-p 0` y en cualquier fuente no conectada. |
| `Address already in use` | Ya hay otra instancia en marcha, a mano o como servicio. Para el servicio con `sudo systemctl stop robocomp-powermonitor` o la instancia manual con `pkill -x powerMonitor`. |
| `TypeError: argument 1 must be str, not list` en `Ice.loadSlice` | El `generated/interfaces.py` de ese componente solo admite Ice 3.8 y tienes Ice 3.7. Regenéralo con una versión reciente de `robocompdsl`, que admite ambas. |
| `Permission denied` en `/dev/i2c-*` | El usuario no está en el grupo `i2c` (ver el paso 1). |
| `GF_PATHS_DATA ... is not writable` | Faltan los permisos del paso 5. Aplica el `chown` y ejecuta `docker compose restart grafana`. |
| `[InfluxDB] write error: ... unauthorized` | El token de `.env` no coincide con el de la base de datos ya creada (ver la nota del paso 4). |

## Interfaces

| Tipo | Interfaz | Rol |
|------|----------|-----|
| requires | `Power`, `Power1`, `Power2` | Hasta 3 fuentes de potencia a monitorizar (p. ej. `victron`, `INA226`, ...) |
| implements | `Power` | Agregador: expone las últimas lecturas combinadas de todas las fuentes |

## Configuration parameters

Como cualquier otro componente de RoboComp, *powerMonitor* necesita un fichero de configuración para arrancar. En `etc/config` se encuentra el fichero de ejemplo:

```ini
# Proxies de las interfaces requeridas (fuentes de potencia a monitorizar)
Proxies.Power = "power:tcp -h localhost -p 15002"   # victron
Proxies.Power1 = "power:tcp -h localhost -p 15001"  # INA226
# Power2 sin usar por ahora (no hay una tercera fuente configurada); se deja
# apuntando a un puerto inalcanzable a propósito
Proxies.Power2 = "power:tcp -h localhost -p 0"

# Endpoint de la interfaz Power implementada por este propio componente
Endpoints.Power = "tcp -p 15003"

Period.Compute = 100
Period.Emergency = 500

Component.Debug.Verbose = true

Ice.Warn.Connections = "0"
Ice.Trace.Network = "0"
Ice.Trace.Protocol = "0"
Ice.MessageSizeMax = "2097151"
```

- `Proxies.Power` / `Proxies.Power1` / `Proxies.Power2`: direcciones Ice de las hasta 3 fuentes de potencia a monitorizar. Deben apuntar al `Endpoints.Power` configurado en cada componente origen (en este repositorio: `victron` → `15002`, `INA226` → `15001`). Si hay menos de 3 fuentes disponibles, deja el proxy sobrante apuntando a un puerto inalcanzable (p. ej. `-p 0`); el fallo de conexión se captura y se ignora en cada ciclo.
- `Endpoints.Power`: puerto por el que otros componentes pueden consultar las lecturas agregadas de `powerMonitor`.
- `Period.Compute = 100`: frecuencia de muestreo/volcado a InfluxDB (10 Hz).
- Credenciales de InfluxDB: no se definen en `etc/config`; ver `docker/.env` / `docker/.env.example`.
