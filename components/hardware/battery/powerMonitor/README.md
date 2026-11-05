# powerMonitor

Componente RoboComp que agrega las lecturas de potencia de varias fuentes (hasta 3) a través de la interfaz **`Power`**, las vuelca a una base de datos de series temporales **InfluxDB** a un ritmo de **10 Hz** (`Period.Compute = 100` ms), y las sirve de nuevo hacia fuera implementando la propia interfaz `Power` como agregador. Las métricas se pueden visualizar con **Grafana**.

```
victron (Power, :15002) ─┐
INA226  (Power, :15001) ─┼─► powerMonitor ──► InfluxDB (:8086) ──► Grafana (:3000)
<tercera fuente>  (Power2) ─┘        (Power, :15003)
```

En cada ciclo de `compute()`, el componente llama a `getPowers()` sobre cada uno de los 3 proxies `Power` requeridos (`Power`, `Power1`, `Power2`); si una fuente no responde (proxy inalcanzable), se registra el error por consola y se continúa con las demás sin interrumpir el ciclo. Cada `PowerData` recibido se escribe en InfluxDB como un punto de la medida `power` (tag `source` = nombre de la fuente; campos `voltage`, `current`, `power`; timestamp = el del propio dato, en milisegundos) y se guarda en caché para responder a `Power_getPower()` / `Power_getPowers()`.

## Dependencias

```bash
pip3 install influxdb-client python-dotenv
```

Y **Docker** + **Docker Compose** para desplegar la pila de InfluxDB/Grafana (ver más abajo).

> **Seguridad**: las credenciales de InfluxDB (token, usuario, contraseña) **nunca** se guardan en `etc/config` ni en el código — solo viven en `docker/.env`, que está en `.gitignore` y no se sube al repositorio. El componente las carga en tiempo de ejecución con `python-dotenv` a partir de ese fichero.

## Interfaces

| Tipo | Interfaz | Rol |
|------|----------|-----|
| requires | `Power`, `Power1`, `Power2` | Hasta 3 fuentes de potencia a monitorizar (p. ej. `victron`, `INA226`, ...) |
| implements | `Power` | Agregador: expone las últimas lecturas combinadas de todas las fuentes |

## Despliegue de InfluxDB + Grafana (docker-compose)

La configuración vive en `docker/`:

```bash
cd <ruta de powerMonitor>/docker
cp .env.example .env   # y edita las credenciales con valores propios
docker compose up -d
```

`docker/.env` (nunca se commitea; solo `.env.example` queda versionado como plantilla) debe definir:

```bash
INFLUXDB_USER=...
INFLUXDB_PASSWORD=...
INFLUXDB_ORG=...
INFLUXDB_BUCKET=...
INFLUXDB_RETENTION=...
INFLUXDB_TOKEN=...
INFLUXDB_URL=http://localhost:8086
```

Estas mismas variables son las que lee `src/specificworker.py` (vía `python-dotenv`) para conectarse a InfluxDB, así que `docker/.env` es la única fuente de verdad: configúralo una vez y tanto el stack de Docker como el propio componente lo usarán.

Servicios desplegados:

- **InfluxDB 2.7** — `http://localhost:8086` (UI/API). Bucket `powers` dentro de la organización `shadow`, con la retención y el token definidos arriba. Datos persistidos en `docker/influxdb/data`.
- **Grafana** — `http://localhost:3000` (usuario/contraseña por defecto `admin`/`admin` en el primer arranque). Datos persistidos en `docker/grafana/data`. Para consultar las métricas, añade una datasource de tipo *InfluxDB* apuntando a `http://influxdb:8086` (o `http://localhost:8086` si Grafana corre fuera de la red de docker-compose), con la organización, el bucket y el token anteriores.

> **Nota de permisos**: si `docker/grafana/data` se crea por primera vez con `docker compose up`, puede quedar con propietario `root` y provocar que el contenedor de Grafana (UID 472) no pueda escribir en él. Si Grafana no arranca con un error `GF_PATHS_DATA ... is not writable`, corrige los permisos con:
> ```bash
> docker run --rm --user 0 --entrypoint chown \
>   -v "$(pwd)/grafana/data:/var/lib/grafana" \
>   grafana/grafana:latest -R 472:472 /var/lib/grafana
> docker compose restart grafana
> ```

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

## Starting the component

Para evitar modificar el *config* del repositorio, se puede copiar a un fichero local que no se vea afectado por futuros `git pull`:

```bash
cd <ruta de powerMonitor>
cp etc/config config
```

Tras editar el nuevo fichero de configuración (puertos de las fuentes) y tener `docker/.env` configurado, arrancamos el componente:

```bash
bin/powerMonitor config
```
