"""
plot_viewer.py — visor pyqtgraph de la trayectoria, lanzado como proceso de
sistema operativo GENUINAMENTE independiente (subprocess.Popen), NUNCA con
multiprocessing.Process.

Por qué no multiprocessing: generated/imu_fusion.py hace
`from src.specificworker import *` (Ice, pydsr, el driver del Phidget) A
NIVEL DE MÓDULO, antes del `if __name__ == '__main__':`. Con
multiprocessing.Process(spawn), el hijo tiene que reimportar ese mismo
módulo como __mp_main__ para poder resolver la función objetivo — el guard
evita que se relance el agente, pero las importaciones pesadas de arriba SÍ
se re-ejecutan igualmente, con sus efectos secundarios (pools de hilos de
Ice, el QueuedSignalRunner de DSR, intento de reabrir el Phidget...).
Verificado en vivo: el hijo acababa con ~35 hilos para una ventana que solo
necesita pyqtgraph, y el agente se quedaba bloqueado. subprocess.Popen
ejecuta este archivo como script nuevo y aislado — nunca toca
generated/imu_fusion.py ni sus imports.

Protocolo (una línea JSON por frame en stdin, ver _send_plot_frame en
specificworker.py):
  1ª línea : {"room_contour": [[x,y], ...]} o {"room_contour": null}
  siguientes: {"t": [...], "x": [...], "y": [...], "yaw": [...],
               "sx": [...], "sy": [...], "nis": [...],
               "x_odom": [...], "y_odom": [...],      (deltas, no el histórico)
               "table": {...} o ausente}   (snapshot NO acumulativo del
                                             último get_state(), ver
                                             _build_state_table en
                                             specificworker.py; sustituye
                                             al anterior, no se acumula)
  EOF (el padre cierra stdin) → cierra la ventana y termina.
"""
import json
import sys
import threading
import queue

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QApplication, QWidget, QHBoxLayout,
                               QTableWidget, QTableWidgetItem, QHeaderView)

# Filas de la tabla de variables internas del ESKF: (etiqueta, clave en
# "table", nº de valores). Las de 3 valores comparten columnas X/Y/Z --
# para las filas de ángulo esas mismas columnas son roll/pitch/yaw (ver
# cabecera de la tabla más abajo), y para NIS/contadores solo se rellenan
# las columnas que hacen falta.
_TABLE_ROWS = [
    ("pos [m]",           "pos",           3),
    ("vel [m/s]",         "vel",           3),
    ("rpy [°]",           "rpy_deg",       3),
    ("ba [m/s²]",         "ba",            3),
    ("bg [°/s]",          "bg_deg",        3),
    ("σ pos [m]",         "sigma_pos",     3),
    ("σ vel [m/s]",       "sigma_vel",     3),
    ("σ rpy [°]",         "sigma_rpy_deg", 3),
    ("σ ba [m/s²]",       "sigma_ba",      3),
    ("σ bg [°/s]",        "sigma_bg_deg",  3),
    ("NIS (last / mean)", "_nis",          2),
    ("upd / rej / consec_rej", "_counts",  3),
    ("motion_alpha (0=estát/1=din)", "_alpha", 1),  # Bug #13: ruido adaptativo, ver ekf.py
]


def _update_state_table(table: QTableWidget, data: dict) -> None:
    """Rellena la tabla con el snapshot recibido (ver _TABLE_ROWS)."""
    for row, (_, key, _n) in enumerate(_TABLE_ROWS):
        if key == "_nis":
            vals = [data.get("nis_last"), data.get("nis_mean"), None]
        elif key == "_counts":
            vals = [data.get("n_updates"), data.get("n_rejected"), data.get("n_consec_rej")]
        elif key == "_alpha":
            vals = [data.get("motion_alpha"), None, None]
        else:
            v = data.get(key)
            vals = list(v) if v is not None else [None, None, None]
        for col in range(3):
            v = vals[col] if col < len(vals) else None
            if v is None:
                text = ""
            elif key == "_counts":
                text = str(int(v))
            else:
                text = f"{float(v):+.4f}"
            table.item(row, col).setText(text)

_ELLIPSE_EVERY = 50   # una elipse cada 50 muestras (~0.5 s a 100Hz de grabación)
_POLL_MS = 100        # cadencia de redibujado (~10 Hz), igual que el _VIEW_EVERY previo
_TRAIL_LEN = 100      # nº de poses recientes resaltadas (cola/comet trail)
_WINDOW_S = 20.0      # ventana móvil: se descarta todo lo más viejo que esto,
                       # así el coste de redibujado queda acotado sin importar
                       # cuánto dure la sesión (rec['t'] es tiempo relativo al
                       # inicio de RUNNING, ver _record_step en specificworker.py)


def _stdin_reader(local_queue: queue.Queue) -> None:
    """
    Hilo lector: bloquea en sys.stdin, nunca en el hilo Qt. Empuja cada línea
    ya parseada a local_queue (queue.Queue estándar, thread-safe dentro de
    este único proceso — no hay multiprocessing de por medio).
    """
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            local_queue.put(json.loads(line))
        except json.JSONDecodeError:
            continue
    local_queue.put(None)  # EOF del padre: sentinel de cierre


def main() -> None:
    app = QApplication.instance() or QApplication([])

    local_queue: queue.Queue = queue.Queue()
    reader = threading.Thread(target=_stdin_reader, args=(local_queue,), daemon=True)
    reader.start()

    # Primera línea, bloqueante: contorno de la room (estático, se dibuja una vez).
    try:
        first = local_queue.get()
    except Exception:
        first = None
    room_contour = None
    if isinstance(first, dict) and first.get("room_contour"):
        room_contour = np.asarray(first["room_contour"], dtype=np.float64)

    rec = {'t': [], 'x': [], 'y': [], 'yaw': [], 'sx': [], 'sy': [], 'nis': [],
           'x_odom': [], 'y_odom': []}
    ell_items: list = []        # [(t, PlotDataItem), ...] -- se podan al salir de la ventana
    state = {'total_rows': 0, 'ell_created': 0}  # contadores acumulativos, no se resetean al recortar

    win = pg.GraphicsLayoutWidget(title='imu_base_fusion — trayectoria en tiempo real')
    win.setMinimumWidth(750)

    traj = win.addPlot(title='Posición absoluta (elipses 3σ)')
    traj.setAspectLocked(True)
    traj.setLabel('left', 'y', units='m')
    traj.setLabel('bottom', 'x', units='m')
    traj.showGrid(x=True, y=True, alpha=0.25)
    traj.addLegend(offset=(5, 5))
    if room_contour is not None and len(room_contour) >= 3:
        closed = np.vstack([room_contour, room_contour[0]])
        traj.plot(closed[:, 0], closed[:, 1],
                  pen=pg.mkPen((120, 120, 120), width=1.5), name='contorno room')
    traj.plot([0], [0], symbol='t', symbolSize=10, symbolBrush='g', pen=None, name='inicio')
    traj_curve = traj.plot([], [], pen=pg.mkPen('b', width=1.2), name='fusión (imu+base)')
    trail_curve = traj.plot([], [], pen=pg.mkPen('c', width=3.0), name=f'cola (últimas {_TRAIL_LEN})')
    traj_dot = traj.plot([], [], symbol='o', symbolSize=7, symbolBrush='b', pen=None, name='actual')
    odom_curve = traj.plot([], [], pen=pg.mkPen((255, 140, 0), width=1.2, style=Qt.DashLine),
                           name='odometría pura')
    traj_arrow = pg.ArrowItem(angle=0, tipAngle=30, headLen=12,
                              brush=pg.mkBrush('r'), pen=pg.mkPen('r', width=1))
    traj.addItem(traj_arrow)

    win.nextColumn()
    nis_plot = win.addPlot(title='NIS  /  σ_pos')
    nis_plot.setLabel('bottom', 't', units='s')
    nis_plot.setLabel('left', 'NIS')
    nis_plot.showGrid(x=True, y=True, alpha=0.25)
    nis_plot.setYRange(0, 15)
    nis_plot.addLegend(offset=(5, 5))
    nis_plot.addLine(y=3.0, pen=pg.mkPen('g', width=1, style=Qt.DashLine))
    nis_plot.addLine(y=11.34, pen=pg.mkPen('r', width=1, style=Qt.DashLine))
    nis_curve = nis_plot.plot([], [], pen=pg.mkPen(color=(160, 80, 200), width=1.0), name='NIS')
    nis_vb = pg.ViewBox()
    nis_plot.showAxis('right')
    nis_plot.scene().addItem(nis_vb)
    nis_plot.getAxis('right').linkToView(nis_vb)
    nis_vb.setXLink(nis_plot)
    nis_plot.getAxis('right').setLabel('σ_pos', units='m', color='#FFA040')
    sig_curve = pg.PlotDataItem([], [], pen=pg.mkPen(color='#FFA040', width=1, style=Qt.DotLine),
                                name='σ_xy')
    nis_vb.addItem(sig_curve)

    def _sync_vb():
        nis_vb.setGeometry(nis_plot.vb.sceneBoundingRect())
    nis_plot.vb.sigResized.connect(_sync_vb)

    # Tabla de variables internas del ESKF (pos/vel/actitud/sesgos/covarianzas/
    # NIS/contadores) -- útil para diagnosticar p.ej. deriva lateral en
    # estático que no se aprecia bien en la trayectoria x,y (mirar vel/
    # sigma_vel/ba directamente en vez de esperar a que se note en pos).
    state_table = QTableWidget(len(_TABLE_ROWS), 3)
    state_table.setHorizontalHeaderLabels(['X / roll / #1', 'Y / pitch / #2', 'Z / yaw / #3'])
    state_table.setVerticalHeaderLabels([label for label, _key, _n in _TABLE_ROWS])
    state_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    state_table.setEditTriggers(QTableWidget.NoEditTriggers)
    state_table.setMaximumWidth(430)
    for _r in range(state_table.rowCount()):
        for _c in range(3):
            state_table.setItem(_r, _c, QTableWidgetItem(''))

    container = QWidget()
    container.setWindowTitle('imu_base_fusion')
    outer_layout = QHBoxLayout(container)
    outer_layout.addWidget(win, 3)
    outer_layout.addWidget(state_table, 2)
    container.resize(1480, 480)
    container.show()

    def _trim_window():
        """Descarta de rec todo lo más viejo que _WINDOW_S y poda las elipses
        que se quedaron fuera. Coste acotado: como mucho recorre lo que entró
        en un ciclo de redibujado, nunca la sesión completa."""
        if not rec['t']:
            return
        cutoff = rec['t'][-1] - _WINDOW_S
        i = 0
        n = len(rec['t'])
        while i < n and rec['t'][i] < cutoff:
            i += 1
        if i > 0:
            for k in rec:
                del rec[k][:i]
        while ell_items and ell_items[0][0] < cutoff:
            _, old_item = ell_items.pop(0)
            traj.removeItem(old_item)

    def _drain_and_redraw():
        got_data = False
        latest_table = None
        while True:
            try:
                item = local_queue.get_nowait()
            except queue.Empty:
                break
            if item is None:
                app.quit()
                return
            if item.get('table') is not None:
                latest_table = item['table']  # solo importa el más reciente, no se acumula
            n_new = len(item.get('t', []))
            for k in rec:
                rec[k].extend(item.get(k, []))
            state['total_rows'] += n_new
            got_data = True

        # Independiente del resto: puede llegar sin filas nuevas de trayectoria
        # (p.ej. antes de la alineación con room_concept, ver _write_dsr).
        if latest_table is not None:
            _update_state_table(state_table, latest_table)

        if not got_data or not rec['t']:
            return

        _trim_window()
        if not rec['t']:
            return

        xs, ys, n = rec['x'], rec['y'], len(rec['x'])
        traj_curve.setData(xs, ys)
        trail_curve.setData(xs[-_TRAIL_LEN:], ys[-_TRAIL_LEN:])
        traj_dot.setData([xs[-1]], [ys[-1]])
        odom_curve.setData(rec['x_odom'], rec['y_odom'])

        # Ver nota histórica en el diseño original: angle=180-yaw_deg y punta
        # desplazada 0.20m hacia delante (setPos fija la punta, no la cola).
        yaw_rad = float(rec['yaw'][-1])
        yaw_deg = np.degrees(yaw_rad)
        arrow_len_m = 0.20
        tip_x = xs[-1] + arrow_len_m * np.cos(yaw_rad)
        tip_y = ys[-1] + arrow_len_m * np.sin(yaw_rad)
        traj_arrow.setPos(tip_x, tip_y)
        traj_arrow.setStyle(angle=180 - yaw_deg)

        # Elipses cada _ELLIPSE_EVERY filas RECIBIDAS (contador acumulativo,
        # ajeno al recorte de ventana) -- se dibujan sobre el punto más
        # reciente disponible, no sobre el índice histórico exacto (la
        # ventana móvil ya descartó ese pasado; para eso está el CSV si hace
        # falta fidelidad completa, ver Fusion.enable_recording).
        target_ell = state['total_rows'] // _ELLIPSE_EVERY
        while state['ell_created'] < target_ell:
            state['ell_created'] += 1
            rx = 3 * rec['sx'][-1]
            ry = 3 * rec['sy'][-1]
            cx, cy = xs[-1], ys[-1]
            theta = np.linspace(0, 2 * np.pi, 48)
            yaw_i = rec['yaw'][-1]
            ex = rx * np.cos(theta) * np.cos(yaw_i) - ry * np.sin(theta) * np.sin(yaw_i) + cx
            ey = rx * np.cos(theta) * np.sin(yaw_i) + ry * np.sin(theta) * np.cos(yaw_i) + cy
            item = traj.plot(ex, ey, pen=pg.mkPen(color=(70, 130, 180, 120), width=0.8), fillLevel=None)
            ell_items.append((rec['t'][-1], item))

        ts = rec['t']
        nis_curve.setData(ts, rec['nis'])
        sig_m = [(sx + sy) / 2 for sx, sy in zip(rec['sx'], rec['sy'])]
        sig_curve.setData(ts, sig_m)
        # Fuerza el rango visible a la ventana de datos retenida -- el
        # autorange de pyqtgraph no siempre re-encuadra solo tras recortar
        # rec (y se desactiva si el usuario interactúa con el plot), así que
        # el eje t se fija explícitamente cada frame para que se vea deslizar
        # de verdad con los datos, como un osciloscopio.
        span = ts[-1] - ts[0]
        if span > 0:
            nis_plot.setXRange(ts[0], ts[-1], padding=0.02)
        nis_vb.setGeometry(nis_plot.vb.sceneBoundingRect())

    timer = QTimer()
    timer.timeout.connect(_drain_and_redraw)
    timer.start(_POLL_MS)

    app.exec()


if __name__ == '__main__':
    main()
