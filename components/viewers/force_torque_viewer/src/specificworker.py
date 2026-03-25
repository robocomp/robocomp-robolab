#!/usr/bin/python3
# -*- coding: utf-8 -*-

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import collections
import numpy as np
import matplotlib
matplotlib.use("QtAgg")  # backend Qt para PySide6
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

# ── Configuración ─────────────────────────────────────────────────────────────
HISTORY_SECONDS = 10
PLOT_HZ         = 20    # refresco de la gráfica (ms → ver timer_plot)

# ── Estilo ────────────────────────────────────────────────────────────────────
BG       = "#0d1117"
PANEL_BG = "#161b22"
GRID_C   = "#21262d"
TEXT_C   = "#c9d1d9"

FORCE_COLORS  = {"Fx": "#e05252", "Fy": "#52b0e0", "Fz": "#52e08a"}
TORQUE_COLORS = {"Tx": "#e0a852", "Ty": "#a052e0", "Tz": "#e0e052"}
TEMP_COLOR    = "#e07852"


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        if startup_check:
            self.startup_check()
            return

        # ── Buffers de datos ──────────────────────────────────────────────────
        maxlen = HISTORY_SECONDS * PLOT_HZ
        self.t0        = None
        self.timestamps   = collections.deque(maxlen=maxlen)
        self.forces       = {k: collections.deque(maxlen=maxlen) for k in ("Fx","Fy","Fz")}
        self.torques      = {k: collections.deque(maxlen=maxlen) for k in ("Tx","Ty","Tz")}
        self.temperatures = collections.deque(maxlen=maxlen)

        # ── Construir ventana con la gráfica ──────────────────────────────────
        self._build_plot_window()

        # ── Timer de lectura del sensor ───────────────────────────────────────
        self.timer.timeout.connect(self.compute)
        self.timer.start(self.Period)

        # ── Timer de refresco de la gráfica (independiente del compute) ───────
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self._update_plot)
        self.timer_plot.start(1000 // PLOT_HZ)   # ~50 ms
        

    def __del__(self):
        pass

    # ── Construcción de la figura ─────────────────────────────────────────────
    def _build_plot_window(self):
        self.fig = plt.figure(figsize=(13, 7), facecolor=BG)
        self.fig.suptitle("Bota FT Sensor — Live Monitor",
                          color=TEXT_C, fontsize=13, fontfamily="monospace", y=0.97)

        gs = self.fig.add_gridspec(3, 1, hspace=0.5,
                                   left=0.07, right=0.97,
                                   top=0.92, bottom=0.07)
        self.ax_f = self.fig.add_subplot(gs[0])
        self.ax_t = self.fig.add_subplot(gs[1])
        self.ax_T = self.fig.add_subplot(gs[2])

        for ax, title in [(self.ax_f, "Forces  [N]"),
                          (self.ax_t, "Torques  [N·m]"),
                          (self.ax_T, "Temperature  [°C]")]:
            ax.set_facecolor(PANEL_BG)
            ax.tick_params(colors=TEXT_C, labelsize=8)
            ax.spines[:].set_color(GRID_C)
            ax.grid(color=GRID_C, linewidth=0.6, linestyle="--")
            ax.set_title(title, color=TEXT_C, fontsize=9,
                         fontfamily="monospace", loc="left", pad=4)
            ax.set_xlabel("t  [s]", color=TEXT_C, fontsize=8)

        # Líneas de fuerzas
        self.f_lines = {
            k: self.ax_f.plot([], [], lw=1.4, color=c, label=k)[0]
            for k, c in FORCE_COLORS.items()
        }
        # Líneas de torques
        self.t_lines = {
            k: self.ax_t.plot([], [], lw=1.4, color=c, label=k)[0]
            for k, c in TORQUE_COLORS.items()
        }
        # Línea de temperatura
        self.T_line, = self.ax_T.plot([], [], lw=1.4, color=TEMP_COLOR, label="Temp")

        for ax_obj, lines in [(self.ax_f, self.f_lines), (self.ax_t, self.t_lines)]:
            ax_obj.legend(loc="upper left", fontsize=8, framealpha=0.3,
                          facecolor=PANEL_BG, labelcolor=TEXT_C, edgecolor=GRID_C)
        self.ax_T.legend(loc="upper left", fontsize=8, framealpha=0.3,
                         facecolor=PANEL_BG, labelcolor=TEXT_C, edgecolor=GRID_C)

        # Embeber en ventana Qt
        self.canvas = FigureCanvas(self.fig)
        self.window = self
        self.window.setWindowTitle("Bota FT Sensor")
        self.window.setStyleSheet(f"background-color: {BG};")
        central = QWidget()
        layout  = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self.window.setCentralWidget(central)
        self.window.resize(1200, 700)
        self.window.show()

    # ── Lectura del sensor (timer de compute) ─────────────────────────────────
    @QtCore.Slot()
    def compute(self):
        try:
            data = self.forcetorquesensor_proxy.getSensorData()

            import time
            if self.t0 is None:
                self.t0 = time.time()
            now = time.time() - self.t0

            self.timestamps.append(now)
            self.forces["Fx"].append(data.fx)
            self.forces["Fy"].append(data.fy)
            self.forces["Fz"].append(data.fz)
            self.torques["Tx"].append(data.tx)
            self.torques["Ty"].append(data.ty)
            self.torques["Tz"].append(data.tz)
            self.temperatures.append(data.temperature)

        except Ice.Exception as e:
            console.print_exception()
            print(e)

        return True

    # ── Refresco de la gráfica (timer_plot) ───────────────────────────────────
    @QtCore.Slot()
    def _update_plot(self):
        if len(self.timestamps) < 2:
            return

        ts   = np.array(self.timestamps)
        fdat = {k: np.array(v) for k, v in self.forces.items()}
        tdat = {k: np.array(v) for k, v in self.torques.items()}
        temp = np.array(self.temperatures)

        x_min, x_max = ts[0], ts[-1]
        if x_max - x_min < 1.0:
            x_max = x_min + 1.0

        # Fuerzas
        for k, line in self.f_lines.items():
            line.set_data(ts, fdat[k])
        self.ax_f.set_xlim(x_min, x_max)
        all_f = np.concatenate(list(fdat.values()))
        if len(all_f):
            pad = (all_f.max() - all_f.min()) * 0.1 + 1.0
            self.ax_f.set_ylim(all_f.min() - pad, all_f.max() + pad)

        # Torques
        for k, line in self.t_lines.items():
            line.set_data(ts, tdat[k])
        self.ax_t.set_xlim(x_min, x_max)
        all_t = np.concatenate(list(tdat.values()))
        if len(all_t):
            pad = (all_t.max() - all_t.min()) * 0.1 + 0.1
            self.ax_t.set_ylim(all_t.min() - pad, all_t.max() + pad)

        # Temperatura
        self.T_line.set_data(ts, temp)
        self.ax_T.set_xlim(x_min, x_max)
        if len(temp):
            pad = (temp.max() - temp.min()) * 0.1 + 0.5
            self.ax_T.set_ylim(temp.min() - pad, temp.max() + pad)

        self.canvas.draw_idle()

    # ── Startup check ─────────────────────────────────────────────────────────
    def startup_check(self):
        print(f"Testing RoboCompForceTorqueSensor.SensorData from ifaces.RoboCompForceTorqueSensor")
        test = ifaces.RoboCompForceTorqueSensor.SensorData()
        QTimer.singleShot(200, QApplication.instance().quit)

    ######################
    # From the RoboCompForceTorqueSensor you can call this methods:
    # RoboCompForceTorqueSensor.SensorData self.forcetorquesensor_proxy.getSensorData()

    ######################
    # From the RoboCompForceTorqueSensor you can use this types:
    # ifaces.RoboCompForceTorqueSensor.SensorData
