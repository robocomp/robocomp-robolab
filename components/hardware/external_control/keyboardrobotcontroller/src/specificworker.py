#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import RoboCompJoystickAdapter
import curses

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

try:
    import pygame
    from pygame.locals import *
    pygame_flag = True  # Flag for presence of pygame
    print("PyGame imported...")
except ImportError as e:
    pygame_flag = False
    print("PyGame not installed. Using ncurses ...")


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.screen = None

        try:
            # get the curses self.screen window
            self.screen = curses.initscr()
            # turn off input echoing
            curses.noecho()
            # respond to keys immediately (don't wait for enter)
            curses.cbreak()
            # map arrow keys to special values
            self.screen.keypad(True)
            # make getch() non-blocking so the Qt event loop is never frozen
            self.screen.nodelay(True)
        except Exception as e:
            self._curses_restore()
            raise RuntimeError(f"Failed to initialise curses: {e}") from e

        self.Period = 1
        self.adv_command = 'STOP'
        self.rot_command = 'STOP'

        if startup_check:
            self.startup_check()
        else:
            self.adv = 0
            self.rot = 0
            self.tt1 = 2000
            self.tt2 = 2
            if pygame_flag:
                pygame.init()
                pygame.display.set_mode((1, 1), pygame.NOFRAME)
                self.screen.addstr(0, 0,
                              'Connected to robot. Use arrows (or) ASWD to control direction, Shift to speed up, space bar to stop and q to exit')
                self.adv_command = 'STOP'
                self.rot_command = 'STOP'
            else:
                self.screen.addstr(0, 0,
                              'Connected to robot. Use arrows to control speed, space bar to stop and q to exit')
            self.screen.refresh()
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    @staticmethod
    def _curses_restore():
        """Restore terminal to a sane state regardless of how far curses got."""
        try:
            curses.echo()
            curses.nocbreak()
            curses.endwin()
        except Exception:
            pass

    def __del__(self):
        """Destructor"""
        self._curses_restore()

    def end_run(self, final_string):
        print("Ending...")
        try:
            if pygame_flag:
                pygame.quit()
            if self.screen is not None:
                self.screen.clear()
            self._curses_restore()
            print(final_string)
            self.timer.stop()
            QtCore.QCoreApplication.instance().quit()
        except Exception:
            print("end_run: Problems ending process")

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        # Keyboard is always read via curses (non-blocking) because it reads
        # from the terminal stdin where the user is typing. Pygame cannot
        # receive keyboard events from a 1x1 background window.
        if pygame_flag:
            self.pygame_pump()   # service pygame's event queue (joystick, quit, etc.)
        result = self.ncurses_compute()
        self.screen.refresh()
        if result is True:
            try:
                data = RoboCompJoystickAdapter.TData()
                data.id = "keyboard"
                adv_axis = RoboCompJoystickAdapter.AxisParams()
                adv_axis.name = "advance"
                adv_axis.value = float(self.adv)
                rot_axis = RoboCompJoystickAdapter.AxisParams()
                rot_axis.name = "rotate"
                rot_axis.value = float(self.rot)
                data.axes = [adv_axis, rot_axis]
                data.buttons = []
                self.joystickadapter_proxy.sendData(data)
            except Ice.Exception as e:
                print(e)
        return True

    def startup_check(self):
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)

    def pygame_pump(self):
        """Service pygame's event queue to handle QUIT and keep it alive.
        Keyboard input is handled by ncurses_compute() instead, because pygame
        cannot receive keyboard events from a background/unfocused window."""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.end_run("Terminado")

    def ncurses_compute(self):
        # control algorithm for absence of pygame
        self.screen.addstr(2, 0, 'ncurses compute')
        self.screen.refresh()
        key = self.screen.getch()
        if key == curses.KEY_UP or key == ord('w'):
            if self.adv > self.tt1:
                self.adv = self.adv
            else:
                self.adv = self.adv + 20
            self.screen.addstr(5, 0, 'up: ' + '%.2f' % self.adv + ' : ' + '%.2f' % self.rot)
            return True

        elif key == curses.KEY_DOWN or key == ord('s'):
            if self.adv < -1 * self.tt1:
                self.adv = self.adv
            else:
                self.adv = self.adv - 20
            self.screen.addstr(5, 0, 'down: ' + '%.2f' % self.adv + ' : ' + '%.2f' % self.rot)
            return True

        elif key == curses.KEY_LEFT or key == ord('a'):
            if self.rot < -1 * self.tt2:
                self.rot = self.rot
            else:
                self.rot = self.rot - 0.1
            self.screen.addstr(5, 0, 'left: ' + '%.2f' % self.adv + ' : ' + '%.2f' % self.rot)
            return True

        elif key == curses.KEY_RIGHT or key == ord('d'):
            if self.rot > self.tt2:
                self.rot = self.rot
            else:
                self.rot = self.rot + 0.1
            self.screen.addstr(5, 0, 'right: ' + '%.2f' % self.adv + ' : ' + '%.2f' % self.rot)
            return True

        elif key == ord(' '):
            self.rot = 0
            self.adv = 0
            self.screen.addstr(5, 0, 'stop: ' + '%.2f' % self.adv + ' : ' + '%.2f' % self.rot)
            return True

        elif key == ord('q'):
            self.end_run("Terminado")
        return 0

    ######################
    # From the RoboCompJoystickAdapter you can publish to this topic:
    # self.joystickadapter_proxy.sendData(RoboCompJoystickAdapter.TData)
    #
    # RoboCompJoystickAdapter.TData fields:
    #   id      : string
    #   axes    : list of AxisParams  { name: string, value: float }
    #   buttons : list of ButtonParams { name: string, step: int }
