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

import curses
import traceback

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication

from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

try:
    import pygame
    from pygame.locals import *
    pygame_flag = True  # Flag for presence of pygame
    print("PyGame imported...")
except ImportError as e:
    pygame_flag = False
    print("PyGame not installed. Using ncurses ...")



class SpecificWorker(GenericWorker):

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        # get the curses self.screen window
        self.screen = curses.initscr()
        # turn off input echoing
        curses.noecho()
        # respond to keys immediately (don't wait for enter)
        curses.cbreak()
        # map arrow keys to special values
        self.screen.keypad(True)
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
                              'Connected to robot. Use arrows (or) ASWD to control direction, Shift to speed up, space bar to stop and ''q'' to exit')
                self.adv_command = 'STOP'
                self.rot_command = 'STOP'
            else:
                self.screen.addstr(0, 0,
                              'Connected to robot. Use arrows to control speed, space bar to stop and ''q'' to exit')
            self.screen.refresh()
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

            # tt1=input("max :")
            # tt2=input("min :")

    def __del__(self):
        pass
        # self.end_run()

    def end_run(self, final_string):
        print("Ending...")
        try:
            if pygame_flag:
                pygame.quit()
            if curses is not None:
                self.screen.clear()
                curses.endwin()
                curses.echo()
                curses.nocbreak()
            print(final_string)
            self.timer.stop()
            QtCore.QCoreApplication.instance().quit()
        except TypeError:
            print("end_run: Problems ending process")

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        if pygame_flag:
            result = self.pygame_compute()
        else:
            result = self.ncurses_compute()
        self.screen.refresh()
        if result is True:
            try:
                self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
            except Ice.Exception as e:
                traceback.print_exc()
                print(e)
                self.end_run("Could not connect to any differential robot proxy. Is the component or RCIS started?")
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def pygame_compute(self):
        # control algorithm for presence of pygame

        for event in pygame.event.get():
            self.screen.move(2, 0)
            self.screen.clrtoeol()
            self.screen.addstr(2, 0, f'pygame compute {pygame.event.event_name(event.type)}')
            if event.type == QUIT:
                self.end_run("Terminado")

            if event.type == KEYDOWN:
                self.screen.addstr(3, 0, f'Key: {pygame.key.name(event.key)}')
                if event.key == pygame.K_UP or event.key == pygame.K_w:
                    if self.adv < self.tt1:
                        self.adv += 300
                    self.adv_command = 'UP'

                if event.key == pygame.K_DOWN or event.key == pygame.K_s:
                    if self.adv > -1 * self.tt1:
                        self.adv -= 300
                    self.adv_command = 'DOWN'

                if event.key == pygame.K_RIGHT or event.key == pygame.K_d:
                    if self.adv >= 0:
                        if self.rot < self.tt2:
                            self.rot += 0.25
                    else:
                        if self.rot > -1 * self.tt2:
                            self.rot -= 0.25
                    self.rot_command = "RIGHT"

                if event.key == pygame.K_LEFT or event.key == pygame.K_a:
                    if self.adv >= 0:
                        if self.rot > -1 * self.tt2:
                            self.rot -= 0.25
                    else:
                        if self.rot < self.tt2:
                            self.rot += 0.25
                    self.rot_command = "LEFT"

                if event.key == pygame.K_LSHIFT or event.key == pygame.K_RSHIFT:
                    self.adv *= 2
                    self.rot *= 1.7

                if pygame.key.get_pressed()[K_SPACE]:
                    self.rot = 0
                    self.adv = 0
                    self.adv_command = 'STOP'
                    self.rot_command = 'STOP'

                if event.key == pygame.K_q or event.key == ord('q'):
                    self.end_run("Terminado")
                self.screen.addstr(5, 0, self.adv_command + ': ' + '%.2f' % self.adv)
                self.screen.clrtoeol()
                self.screen.addstr(6, 0, self.rot_command + ': ' + '%.2f' % self.rot)
                self.screen.clrtoeol()
                self.screen.refresh()
                return True
            if event.type == KEYUP:
                self.screen.move(3, 0)
                self.screen.clrtoeol()
                if event.key == pygame.K_DOWN or event.key == pygame.K_UP or event.key == pygame.K_s or event.key == pygame.K_w:
                    self.adv = 0
                    self.adv_command = 'STOP'
                if event.key == pygame.K_LEFT or pygame.K_RIGHT or event.key == pygame.K_d or event.key == pygame.K_a:
                    # if self.rot < self.tt2:
                    self.rot = 0
                    self.rot_command = 'STOP'
                if event.key == pygame.K_LSHIFT or event.key == pygame.K_RSHIFT:
                    self.adv /= 2
                    self.rot /= 1.7
                self.screen.addstr(5, 0, self.adv_command + ': ' + '%.2f' % self.adv)
                self.screen.clrtoeol()
                self.screen.addstr(6, 0, self.rot_command + ': ' + '%.2f' % self.rot)
                self.screen.clrtoeol()
                self.screen.refresh()
                return True
            else:
                return False

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
    # From the RoboCompDifferentialRobot you can call this methods:
    # self.differentialrobot_proxy.correctOdometer(...)
    # self.differentialrobot_proxy.getBasePose(...)
    # self.differentialrobot_proxy.getBaseState(...)
    # self.differentialrobot_proxy.resetOdometer(...)
    # self.differentialrobot_proxy.setOdometer(...)
    # self.differentialrobot_proxy.setOdometerPose(...)
    # self.differentialrobot_proxy.setSpeedBase(...)
    # self.differentialrobot_proxy.stopBase(...)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # RoboCompDifferentialRobot.TMechParams
