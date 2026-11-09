#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by Alejandro Torrejon Harto
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
import os
import board

console = Console(highlight=False)

BRIGHTNESS = 0.2
# SPI1 on the Orange Pi Zero 2W runs at ~6 MHz: one WS2812B bit is one SPI byte (see README)
SPI_FREQUENCY = 6000000
SPI_BIT0 = 0b11000000
SPI_BIT1 = 0b11111000


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        if startup_check:
            self.startup_check()
        else:
            self.pixels = {}
            self.ledStrip = self.create_strip(configData["LED"])
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""


    def create_strip(self, led):
        """Create the strip driver for the output selected in the LED config section."""
        number = int(led["Number"])
        if str(led.get("Interface", "GPIO")).upper() == "SPI":
            # Blinka's neopixel_write only supports the Raspberry Pi, so other boards use SPI MOSI
            import neopixel_spi
            return neopixel_spi.NeoPixel_SPI(board.SPI(), number, brightness=BRIGHTNESS, auto_write=False,
                                             frequency=SPI_FREQUENCY, bit0=SPI_BIT0, bit1=SPI_BIT1)
        import neopixel
        return neopixel.NeoPixel(getattr(board, f"D{led['GPIO']}"), number, brightness=BRIGHTNESS, auto_write=False)

    @QtCore.Slot()
    def compute(self):
        return True

    def startup_check(self):
        print(f"Testing RoboCompLEDArray.Pixel from ifaces.RoboCompLEDArray")
        test = ifaces.RoboCompLEDArray.Pixel()
        QTimer.singleShot(200, QApplication.instance().quit)




    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getLEDArray method from LEDArray interface
    #
    def LEDArray_getLEDArray(self):
        return dict(self.pixels)
    #
    # IMPLEMENTATION of setLEDArray method from LEDArray interface
    #
    def LEDArray_setLEDArray(self, pixelArray):
        ret = True
        for id, rgb in pixelArray.items():
            if 0 <= id < len(self.ledStrip):
                self.ledStrip[id] = (rgb.red, rgb.green, rgb.blue)
                self.pixels[id] = rgb
            else:
                ret = False
        self.ledStrip.show()
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompLEDArray you can use this types:
    # ifaces.RoboCompLEDArray.Pixel


