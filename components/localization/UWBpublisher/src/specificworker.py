#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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

from genericworker import *
from PySide2.QtWidgets import QApplication
import decawave_ble as deca

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        self.timer.start(self.Period)

        self.defaultMachine.start()
        self.destroyed.connect(self.t_compute_to_finalize)

        self.devicelist = []

    def __del__(self):
        print('SpecificWorker destructor')
        self.t_compute_to_finalize.emit()

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    @QtCore.Slot()
    def compute(self):
        data = None
        for x in range(4):
            try:
                data = deca.get_data_multiple_devices(self.devicelist)
                if data is not None:
                    break
            except Ice.Exception as e:
                traceback.print_exc()
                print(e)
                self.t_compute_to_finalize.emit()
                break
            except (KeyboardInterrupt, SystemExit, SystemError) as e:
                traceback.print_exc()
                print(e)
                self.t_compute_to_finalize.emit()
                break
            except:
                continue

        taglist = []
        if data is not None:
            for key, device in data.items():
                if device is not None and "location_data" in device:
                    position = PointUWB()
                    if device["location_data"] is None:
                        continue
                    if device["location_data"]["position_data"] is None:
                        continue
                    position.deviceName = key
                    position.x = float(device["location_data"]["position_data"]["x_position"])
                    position.y = float(device["location_data"]["position_data"]["y_position"])
                    position.z = float(device["location_data"]["position_data"]["z_position"])
                    position.tag = not bool(device["operation_mode_data"]["device_type"])
                    taglist.append(position)
            try:
                self.uwbsimple_proxy.getTagPositions(taglist)
            except Ice.Exception as e:
                print(e)
                traceback.print_exc()
                print(e)
            except Exception as e:
                print(e)
        return True

    # =============== Slots methods for State Machine ===================
    # ===================================================================
    #
    # sm_initialize
    #
    @QtCore.Slot()
    def sm_initialize(self):
        print("Entered state initialize")
        if os.getuid() != 0:
            print("You need root privileges to run this component\nTry executing using 'sudo' before the call")
            self.t_initialize_to_finalize.emit()
        else:
            _original = deca.is_decawave_scan_entry

            def is_decawave_scan_entry(scan_entry):
                for (adtype, desc, value) in scan_entry.getScanData():
                    if adtype == 33 or desc == "128b Service Data" or "e72913c2a1" in value:
                        return True
                    continue
                return _original(scan_entry)

            deca.is_decawave_scan_entry = is_decawave_scan_entry

            self.devicelist = deca.scan_for_decawave_devices()  # scan_for_decawave_devices()
            anchor_devices = {}
            tag_devices = {}
            for k, dev in self.devicelist.items():
                if dev is not None:
                    for x in range(4):
                        try:
                            data = deca.get_data(dev)
                            if data["operation_mode_data"]["device_type"] == 0:
                                tag_devices[k] = dev
                            elif data["operation_mode_data"]["device_type"] == 1:
                                anchor_devices[k] = dev
                            break
                        except Ice.Exception as e:
                            traceback.print_exc()
                            print(e)
                            break
                        except (KeyboardInterrupt, SystemExit, SystemError) as e:
                            traceback.print_exc()
                            print(e)
                            break
                        except:
                            continue
            if len(tag_devices) > 1:
                self.devicelist = tag_devices
                print("Found ", len(self.devicelist), " devices")
            else:
                print("There's no tag devices connected")
                self.t_initialize_to_finalize.emit()

            self.t_initialize_to_compute.emit()
            print('SpecificWorker.compute...')

    #
    # sm_compute
    #
    @QtCore.Slot()
    def sm_compute(self):
        self.compute()


    #
    # sm_finalize
    #
    @QtCore.Slot()
    def sm_finalize(self):
        print("Entered state finalize")
        # QApplication.quit()

# =================================================================
# =================================================================
