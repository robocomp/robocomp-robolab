#
#    Copyright (C) 2026 by YOUR NAME HERE
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

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


slice_dir = os.path.dirname(os.path.abspath(__file__))
try:
    # Ice 3.8
    Ice.loadSlice(["-I", slice_dir, os.path.join(slice_dir, "Power.ice")])
except (TypeError):
    # Ice 3.7
    Ice.loadSlice(f'-I {slice_dir} {os.path.join(slice_dir, "Power.ice")}')


from RoboCompPower import *

class PowerI(Power):
    def __init__(self, worker, id:str):
        self.worker = worker
        self.id = id


    def getPower(self, name, ice):
        return getattr(self.worker, f"Power{self.id}_getPower")(name)

    def getPowers(self, ice):
        return getattr(self.worker, f"Power{self.id}_getPowers")()
