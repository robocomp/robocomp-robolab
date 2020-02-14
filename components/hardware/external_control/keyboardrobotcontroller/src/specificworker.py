#
# Copyright (C) 2015 by YOUR NAME HERE
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

import sys, os, Ice, traceback, curses
from PySide import *
from genericworker import *

# get the curses screen window
screen = curses.initscr()
 
# turn off input echoing
curses.noecho()
 
# respond to keys immediately (don't wait for enter)
curses.cbreak()
 
# map arrow keys to special values
screen.keypad(True)
 
ROBOCOMP = ''

try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"DifferentialRobot.ice")
from RoboCompDifferentialRobot import *
Ice.loadSlice(preStr+"Laser.ice")
from RoboCompLaser import *

class SpecificWorker(GenericWorker):
        adv = 0
        rot = 0
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 1
		self.timer.start(self.Period)
		screen.addstr(0,0,'Connected to robot. Use arrows to control speed, space bar to stop ans ''q'' to exit')
		#tt1=input("max :")
		#tt2=input("min :")

	def setParams(self, params):
		return True

	tt1=2000
	tt2=2
	@QtCore.Slot()
	def compute(self):
	    try:
                key = screen.getch()
            
                if key == curses.KEY_UP:
		    if self.adv  > self.tt1 :
		    	self.adv=self.adv
	            else :
                    	self.adv = self.adv + 20;
                    screen.addstr(5, 0, 'up: '+ '%.2f' % self.adv + ' : ' + '%.2f' % self.rot)
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == curses.KEY_DOWN:
		    if self.adv < -1*self.tt1 :
	            	self.adv=self.adv
		    else :
                    	self.adv = self.adv - 20;
                    screen.addstr(5, 0, 'down: '+ '%.2f' % self.adv+ ' : ' + '%.2f' % self.rot)
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == curses.KEY_LEFT:
		    if self.rot < -1*self.tt2 :
	            	self.rot=self.rot
		    else :
                    	self.rot = self.rot - 0.1;
                    screen.addstr(5, 0, 'left: '+ '%.2f' % self.adv+ ' : ' + '%.2f' % self.rot)
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == curses.KEY_RIGHT:
		    if self.rot > self.tt2 :
		    	self.rot=self.rot
		    else :
                    	self.rot = self.rot + 0.1;
                    screen.addstr(5, 0, 'right: '+ '%.2f' % self.adv+ ' : ' + '%.2f' % self.rot)
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == ord(' '):
                    self.rot = 0;
                    self.adv = 0;
                    screen.addstr(5, 0, 'stop: '+ '%.2f' % self.adv+ ' : ' + '%.2f' % self.rot)
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
               	elif key == ord('q'):
		    curses.endwin()
		    sys.exit() 
            except Ice.Exception, e:
		curses.endwin()
                traceback.print_exc()
                print e
            return True





