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

import sys, os, Ice, traceback
import blescan
import sys
import bluetooth._bluetooth as bluez

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"Beacons.ice")
from RoboCompBeacons import *


from beaconsI import *

class SpecificWorker(GenericWorker):
  	potord = []
	indexord = []
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		potencia, indice = self.funcion()
		self.indexord, self.potord = self.ordenar(indice, potencia)
		print "potenciaorde"
		print self.potord
		return True


	#
	# getBeacons
	#
	def getBeacons(self):
		#
		# YOUR CODE HERE
		#
		return self.indexord, self.potord
		
	def dicc(self, major, minor, idc):
            c=0
            beacons = {}
            beacons[1,1,"f15"] = ["11"]
            beacons[1,2,"c58"] = ["12"]
            beacons[2,1,"094"] = ["21"]
            beacons[3,1,"5fc"] = ["31"]
            beacons[4,1,"286"] = ["41"] 
            beacons[4,2,"218"] = ["42"]   
            idbeacons = [0]       
            
            if(major,minor,idc) in beacons.keys():
                    idbeacons = beacons[major, minor, idc]
                    c = 1
            return idbeacons,c
	
	def funcion(self):
    
            dev_id = 0
            try:
                sock = bluez.hci_open_dev(dev_id)
                print "ble thread started"
        
            except:
                print "error accessing bluetooth device..."
                sys.exit(1)
        
            blescan.hci_le_set_scan_parameters(sock)
            blescan.hci_enable_le_scan(sock)
            cont = 0
            #daydatetime timecurrent y luego el tiempo en cada tramo  
            returnedList = blescan.parse_events(sock, 5)
            potencia = []
            indice = []
            for allbeacons in returnedList:      
                words = allbeacons.split(',')  
                idb = words[1][-3:]
                idbeacons,hayBeacon = self.dicc(int(words[2]),int(words[3]),idb)
        
                if hayBeacon == 0:  
                
                    idbeacons = [0]
                    indice.append(int(cont))
                    potencia.append(0) 
                    cont = cont+1;
            
                if hayBeacon == 1:
                    if int(idbeacons[0]) in indice:
                        indice.append(int(cont))
                        potencia.append(0)
                    else:
                        indice.append(int(idbeacons[0]))
                        aniadir = 100 + (int(words[5]))
                        #potencia.append(-int(words[5]))
                        potencia.append(aniadir)
                    cont = cont+1;
                            
            return potencia, indice
	
	def ordenar(self, lista1, lista2):
            menor = 9999
            listaordenada = []
            listaordenada2 = []
            i = 0
            while  len(lista1)>0:
                while i < len(lista1):
                    if menor > lista1[i]:
                        menor = lista1[i]
                        otro = lista2[i]
                        
                    i=i+1
                listaordenada.append(menor)
                listaordenada2.append(otro)
                lista1.remove(int(menor))
                lista2.remove(int(otro))
                menor=9999
                i=0
            
            return listaordenada,listaordenada2

