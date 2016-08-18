
import csv
import sys
import os.path
import math

# TODO pass type converion to params

if len(sys.argv) == 1:
	print 'I need filename to convert!\n'
elif len(sys.argv) != 2:
        print 'wrong parameters!'
else:
	if(os.path.isfile(sys.argv[1]) is False):
		print 'The ' + str(sys.argv[1]) + ' not exist in the local disk\n'
	else:
		if (os.access(sys.argv[1], os.R_OK) is False):
			print 'The ' + str(sys.argv[1]) + ' exist in the local disk but I cant read\n'
		else:
			# world conversion value: robocomp2rcis or cgr2rcis
			conversion = 'robocomp2rcis'
			head='<innerModel>\n\t<transform id="world">\n\t\t<include path="/home/robocomp/robocomp/components/robocomp-shelly/etc/shelly.xml" />\n\t\t<transform id="floor">\n\t\t\t<plane id="back" ny="1" size="30000,30000,10" texture="/home/robocomp/robocomp/files/osgModels/textures/wood.jpg" />\n\t\t</transform>\n'
			tail='\n\t</transform>\n</innerModel>'
			
			texture = '/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg'
			idObject = 0
			content = ''
			with open(sys.argv[1]) as csvfile:
				contentCSV = csv.reader(csvfile, delimiter=',')
				lines = []
				for data in contentCSV:
					lines.append(data)
				for line in lines:
					x1 = float(line[0])*1000
                                        y1 = float(line[1])*1000
                                        x2 = float(line[2])*1000
                                        y2 = float(line[3])*1000
					if conversion == 'robocomp2rcis':
						tx = (x1 + x2)/2
                                                ty = 400
                                                tz = (y1 + y2)/2
                                                width = math.sqrt( (x2 - x1) ** 2 + (y2 - y1) ** 2 )
					else: # cgr2rcis
						tx = -(y1 + y2)/2
						ty = 400
						tz = (x1 + x2)/2
						width = math.sqrt( (x2 - x1) ** 2 + (y2 - y1) ** 2 )
					nx = -(y2 - y1)
					nz = (x2 - x1)
					content += '\t\t<transform id="pared'+str(idObject)+'" tx="'+str(tx)+'" tz="'+str(tz)+'" ty="400" >\n'
					content += '\t\t\t<plane id="muro'+str(idObject)+'" nx="'+str(nx)+'" nz="'+str(nz)+'" size="'+str(width)+',800"  texture="'+texture+'" />\n'
					content += '\t\t</transform>\n'
					idObject += 1

			name = (os.path.basename(sys.argv[1])).split('.')
			filetarget = open(name[0]+'_rcis.xml', 'w')
			filetarget.write(head)
			filetarget.write(content)
			filetarget.write(tail)
			filetarget.close()
