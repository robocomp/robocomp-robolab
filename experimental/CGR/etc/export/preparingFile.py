
import csv
import sys
import os.path

# TODO pass type converion to params

print "example of transform origin \n"
print "ALERT: first move, after change orientation !!!!!!!!!! \n"
print "python preparingFile.py filename moveInX moveInY changeOrrientationX changeOrrientationY doRotateAxisRockin\n"
print "python preparingFile.py rockin0.txt 3 4.5 False True True\n"
print "------------------------------------------------ \n"
print "------------------------------------------------ \n"
print "------------------------------------------------ \n"

if len(sys.argv) == 1:
	print 'I need filename to convert!\n'
elif len(sys.argv) != 7:
        print 'wrong parameters!'
else:
	if(os.path.isfile(sys.argv[1]) is False):
		print 'The ' + str(sys.argv[1]) + ' not exist in the local disk\n'
	else:
		if (os.access(sys.argv[1], os.R_OK) is False):
			print 'The ' + str(sys.argv[1]) + ' exist in the local disk but I cant read\n'
		else:
			moveInX = float(sys.argv[2])
			moveInY = float(sys.argv[3])
			changeOrientationX = sys.argv[4]
			changeOrientationY = sys.argv[5]
			doRotateAxisRockin = sys.argv[6]

			content = ''
			lines = []
			with open(sys.argv[1]) as csvfile:
				contentCSV = csv.reader(csvfile, delimiter=',')
				for data in contentCSV:
					if not '#' in data[0]:
						lines.append(data)
				error = False
				for line in lines:
					x1 = "{:.3f}".format(float(line[0]) + moveInX)
					y1 = "{:.3f}".format(float(line[1]) + moveInY)
					x2 = "{:.3f}".format(float(line[2]) + moveInX)
					y2 = "{:.3f}".format(float(line[3]) + moveInY)
					
					if changeOrientationX == "True":
						if '-' == x1[0]:
							x1 = x1.replace("-", "")
						else:
							x1 = '-' + x1
						if '-' == x2[0]:
							x2 = x2.replace("-", "")
						else:
							x2 = '-' + x2
					elif changeOrientationX == "False":
						pass
					else:
						print "Wrong change orientation in X param!\n"
						error = True
						break
					if changeOrientationY == "True":
						if "-" == y1[0]:
							y1 = y1.replace("-", "")
						else:
							y1 = "-" + y1
						if "-" == y2[0]:
							y2 = y2.replace("-", "")
						else:
							y2 = "-" + y2

					elif changeOrientationY == "False":
						pass
					else:
						print "Wrong change orientation in Y param!\n"
						error = True
						break

					if error is not True:
						if doRotateAxisRockin == "False":
							content += str(x1) + ',' + str(y1) + ',' + str(x2) + ',' + str(y2) + '\n'
						else:
							aux1 = y1
							aux2 = y2
							y1 = x1
							y2 = x2
							if "-" == aux1[0]:
								aux1 = aux1.replace("-", "")
							else:
								aux1 = "-" + aux1
							if "-" == aux2[0]:
								aux2 = aux2.replace("-", "")
							else:
								aux2 = "-" + aux2
							content += str(aux1) + ',' + str(y1) + ',' + str(aux2) + ',' + str(y2) + '\n'
					else:
						print "Not export file by error\n"
					
				name = (os.path.basename(sys.argv[1])).split('.')
				filetarget = open(name[0]+'_vector.txt', 'w')
				filetarget.write(content)
				filetarget.close()
