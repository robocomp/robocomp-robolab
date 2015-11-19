
import csv
import sys
import os.path

# TODO pass type converion to params

print "example of transform origin \n"
print "ALERT: first move, after change orientation !!!!!!!!!! \n"
print "python preparingFile.py filename moveInX moveInY changeOrrientationX changeOrrientationY \n"
print "python preparingFile.py rockin0.txt 3 4.5 False True \n"
print "------------------------------------------------ \n"
print "------------------------------------------------ \n"
print "------------------------------------------------ \n"

if len(sys.argv) == 1:
	print 'I need filename to convert!\n'
elif len(sys.argv) != 6:
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
						x1 = '-' + x1
						x2 = '-' + x2
					elif changeOrientationX == "False":
						pass
					else:
						print "Wrong change orientation in X param!\n"
						error = True
						break
					if changeOrientationY == "True":
						y1 = '-' + y1
						y2 = '-' + y2
					elif changeOrientationY == "False":
						pass
					else:
						print "Wrong change orientation in Y param!\n"
						error = True
						break
						#data[0] += moveInX
						#data[1] += moveInY
						#data[2] += moveInX
						#data[3] += moveInY
						#if changeOrientationX is True:
							#data[0] *= -1
							#data[2] *= -1
						#if changeOrientationY is True:
							#data[1] *= -1
							#data[3] *= -1
						#content+= str(data).strip('[\'\']')+'\n'
					if error is not True:
						content += str(x1) + ',' + str(y1) + ',' + str(x2) + ',' + str(y2) + '\n'
					else:
						print "Not export file by error\n"
					
				name = (os.path.basename(sys.argv[1])).split('.')
				filetarget = open(name[0]+'_veector.txt', 'w')
				filetarget.write(content)
				filetarget.close()
