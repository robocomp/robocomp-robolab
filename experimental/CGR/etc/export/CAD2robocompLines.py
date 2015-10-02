
import sys
import os.path
import locale

locale.setlocale(locale.LC_ALL, 'en_US.utf8')

if len(sys.argv) == 1:
	print 'I need filename to convert!\n'
elif len(sys.argv) != 2:
        print 'wrong parameters!'
else:
	if(os.path.isfile(sys.argv[1]) is False):
		print 'The ' + str(sys.argv[1]) + ' not exist in the local disk\n'
	else:
		# TODO speedup this section
		if (os.access(sys.argv[1], os.R_OK) is True):
			print 'Openning ' + str(sys.argv[1]) + '!\n'
			infile = open(sys.argv[1], 'r')
			listNumber1=[]
			listNumber2=[]
			for line in infile:
				if 'de punto' in line:
					line = line.split(' ')
					row=''
					cont=0
					for word in line:
						try:
							num=float(word)
							row += str(num) +','
							cont +=1
							if cont % 3 is 0:
								row = row[:-1]
								listNumber1.append(row)
								row=''
						except ValueError:
							pass
				elif 'hasta punto' in line:
					line = line.split(' ')
					row=''
					cont=0
					for word in line:
						try:
							num=float(word)
							row += str(num) +','
							cont +=1
							if cont % 3 is 0:
								row = row[:-1]
								listNumber2.append(row)
								row=''
						except ValueError:
							pass
			infile.close()
			if(len(listNumber1) != len(listNumber2)):
				print ('error in file')
			else:
				for i in range(len(listNumber1)):
					listNumber = listNumber1[i].split(',')
					line=''
					for point in range(len(listNumber) - 1):
						line += str(listNumber[point]) +','
					listNumber = listNumber2[i].split(',')
					for point in range(len(listNumber) - 1):
						line += str(listNumber[point]) +','
					line = line[:-1]
					print line