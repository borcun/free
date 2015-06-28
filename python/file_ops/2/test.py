file = open( "test.txt", "r" )

while True:
	line = file.readline()

	if line == '':
		break
	else:
		print( line )
