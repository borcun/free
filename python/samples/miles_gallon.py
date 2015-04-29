while True:
	gallon = int(raw_input("Enter the gallons used (-1 to end): "))

	if gallon == -1:
		break

	mile = float(raw_input("Enter the miles driven: "))
	print "The miles / gallons for this tank was ", (mile / gallon) 
	
