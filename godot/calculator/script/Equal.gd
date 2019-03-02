extends Button

func _pressed():
	if Utility.first_number == null and Utility.second_number == null:
		return
	
	if Utility.error == 1:
		return
		
	var lbl = get_node( "../../ResultContainer/ResultLabel" )
	
	Utility.second_number = lbl.get_text().to_int()
	
	if Utility.operator == "+":
		Utility.first_number = Utility.first_number + Utility.second_number
		lbl.set_text( str( Utility.first_number ) )
	elif Utility.operator == "-":
		Utility.first_number = Utility.first_number - Utility.second_number
		lbl.set_text( str( Utility.first_number ) )
	elif Utility.operator == "*":
		Utility.first_number = Utility.first_number * Utility.second_number
		lbl.set_text( str( Utility.first_number ) )
	elif Utility.operator == "/":
		if Utility.second_number == 0:
			Utility.first_number = null
			Utility.second_number = null
			Utility.operator = null
			Utility.error = 1
			
			lbl.set_text( "Error" )
			
			return
			
		Utility.first_number = Utility.first_number / Utility.second_number
		lbl.set_text( str( Utility.first_number ) )
	elif Utility.operator == "%":
		if Utility.second_number == 0:
			Utility.first_number = null
			Utility.second_number = null
			Utility.operator = null
			Utility.error = 1
			
			lbl.set_text( "Error" )
			
			return
			
		Utility.first_number = Utility.first_number % Utility.second_number
		lbl.set_text( str( Utility.first_number ) )
	
	Utility.operator = null
	Utility.first_number = null
	Utility.second_number = null