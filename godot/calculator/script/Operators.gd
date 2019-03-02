extends Button

func _pressed():
	if Utility.error == 1:
		return

	var lbl = get_node( "../../ResultContainer/ResultLabel" )

	if Utility.first_number == null:
		var num = lbl.get_text()

		if	-1 != num.find( "." ):
			Utility.first_number = num.to_float()
		else:
			Utility.first_number = num.to_int()
		
		Utility.operator = self.text
		
		return
	else:
		var num = lbl.get_text()
		
		if	-1 != num.find( "." ):
			Utility.second_number = num.to_float()
		else:
			Utility.second_number = num.to_int()
			
		if Utility.operator == "+":
			Utility.first_number = Utility.first_number + Utility.second_number
		elif Utility.operator == "-":
			Utility.first_number = Utility.first_number - Utility.second_number
		elif Utility.operator == "*":
			Utility.first_number = Utility.first_number * Utility.second_number
		elif Utility.operator == "/":
			if Utility.second_number == 0:
				Utility.first_number = null
				Utility.second_number = null
				Utility.operator = null
				Utility.error = 1
				
				lbl.set_text( "Error" )
				
				return
				
			Utility.first_number = Utility.first_number / Utility.second_number
		elif Utility.operator == "%":
			if Utility.second_number == 0 or -1 != str(Utility.first_number).find(".") or -1 != str(Utility.second_number).find("."):
				Utility.first_number = null
				Utility.second_number = null
				Utility.operator = null
				Utility.error = 1

				lbl.set_text( "Error" )

				return

			Utility.first_number = Utility.first_number % Utility.second_number
		else:
			return

		lbl.set_text( str( Utility.first_number ) )

	Utility.operator = self.text
	Utility.first_number = null
	Utility.second_number = null