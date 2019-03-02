extends Button

func _pressed():
	var lbl = get_node( "../../ResultContainer/ResultLabel" )
	
	if Utility.error == 1:
		Utility.error = 0
		lbl.set_text( "0" )

	if lbl.get_text() == "0":
		lbl.set_text( self.text )
	else:
		if Utility.first_number == null and Utility.second_number == null and Utility.operator != null:
			var num = lbl.get_text()
			
			if	-1 != num.find( "." ):
				Utility.first_number = num.to_float()
			else:
				Utility.first_number = num.to_int()
			
			lbl.set_text( self.text )
		else:
			if Utility.first_number != null and Utility.second_number == null:
				lbl.set_text( "" )
			
			lbl.set_text( lbl.get_text() + self.text )
			Utility.second_number = lbl.get_text().to_int()