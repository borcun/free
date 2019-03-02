extends Button

func _pressed():
	var lbl = get_node( "../../ResultContainer/ResultLabel" )
	
	lbl.set_text( "0" )
	
	Utility.first_number = null
	Utility.second_number = null
	Utility.operator = null
	Utility.error = null