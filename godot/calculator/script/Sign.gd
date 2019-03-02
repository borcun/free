extends Button

func _pressed():
	if Utility.error == 1:
		return
		
	var lbl = get_node( "../../ResultContainer/ResultLabel" )
	var num = lbl.get_text().to_int()
	
	if 0 != num:
		lbl.set_text( str( -1 * num ) )
		