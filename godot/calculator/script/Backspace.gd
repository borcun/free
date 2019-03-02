extends Button

func _pressed():
	var lbl = get_node( "../../ResultContainer/ResultLabel" )

	if Utility.error == 1:
		Utility.error = null
		lbl.set_text( "0" )
		return
		
	var num_len = lbl.get_text().length()
	
	if num_len > 1:
		lbl.set_text( lbl.get_text().substr( 0, num_len-1 ) )
		