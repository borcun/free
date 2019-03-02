extends Button

func _pressed():
	if Utility.error == 1:
		return
		
	var lbl = get_node( "../../ResultContainer/ResultLabel" )
	lbl.set_text( lbl.get_text() + "." )