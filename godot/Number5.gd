extends Button

# class member variables go here, for example:
# var a = 2
# var b = "textvar"

func _ready():
	# Called when the node is added to the scene for the first time.
	# Initialization here
	pass

#func _process(delta):
#	# Called every frame. Delta is time since last frame.
#	# Update game logic here.
#	pass

func _pressed():
	var lbl = get_node( "../../PanelContainer/Label" )
	
	if Memory.error == 1:
		Memory.error = 0
		lbl.set_text( "0" )

	if lbl.get_text() == "0":
		lbl.set_text( "5" )
	else:
		if Memory.state == 1:
			lbl.set_text( "" )
			
		lbl.set_text( lbl.get_text() + "5" )
		
	Memory.state = 0