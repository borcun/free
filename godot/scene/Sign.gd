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
	if Memory.error == 1:
		return
		
	var lbl = get_node( "../../PanelContainer/Label" )
	var num = lbl.get_text().to_int()
	
	if 0 != num:
		lbl.set_text( str( -1 * num ) )
		