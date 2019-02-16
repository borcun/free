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
	
	lbl.set_text( "0" )
	
	Memory.first_number = 0
	Memory.second_number = 0
	Memory.operator = ""
	Memory.error = 0