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
	
	if Memory.first_number == null:
		Memory.first_number = lbl.get_text().to_int()
	else:
		Memory.second_number = lbl.get_text().to_int()
		
		if Memory.operator == "+":
			Memory.first_number = Memory.first_number + Memory.second_number
		elif Memory.operator == "-":
			Memory.first_number = Memory.first_number - Memory.second_number
		elif Memory.operator == "*":
			Memory.first_number = Memory.first_number * Memory.second_number
		elif Memory.operator == "/":
			if Memory.second_number == 0:
				Memory.first_number = null
				Memory.second_number = null
				Memory.operator = null
				Memory.error = 1
				
				lbl.set_text( "Error" )
				
				return
				
			Memory.first_number = Memory.first_number / Memory.second_number
		elif Memory.operator == "%":
			if Memory.second_number == 0:
				Memory.first_number = null
				Memory.second_number = null
				Memory.operator = null
				Memory.error = 1
				
				lbl.set_text( "Error" )
				
				return
				
			Memory.first_number = Memory.first_number % Memory.second_number
		else:
			return
		
		lbl.set_text( str( Memory.first_number ) )
		
	# set operator when plus is clicked
	Memory.operator = "+"
	Memory.second_number = null
