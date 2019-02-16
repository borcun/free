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
	
	Memory.state = 1
		
	if Memory.first_number == 0:
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
				Memory.first_number = 0
				Memory.second_number = 0
				Memory.operator = ""
				Memory.error = 1
				
				lbl.set_text( "Division Error" )
				
				return
				
			Memory.first_number = Memory.first_number / Memory.second_number
		elif Memory.operator == "%":
			if Memory.second_number == 0:
				Memory.first_number = 0
				Memory.second_number = 0
				Memory.operator = ""
				Memory.error = 1
				
				lbl.set_text( "Modulo Error" )
				
				return
				
			Memory.first_number = Memory.first_number % Memory.second_number
		else:
			return
		
		lbl.set_text( str( Memory.first_number ) )
		
	Memory.operator = "*"
	Memory.second_number = 0