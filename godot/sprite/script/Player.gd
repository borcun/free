extends Area2D

export (int) var SPEED
var velocity = Vector2()
var screensize

func _ready():
	screensize = get_viewport_rect().size
	
func _process(delta):
	velocity = Vector2()
	
	if Input.is_action_pressed( "ui_right" ):
		velocity.x += 1
	if Input.is_action_pressed( "ui_left" ):
		velocity.x -= 1
	if Input.is_action_pressed( "ui_up" ):
		velocity.y -= 1
	if Input.is_action_pressed( "ui_down" ):
		velocity.y += 1
		
	if velocity.length() > 1:
		velocity = velocity.normalized() * SPEED
		
	position += velocity * delta
		