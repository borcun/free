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
		
	if velocity.length() > 0:
		$AnimatedSprite.play()
		velocity = velocity.normalized() * SPEED
	else:
		$AnimatedSprite.stop()
		
	position += velocity * delta
	position.x = clamp( position.x, 40, screensize.x - 40 )
	position.y = clamp( position.y, 40, screensize.y - 40 )
		
	if Input.is_action_pressed( "ui_page_up" ):
		rotation += 3.14159265359 / 4 
		print( "rotation: ", rotation )
	
	if Input.is_action_pressed( "ui_page_down" ):
		rotation -= 3.14159265359 / 4 
		print( "rotation: ", rotation )
		