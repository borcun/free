extends KinematicBody2D

export (int) var SPEED
var velocity = Vector2()
var screensize

func _ready():
	screensize = get_viewport_rect().size

#func _process(delta):
#	# Called every frame. Delta is time since last frame.
#	# Update game logic here.
#	pass

func _physics_process(delta):
	velocity = Vector2()
	
	if Input.is_key_pressed( KEY_LEFT ):
		velocity.x -= 1
	elif Input.is_key_pressed( KEY_RIGHT ):
		velocity.x += 1
	else:
		return
	
	print( "velocity: ( ", velocity.x, ", ", velocity.y, " )" )
	
	velocity = velocity.normalized() * SPEED
	position += velocity * delta
	position.x = clamp( position.x, 40, screensize.x - 40 )
	position.y = clamp( position.y, 40, screensize.y - 40 )
	
	#move_and_collide( position )