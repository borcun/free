extends KinematicBody2D

export (int) var SPEED
export (int) var GRAVITY
var velocity = Vector2()
var screensize

func _ready():
	screensize = get_viewport_rect().size
	
func _physics_process(delta):
	velocity = Vector2()
	velocity.y += delta * GRAVITY
		
	if Input.is_key_pressed( KEY_RIGHT ):
		velocity.x += 1
	elif Input.is_key_pressed( KEY_LEFT ):
		velocity.x -= 1
	elif Input.is_key_pressed( KEY_UP ):
		velocity.y -= delta * GRAVITY
		velocity.y -= 1
	else:
		$GameboySprite.animation = "idle"
		
	if velocity.length() > 0:
		$GameboySprite.animation = "walk"
		
		if velocity.x > 0:
			$GameboySprite.flip_h = false
		elif velocity.x < 0:
			$GameboySprite.flip_h = true
		elif velocity.y > 0:
			$GameboySprite.flip_h = $GameboySprite.flip_h
		elif velocity.y < 0:
			$GameboySprite.flip_h = $GameboySprite.flip_h
		
		velocity = velocity.normalized() * SPEED
		
	$GameboySprite.play()
		
	move_and_collide( velocity )
	
	#position += velocity * delta
	#position.x = clamp( position.x, 40, screensize.x - 40 )
	#position.y = clamp( position.y, 40, screensize.y - 40 )
