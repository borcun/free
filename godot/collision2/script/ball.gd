extends RigidBody2D

var screensize

func _ready():
	screensize = get_viewport_rect().size

#func _process(delta):
#	# Called every frame. Delta is time since last frame.
#	# Update game logic here.
#	pass