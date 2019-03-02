extends Panel

var screensize

func _ready():
	screensize = get_viewport_rect()
		
	$ResultContainer.rect_position.x = 18
	$ResultContainer.rect_size.x = screensize.size.x - 36
	$ResultContainer.rect_size.y = screensize.size.y / 7.68

	$NumberContainer.rect_position.x = 20
	$NumberContainer.rect_position.y = $ResultContainer.rect_size.y + 20
	$NumberContainer.rect_size.x = screensize.size.x - 40
	$NumberContainer.rect_size.y = screensize.size.y / 1.20

