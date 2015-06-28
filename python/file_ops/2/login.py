import datetime

def log( record ) :
	# activity_log.txt file is opened with append mode
	with open( "activity_log.txt", "a" ) as activity_log : 
		activity_log.write( record + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f") + "\n" )
		
	activity_log.close()

# function that check login operation which is performed with username and password
def login( username, password ) :
	# users.txt file is opened with read mode
	with open( "users.txt", "r" ) as users :
		for line in users : 
			info = line.split( ":" )

			if info[ 0 ].strip() == username and info[ 1 ].strip() == password :
				log( username + " - " )
			elif info[ 0 ].strip() == username :
				log( username + " - incorrect password - " )

	users.close()

login( "fsenel", "12345" )
login( "epala", "254322" )
login( "fsenel", "123456" )
login( "epala", "2543222" )
login( "fsenelx", "12345" )
login( "epalax", "254322" )

