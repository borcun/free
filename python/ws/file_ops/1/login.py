import datetime

def login(username, password) :
	user_file = open("users.txt", "r")
	
	for entry in user_file : 
		data = entry.rstrip().split(":")

		if data[0] == username :
			log_file = open("activity_log.txt", "a")
			
			if data[1] == password :
				log_file.write(username + " - " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S\n") )
			else :
				log_file.write(username + " - incorrect password - " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S\n") )

login("fsenel", "12345")
login("epala", "12345")
login("yagiz", "12345")
