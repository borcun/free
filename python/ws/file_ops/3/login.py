import datetime

def login(username, password):
	users_list = open("users.txt", "r")
	log_list = open("activity_log.txt", "a")

	for user in users_list: 
		name_and_pass = user.split(":")

		# rstrip remove escape characters from rightside of string 
		if username == name_and_pass[0].rstrip():
			if password == name_and_pass[1].rstrip():
				log_list.write(username + " " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f") + "\n")
			else:
				log_list.write(username + " incorrect password " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f") + "\n")

	users_list.close()
	log_list.close()

# test
login("fsenel", "12345")
login("epala", "254322")
login("jdoe", "34ad2999")
login("rforrester", "od8966s")
login("fsenel", "34ad2999")
login("fsenel", "1")
