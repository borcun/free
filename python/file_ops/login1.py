# the program searchs users.txt file with login function for its parameter.
# if the parameter is in list, it updates activity_log.txt. Otherwise, it does nothing.

import datetime

log_file = 'activity_log.txt'
user_file = 'users.txt'

# this function updates log file
def update_log(username, password):
    # this is date format for log file
    login_time = str(datetime.datetime.now()) + "\n"
    activity_log = open(log_file, 'a')
    activity_log.write(username + password +  login_time)
    activity_log.close()

# this function checks login operation
def login(username, password):
    users = open(user_file, 'r')

    for user in users:
        data = user.split(":")
        
        # rstrip function removes parameter from string
        if username == data[0] and password == data[1].rstrip("\n"):          
            update_log(username, " - ")
            users.close()
            return True;
        elif username == data[0]:
            update_log(username, " - incorrect pass - ") 
            users.close()
            return True;

    users.close()
    return False

# correct username and password
login("fsenel", "12345")
login("epala", "254322")
login("jdoe", "34ad2999")
login("rforrester", "od8966s")
# correct username wrong password
login("fsenel", "123456")
login("epala", "2543222")
# wrong username and password
login("jfoster", "52dnc12")
