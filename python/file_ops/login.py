import sys
import datetime

def login( username, password ) :
    user_in_file = False
    pass_in_file = False

    try :
        records = open('users.txt', 'r')
    except IOError:
        print( "users.txt file not opened" )
        sys.exit()

    for record in records :
        # truncate line and parse via colon
        fields = record.strip().split(":")
        
        if username == fields[ 0 ] :
            user_in_file = True

            if password == fields[ 1 ]:
                pass_in_file = True
    
    # if the username is already in file, it is enough to update activity log file
    if user_in_file :
        try:
            log = open( 'activity_log.txt', 'a' ) 
        except IOError:
            print( "activity_log.txt file not opened" )
            sys.exit()
            
        # get time and format it
        log_date = datetime.datetime.now().strftime( "%Y-%m-%d_%H:%M:%S.%f" )

        if pass_in_file :
            log.write( username + " - " +  log_date  + "\n" )
        else :
            log.write( username + " - incorrect pass - " + log_date + "\n"  )

        log.close()
        records.close()
        return True;
    else:
        records.close()
        return False;

login( "fsenel", "12345" )
login( "epala", "254322" )
login( "jdoe", "12345" )
