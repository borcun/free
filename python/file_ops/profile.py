import sys

# profile file tags
user  = "USER"
name  = "NAME"
email = "EMAIL"
phone = "PHONE"

def get_records() :
    records = {}
    # key of dictionary entry which will be used as username
    key = ""

    try :
        profile = open( 'profiles.txt', 'r' )
    except IOError:
        print( "profiles.txt not opened" )
        sys.exit()

    for line in profile:
        # truncate line
        line = line.strip()

        if user == line :
            key = profile.readline().strip();
            # username plays role as key of dictionary entry
            # and create an array for each key
            records[ key ] = [];
        elif name == line :
            records[ key ].append( profile.readline().strip() )
        elif email == line :
            records[ key ].append( profile.readline().strip() )
        elif phone == line :
            records[ key ].append( profile.readline().strip() )
        else:
            print( "invalid tag" )

    profile.close()

    return records

records = get_records()
print( records )
