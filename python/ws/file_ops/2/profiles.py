user = "USER"
name = "NAME"
email = "EMAIL"
phone = "PHONE"

# function that converts file content to dictionary
def convert() :
	# create an empty dictionary
	profile_dictionary = {}
	# dictionary key
	dictionary_key = ""
	# profiles.txt file is opened with read mode
	with open( "profiles.txt", "r" ) as profile_list :
		while True:
			profile = profile_list.readline()

			if profile == '':
				break

			line = profile.strip()

 			# when the username is matched, create a array and add other informations next
			if user == line :
				dictionary_key = profile_list.readline()
				dictionary_key = dictionary_key.strip()
				profile_dictionary[ dictionary_key ] = []
				# user's other informations are added into array which is indicated by dictionary_key 
			elif name != line and email != line and phone != line :
				profile_dictionary[ dictionary_key ].append( line )

	profile_list.close()
	return profile_dictionary

print( convert() )
