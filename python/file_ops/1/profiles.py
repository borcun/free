def file_to_dic(path) :
	dic = {}
	username = ""
	profile_file = open(path, "r")

	for entry in profile_file :
		entry = entry.rstrip()

		if "USER" == entry :
			username = profile_file.next().rstrip()
			dic[ username ] = []
		elif "NAME" == entry :
			name = profile_file.next().rstrip()
			dic[ username ].append( name )
		elif "EMAIL" == entry :
			email = profile_file.next().rstrip()
			dic[ username ].append( email )
		elif "PHONE" == entry :
			phone = profile_file.next().rstrip()
			dic[ username ].append( phone )
		else:
			print( "unmatched tag" )

	return dic

dic = file_to_dic("profiles.txt")
print( dic )
