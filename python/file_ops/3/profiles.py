def getRecords():
	records = {}
	record_list = open("profiles.txt", "r")
	is_key = False
	key = ""

	while True:
		record = record_list.readline()
		if record == '' :
			break
		# USER tells that next line becomes key of dictionary
		if record.strip() == "USER":
			is_key = True
		# read key and create an array for it
		elif is_key:
			key = record.strip()
			records[ key ] = []
			is_key = False
		elif record.strip() == "NAME":
			 records.get(key).append(record_list.readline().strip())
		elif record.strip() == "EMAIL":
			 records.get(key).append(record_list.readline().strip())
		elif record.strip() == "PHONE":
			 records.get(key).append(record_list.readline().strip())

	record_list.close()
	return records

# test
dic_records = getRecords()
print(dic_records)
