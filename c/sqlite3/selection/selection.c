#include <stdio.h>
#include <string.h>
#include <sqlite3.h> 

/* callback function is called from sqlite3_exec function to list rows */
static int callback(void *data, int argc, char **argv, char **col_name)
{
	int i;

	// print column names one times
	if(!strcmp(argv[0], "1")) {
		for(i = 0; i < argc ; ++i)
			printf("%10s", col_name[i]);
		printf("\n");

		for(i = 0; i < argc ; ++i)
			printf("%10s", "----");
		printf("\n");
	}

	for(i = 0; i < argc ; ++i)
		printf("%10s", argv[i] ? argv[i] : "NULL");
	printf("\n");	

	return 0;
}

int main(int argc, char* argv[])
{
	sqlite3 *db;
	char *err_msg = 0;
	char *query = "SELECT * from STAFF";

	/* open database */
	if(sqlite3_open("staff.sqlite", &db)) {
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		return -1;
	}
	/* execute SQL statement */
	if(SQLITE_OK != sqlite3_exec(db, query, callback, NULL, &err_msg)){
		fprintf(stderr, "SQL error: %s\n", err_msg);
		sqlite3_free(err_msg);
	}
	/* close database */
	if(SQLITE_OK != sqlite3_close(db)) {
		fprintf(stderr, "Can't close database: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	return 0;
}
