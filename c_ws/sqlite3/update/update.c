#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sqlite3.h> 

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

int main(int argc, char **argv)
{
  sqlite3 *db = NULL;
  char *err_msg = NULL;
  char *insertion = NULL, *update = NULL, *selection = NULL;
  char *creation = "CREATE TABLE STAFF(ID INT PRIMARY KEY NOT NULL, NAME TEXT NOT NULL, AGE INT);";

  /* open database */
  if(sqlite3_open("staff.sqlite", &db)) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    return -1;
  }

  /* create insertion query */
  insertion = "INSERT INTO staff(ID, NAME, AGE) VALUES (1, 'Burak', 27);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (2, 'Mehmet', 20);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (3, 'Ahmet', 20);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (4, 'Ali', 25);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (5, 'Veli', 17);";

  /* execute create query */
  if(SQLITE_OK != sqlite3_exec(db, creation, NULL, NULL, &err_msg)) { 
     fprintf(stderr, "SQL error: %s\n", err_msg);
     sqlite3_free(err_msg);
     sqlite3_close(db);
     return -1;
  } 

  /* execute insertion query */
  if(SQLITE_OK != sqlite3_exec(db, insertion, NULL, NULL, &err_msg)) { 
     fprintf(stderr, "SQL error: %s\n", err_msg);
     sqlite3_free(err_msg);
     sqlite3_close(db);
     return -1;
  } 

	printf("before update operation\n\n");
  selection = "SELECT * from STAFF";
	/* execute selection statement */
	if(SQLITE_OK != sqlite3_exec(db, selection, callback, NULL, &err_msg)){
		fprintf(stderr, "SQL error: %s\n", err_msg);
		sqlite3_free(err_msg);
	}

  update = "UPDATE STAFF set AGE = 22 where ID = 3";
	/* execute update statement */
	if(SQLITE_OK != sqlite3_exec(db, update, callback, NULL, &err_msg)){
		fprintf(stderr, "SQL error: %s\n", err_msg);
		sqlite3_free(err_msg);
	}

	printf("\nafter update operation\n\n");
	/* execute selection statement */
	if(SQLITE_OK != sqlite3_exec(db, selection, callback, NULL, &err_msg)){
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
