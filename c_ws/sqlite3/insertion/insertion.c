#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h> 

int main(int argc, char* argv[])
{
  sqlite3 *db = NULL;
  char *err_msg = NULL;
  char *insertion = NULL;
  char *creation = "CREATE TABLE STAFF(ID INT PRIMARY KEY NOT NULL, NAME TEXT NOT NULL, AGE INT);";

  /* open database */
  if(sqlite3_open("staff.sqlite", &db)) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    return -1;
  }

  fprintf(stderr, "Opened database successfully\n");
  /* create insertion query */
  insertion = "INSERT INTO staff(ID, NAME, AGE) VALUES (1, 'Burak', 27);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (2, 'Mehmet', 20);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (3, 'Ahmet', 20);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (4, 'Ali', 25);" \
    "INSERT INTO staff(ID, NAME, AGE) VALUES (5, 'Veli', 17);";

  /* execute table creation query */
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

  fprintf(stdout, "Records created successfully\n");

  /* close database */
  if(SQLITE_OK != sqlite3_close(db)) {
    fprintf(stderr, "Can't close database: %s\n", sqlite3_errmsg(db));
    return -1;
  }

  fprintf(stdout, "Closed database successfully\n");

  return 0;
}
