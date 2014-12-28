#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h> 

int main(int argc, char* argv[])
{
  sqlite3 *db;
  char *err_msg = 0;
  /* create SQL query */
  char *query = "CREATE TABLE STAFF(ID INT PRIMARY KEY NOT NULL, NAME TEXT NOT NULL, AGE INT);";

  /* open database called staff.sqlite */
  if(sqlite3_open("staff.sqlite", &db)) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    return -1;
  }

  fprintf(stdout, "Opened database successfully\n");

  /* execute query */
  if(SQLITE_OK != sqlite3_exec(db, query, NULL, NULL, &err_msg)){
     fprintf(stderr, "SQL error: %s\n", err_msg);
     sqlite3_free(err_msg);
  }
  else
    fprintf(stdout, "Staff table created successfully\n");

  /* close database */
  if(SQLITE_OK != sqlite3_close(db)) {
    fprintf(stderr, "Can't close database: %s\n", sqlite3_errmsg(db));
    return -1;
  }

  fprintf(stdout, "Closed database successfully\n");

  return 0;
}
