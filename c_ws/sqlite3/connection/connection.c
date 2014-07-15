#include <stdio.h>
#include <sqlite3.h>

int main(int argc, char* argv[])
{
  sqlite3 *db;

  /* the function that opens a connection to an sqlite database file
   * and returns a database connection object to be used by other sqlite
   * function. if the first parameter is NULL or ':memory:', sqlite3_open()
   * will create an in-memory database in RAM that lasts only for duration
   * of the session. if the first parameter is not NULL, sqlite3_open()
   * attempts to open the database file by using its value. if no file by
   * that name exists, sqlite3_open() will open a new database file by that name 
   */
  if(sqlite3_open("connection.sqlite", &db)){
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    return -1;
  }

  printf("Opened database successfully\n");

  /* the function closes a database connection previously opened by a call to
   * sqlite3_open(). all prepared statements associated with the connection
   * should be finalized prior to closing the connection. if any queries remain
   * that have not been finalized, sqlite3_close() will return SQLITE_BUSY with
   * the error message Unable to close due to unfinalized statements. 
   */
  if(SQLITE_OK == sqlite3_close(db))
    printf("Closed database successfully\n");
  else
    printf("Can't close database: %s\n", sqlite3_errmsg(db));

  return 0;
}
