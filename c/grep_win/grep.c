#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <direct.h>
#include <io.h>
#include <windows.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <tchar.h>

#define MAX_CHAR 255

typedef enum ValidDirectory
{
  EMPTY    = -1,
  NO_DIR   =  0,
  OK       =  1,
  OPENERR  =  2,
  NOEXIST  =  3,
  NOREAD   =  4,
  UNEXPEC  =  5
} ValidDirectory;

// function that shows usage of application
void usage(const char *argv);
// function that checks whether path belongs to a directory or empty / full
ValidDirectory isValid(const wchar_t *path);
// function that run finding process
void process(const char *path, const char *word);
// function that finds and prints word in file
void find(const char *file, const char *word);

int main(int argc, char **argv)
{
  wchar_t path[MAX_CHAR] = {'\0'};

  if(argc != 3) {
    usage(argv[0]);
    return -1;
  }

  // convert char * to wchar_t *
  mbstowcs(path, argv[1], strlen(argv[1]));

  printf("Search \'%s\' in %s directory\n\n", argv[2], argv[1]);

  // check directory validity
  switch(isValid(path)) {
    case EMPTY:
      fprintf(stderr, "%s\n\n", "THE DIRECTORY IS EMPTY");
      break;
    case NO_DIR:
      fprintf(stderr, "%s\n\n", "PATH DOESN'T BELONGS TO A DIRECTORY");
      break;
    case OK:
      process(argv[1], argv[2]);
      break;
    case OPENERR:
      fprintf(stderr, "%s\n\n", "THE DIRECTORY CAN NOT BE OPENED");
      break;
    case NOEXIST:
      fprintf(stderr, "%s\n\n", "IT ISN'T A DIRECTORY PATH");
      break;
    case NOREAD:
      fprintf(stderr, "%s\n\n", "THE DIRECTORY HASN'T GOT READ PERMISSION");
      break;
		default:
      fprintf(stderr, "%s\n\n", "UNEXPECTED ERROR");
      break;
  }

  return 0;
}

// usage function
void usage(const char *argv)
{
  printf("usage: %s <directory path>", argv);
  return;
}

// ValidDirectory function
ValidDirectory isValid(const wchar_t *path)
{
  // if path is NULL, return NO_DIR that means it isn't directory
  if(NULL == path)
    return NO_DIR;

  // if directory hasn't been existed, return no access
  if(0 == _taccess_s(path, 0)) {
    // if directory hasn't got read permission, return no read
    if(0 == _taccess_s(path, 4))
      return OK;
    return NOREAD;
  }

  return NOEXIST;
}

// process function
void process(const char *path, const char *word)
{
  HANDLE hFind;
  WIN32_FIND_DATA FindData;
  char file[MAX_CHAR] = {'\0'};
  char absolute_path[MAX_CHAR] = {'\0'};
  wchar_t wabsolute_path[MAX_CHAR] = {'\0'};

  // change working directory
  chdir(path);
  strcpy(absolute_path, path, strlen(path));
  strcat(absolute_path, "*.txt");
  // convert char * to wchar_t *
  mbstowcs(wabsolute_path, absolute_path, strlen(absolute_path));

  // find the first file
  hFind = FindFirstFile(wabsolute_path, &FindData);
  wcstombs(file, FindData.cFileName, MAX_CHAR);
  find(file, word);

  // look for more files
  while(FindNextFile(hFind, &FindData)) {
    memset(file, '\0', MAX_CHAR);
    wcstombs(file, FindData.cFileName, MAX_CHAR);
    find(file, word);
  }

  // close the directory
  FindClose(hFind);

  return;
}

// find function
void find(const char *path, const char *word)
{
  FILE *fptr;
  char line[MAX_CHAR] = {'\0'};
  int count = 0;
  int isFound = 0;

  printf("File  : %s\n", path);
  printf("State : ");

  if(NULL == (fptr = fopen(path, "r"))) {
    printf("NOT OPENED\n");
    return;
  }

  printf("OPENED\n");
  printf("Lines : ");

  // search
  while(NULL != fgets(line, MAX_CHAR, fptr)) {
    if(NULL != strstr(line, word)) {
      printf("%d, ", count);
      isFound = 1;
    }

    memset(line, '\0', MAX_CHAR);
    ++count;
  }

  if(0 == isFound)
    printf("NOT FOUND");

  printf("\n\n");
  fclose(fptr);

  return;
}
