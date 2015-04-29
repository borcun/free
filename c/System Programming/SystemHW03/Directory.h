#include "File.h"
#ifndef PATH_MAX
#define PATH_MAX 255
#endif

// directory status struct
struct dir_stat {
	DIR *directory;			 // directory
	struct file_stat *fStat; // file_stat pointer
	char dirName[PATH_MAX];  // directory name
	int fileNumber;			 // file number in directory
};

// typedef
typedef struct dir_stat* dirStatPtr; 

// function that count file number in directory
int countFileNumber(dirStatPtr dStat);
// function that take files attributes in directory
// and copy them to a file_stat of dir_stat
void takeFileAttributes(dirStatPtr dStat);
// function that sync two directory according to option parameter
// if option is -s, sync directories as files size
// if option is -d, sync directories as files last modification date
void syncDirectory(dirStatPtr dStat1, dirStatPtr dStat2, char *option);
