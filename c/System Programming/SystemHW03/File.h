#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifndef PATH_MAX
#define PATH_MAX 255
#endif

// file status struct
struct file_stat {
	int  fd;				 // file descriptor
	char fileName[PATH_MAX]; // file name
	char filePath[PATH_MAX]; // file full path
	int fileSize;			 // file size
	time_t lastModDate;		 // file last modification date
};

// typedef
typedef struct file_stat* fileStatPtr;

// function that meause file size
void tellFileSize(fileStatPtr fStat);
// function that clone file in source to destination
void cloneFile(char *dest, char *source);
// function that check if path is directory or not
int isDirectory(char *path);
