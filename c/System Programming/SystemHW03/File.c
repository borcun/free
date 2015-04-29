#include "File.h"

// function that calculates file size
// param fStat : struct file_stat contains file whose size is calculated
// return      : file size
void tellFileSize(fileStatPtr fStat) {
	struct stat statbuf;
	int counter = 0;
	int flag;
	char ch;
	
	// check if file is opened or not
	if((fStat->fd = open(fStat->filePath,O_RDONLY)) == -1 ) {
		fprintf(stderr,"%s\n","FILE SIZE NOT CALCULATED");
		exit(EXIT_FAILURE);
	}
	
	// rewind file
	lseek(fStat->fd, 0, SEEK_SET);
	
	// read characters alternately to find size
	do{
		// read a character
		flag = read(fStat->fd,&ch,1);		
		// if character isn't EOF, increase counter
		if(flag != 0)
			++counter;
	}
	while(flag != 0); // until end of file
		
	fStat->fileSize = counter; // assing counter to fileSize
		
	// close file
	close(fStat->fd);	
}

// function that clones a file 
// param destPath   : destination path of file is copied
// param sourcePath : source path of file is copied
// return           : -
void cloneFile(char *destPath, char *sourcePath) {
	int ddes, sdes;
	int flag;
	char ch;
	// set modes
	mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	
	// open files
	ddes = open(destPath, O_WRONLY | O_CREAT | O_TRUNC, mode);
	sdes = open(sourcePath, O_RDONLY, mode);

	// check if files are opened or not
	if(ddes == -1 || sdes == -1) {
		fprintf(stderr,"%s\n","FILE NOT CREATED");
		exit(EXIT_FAILURE);
	}
	
	// go to begin of file
	lseek(sdes, 0, SEEK_SET);
	lseek(ddes, 0, SEEK_SET);

	// copy characters alternately
	do{
		// read a character
		flag = read(sdes,&ch,1);
		
		// if character isn't EOF, write file
		if(flag != 0)
			write(ddes,&ch,1);
	}
	while(flag != 0); // until end of file
	
	// close files
	close(ddes);
	close(sdes);

	return;
}

// function that check if path is directory or not
// param path : path name
// return     : status of path
int isDirectory(char *path) {
	struct stat statbuf;
	
	if (stat(path, &statbuf) == -1)
		return 0;
	else
		return S_ISDIR(statbuf.st_mode);
}
