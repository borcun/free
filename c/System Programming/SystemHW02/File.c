#include "File.h"

// function that calculates file size
// param fStat : struct file_stat contains file whose size is calculated
// return      : file size
long tellFileSize(fileStatPtr fStat) {
	// check if file is opened or not
	if((fStat->filePtr = fopen(fStat->filePath,"r")) == NULL ) {
		fprintf(stderr,"%s\n","FILE SIZE NOT CALCULATED");
		exit(EXIT_FAILURE);
	}
	
	// calculate file size
	fStat->fileSize = ftell(fStat->filePtr);
	// rewind file
	fseek(fStat->filePtr, 0, SEEK_SET);
	// close file
	fclose(fStat->filePtr);	
}

// function that clones a file 
// param destPath   : destination path of file is copied
// param sourcePath : source path of file is copied
// return           : -
void cloneFile(char *destPath, char *sourcePath) {
	FILE *dFile, *sFile;
	char ch;

	// open files
	dFile = fopen(destPath,"w+");
	sFile = fopen(sourcePath,"r");

	// check if files are opened or not
	if(dFile == NULL || sFile == NULL) {
		fprintf(stderr,"%s\n","FILE NOT CREATED");
		exit(EXIT_FAILURE);
	}
	
	// copy characters alternately
	while(!feof(sFile)) {
		fscanf(sFile,"%c",&ch);
		fprintf(dFile,"%c",ch);
	}
	
	// close files
	fclose(dFile);
	fclose(sFile);
	
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
