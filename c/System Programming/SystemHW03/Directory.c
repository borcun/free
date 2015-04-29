#include "Directory.h"

int fileNumber = 0;

// function that count files in directory, update its fileNumber
// param dStat : directory that have files are counted
// return      : number of files in directory
int countFileNumber(dirStatPtr dStat) {
	struct dirent *direntp;
	int counter = 0;
	
	rewinddir(dStat->directory);
	// read until end of directory
	while((direntp = readdir(dStat->directory)) != NULL) {
		// check if path belongs to a directory or not
		if(!isDirectory(direntp->d_name))
			++counter; // increase counter
	}
		
	// go to begin of directory
	rewinddir(dStat->directory);
	// assign file number
	dStat->fileNumber = counter;
		
	return counter;
}

// function that take files attributes of a directory
// param dStat : directory have files whose attributes are taken
// return      : -
void takeFileAttributes(dirStatPtr dStat) {
	struct stat statbuf;
	struct dirent *direntp;
	int i = 0;

	// call countFileNumber to count file number of dStat directory	
	countFileNumber(dStat);

	// allocate file_stat struct of dStat
	dStat->fStat = (struct file_stat *)malloc(sizeof(struct file_stat) * dStat->fileNumber);
	
	// check if dStat->fStat is NULL or not
	if(dStat->fStat == NULL) {
		fprintf(stderr,"%s\n","FILES NOT READ");
		exit(EXIT_FAILURE);
	}
	
	// read until end of directory
	while((direntp = readdir(dStat->directory)) != NULL) {
		if(!isDirectory(direntp->d_name)) {
			// firstly, add directory name to filePath
			strcpy(dStat->fStat[i].filePath, dStat->dirName);
			// next, cat file name name to filePath
			// so, filePath is real path not just file name
			strcat(dStat->fStat[i].filePath, direntp->d_name);
			// assign file name to fileName pointer of fStat
			strcpy(dStat->fStat[i].fileName, direntp->d_name);
			
			// assign file last modification date to lastModDate of fStat
			// check if stat funciton is success or not

			if (stat(dStat->fStat[i].filePath, &statbuf) == -1) {
				fprintf(stderr,"%s\n","Failed to get file status");
				exit(EXIT_FAILURE);
			}
			// take last modification date of file
			else 
				dStat->fStat[i].lastModDate = statbuf.st_mtime;
			
			// calculate file size
			tellFileSize(&dStat->fStat[i]);
			++i; // increase index
		}
	}
	
	// go to begin of directory
	rewinddir(dStat->directory);
	
	return;
}

// function that sync two directory according to option parameter
// param dStat1 : first directory dir_stat struct pointer
// param dStat2 : second directory dir_stat struct pointer
// param option : optional parameter how copy works
// return       : -
void syncDirectory(dirStatPtr dStat1, dirStatPtr dStat2, char *option) {
	char tempPath[PATH_MAX];
	int i, j, flag = 0;
	int bigSize, smallSize;
	
	// if option is -d, sync directories according to last modification date
	if(!strcmp(option,"-d")) {
		for(i=0 ; i < dStat1->fileNumber ; ++i) {
			flag = 0; // flag is 0
			// search file in a directory in other directory
			for(j=0 ; j < dStat2->fileNumber ; ++j) {
				// if there are files these have same names, copy these as sizes of tem
				if(!strcmp(dStat1->fStat[i].fileName, dStat2->fStat[j].fileName)) {
					// if files last modification dates are same, do nothing
					if(difftime(dStat1->fStat[i].lastModDate, dStat2->fStat[j].lastModDate) == 0) {
						flag = 1; // make 1 flag, so not to copy
						break;	  // break for
					}
					// if file in directory1 has more last modification date than file in directory2
					else if(difftime(dStat1->fStat[i].lastModDate, dStat2->fStat[j].lastModDate) > 0)
						cloneFile(dStat2->fStat[j].filePath, dStat1->fStat[i].filePath);
					// if file in directory2 has more last modification date than file in directory1
					else
						cloneFile(dStat1->fStat[i].filePath, dStat2->fStat[j].filePath);
				
					flag = 1; // flag is 1, so copy is done
					break;
				}
			} // end of for	
			// if flag is 0, make copy between directories
			// if flag is not 0, copy was done according to 
			// last modification date of files these have same names
			if(flag == 0) {
				// copy directory name of dStat2 to tempPath
				strcpy(tempPath, dStat2->dirName);
				// cat file name of dStat1 to tempPath
				// so, there will be a path file name of first directory in second directory
				strcat(tempPath, dStat1->fStat[i].fileName);
				// clone file of dStat1 to tempPath
				cloneFile(tempPath, dStat1->fStat[i].filePath);
			}
		} // end of for
			
		// now, copy files in directory2 to directory1
		for(i=0 ; i < dStat2->fileNumber ; ++i) {
			// copy directory name of dStat1 to tempPath
			strcpy(tempPath, dStat1->dirName);
			// cat file name of dStat2 to tempPath
			// so, there will be a path file name of second directory in first directory
			strcat(tempPath, dStat2->fStat[i].fileName);
			// clone file of dStat1 to tempPath
			cloneFile(tempPath, dStat2->fStat[i].filePath);
		}
	}
	// if option is -s, sync directories according to size
	else if(!strcmp(option,"-s")) {
		for(i=0 ; i < dStat1->fileNumber ; ++i) {
			flag = 0; // flag is 0
			// search file in a directory in other directory
			for(j=0 ; j < dStat2->fileNumber ; ++j) {
				// if there are files these have same names, copy these as sizes of tem
				if(!strcmp(dStat1->fStat[i].fileName, dStat2->fStat[j].fileName)) {
					// if files size are same, do nothing
					if(dStat1->fStat[i].fileSize == dStat2->fStat[j].fileSize) {
						flag = 1;
						break;
					}
					// if file in directory1 is bigger than file in directory2
					else if(dStat1->fStat[i].fileSize > dStat2->fStat[j].fileSize)
						cloneFile(dStat2->fStat[j].filePath, dStat1->fStat[i].filePath);
					// if file in directory2 is bigger than file in directory1
					else
						cloneFile(dStat1->fStat[i].filePath, dStat2->fStat[j].filePath);

					flag = 1; // flag is 1, so copy is done
					break;
				}
			} // end of for
			// if flag is 0, make copy between directories
			// if flag is not 0, copy was done according to size of files these have same names
			if(flag == 0) {
				// copy directory name of dStat2 to tempPath
				strcpy(tempPath, dStat2->dirName);
				// cat file name of dStat1 to tempPath
				// so, there will be a path file name of first directory in second directory
				strcat(tempPath, dStat1->fStat[i].fileName);
				// clone file of dStat1 to tempPath
				cloneFile(tempPath, dStat1->fStat[i].filePath);
			}
		} // end of for
	
		// now, copy files in directory2 to directory1
		for(i=0 ; i < dStat2->fileNumber ; ++i) {
			// copy directory name of dStat1 to tempPath
			strcpy(tempPath, dStat1->dirName);
			// cat file name of dStat2 to tempPath
			// so, there will be a path file name of second directory in first directory
			strcat(tempPath, dStat2->fStat[i].fileName);
			// clone file of dStat1 to tempPath
			cloneFile(tempPath, dStat2->fStat[i].filePath);
		}
	}
	// if option is not -s or -d, give error message, exit program
	else {
 		fprintf(stderr,"%s\n","FALSE PARAMETER");
 		fprintf(stderr,"%s is not parameter\n",option);
 		exit(EXIT_FAILURE);
 	}
 	
	return;
}
