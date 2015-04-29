/***********************************************************
  Description   : Sync two or more directory according to 
  				  their size and last modify date
  Author        : Burak Orcun Ozkablan
  Number - HW0X : 06104403 - HW03

 ***********************************************************/

#include "Directory.h"
#define TEMPDIR "/tmp/temp/" // temporary directory path

// function prototypes
void usage();
void help();

// main function
int main(int argc, char **argv)
{
	pid_t childpid;				 // child process' ids
	struct dir_stat *dStat;		 // directory status struct of child process'
	struct dir_stat tempDirStat; // temporary directory status struct
	int i, j;					 // counter
	int flag = 0;				 // flag
	
	// if argc equals 2 and second parameter is --help, call help
	if(argc == 2 && !strcmp("--help",argv[1])) {
		help();
		return 1;
	}
	
	// if argc isn't bigger than 3, call usage and terminate program
	if(argc < 4) {
		usage();
		return 1;
	}
	
	// if argc is bigger than 3 but optional parameter doesn't equal -s or -d
	// call usage and terminate program
	if(strcmp(argv[argc-1],"-s") != 0 && strcmp(argv[argc-1],"-d") != 0) {
		usage();
		return 1;
	}
	
	// check if there is any directory in all paths
	for(i=1 ; i < argc-1 ; ++i) {
		if((tempDirStat.directory = opendir(argv[i])) != NULL) {
			flag = 1;
			closedir(tempDirStat.directory);
			break;
		}
	}
	
	// if there isn't any path of a directory, giver error
	if(flag == 0) {
		fprintf(stderr,"%s\n%s\n","There isn't any path of directory",
						"At least, valid path of directory should be written");		
		return 1;
	}
	
	// allocate dir_stat pointer as many as argc-2
	// first parameter is program name, last parameter is optional parameter
	dStat = (struct dir_stat *)malloc((argc-2) * sizeof(struct dir_stat));
	
	// take temporary directory name to tempDirStat.dirName
	strcpy(tempDirStat.dirName,TEMPDIR);
	
	// create temporary directory to read,write and execute by user
	// check if directory is made or not, if there is directory, just open
	if(!isDirectory(tempDirStat.dirName)) {	
		if(mkdir(tempDirStat.dirName, S_IRWXU | S_IRWXG | S_IRWXO) < 0) {
			fprintf(stderr,"%s\n","TEMPORARY DIRECTORY NOT CREATED");
			return 1;	
		}
	}
	
	// open temporary directory and check it
	if((tempDirStat.directory = opendir(tempDirStat.dirName)) == NULL) {
		fprintf(stderr,"%s\n","TEMPORARY DIRECTORY NOT OPENED");
		rmdir(tempDirStat.dirName); // remove temporary directory
		return 1;	
	}

	// check if there is memory allocation error or not
	if(dStat == NULL || tempDirStat.directory == NULL) {
		fprintf(stderr,"%s\n","MEMORY ALLOCATION ERROR");
		rmdir(tempDirStat.dirName); // remove temporary directory
		return 1;
	}
	
	// check content of temporary file, if there is file(s)
	// remove all files from temporary directory
	takeFileAttributes(&tempDirStat);	
	for(i=0 ; i < tempDirStat.fileNumber ; ++i)
		remove(tempDirStat.fStat[i].filePath);
	
	// make fork argc-2 time by using fan process method
	for(i=0 ; i < argc-2 ; ++i)
		if((childpid = fork()) <= 0)
			break;
	
	// if i doesn't equal argc-2, that means one of child processes
	if(!childpid && i != argc-2) {	
		// open directory, start i+1 because i start from 0 to argc-2
		// 0th index belongs to program name, (argc-1)th index belongs to optional parameter
		if((dStat[i].directory = opendir(argv[i+1])) == NULL) {
			// create temporary directory to read,write and execute by user
			// check if directory is made or not
			if(mkdir(argv[i+1], S_IRWXU | S_IRWXG | S_IRWXO) < 0 ) {
				fprintf(stderr,"%s\n","TEMPORARY DIRECTORY NOT CREATED");
				return 1;	
			}
		}
		
		// copy directory name
		strcpy(dStat[i].dirName, argv[i+1]);		
		// take files attributes of directories
		takeFileAttributes(&dStat[i]);
		takeFileAttributes(&tempDirStat);
		//sync directories according to optional parameter argv[argc-1]
		syncDirectory(&dStat[i], &tempDirStat, argv[argc-1]);
		// close directory
		closedir(dStat[i].directory);
	}
	
	// if childpid is 0 and i equals argc-2, that means parent process
	if(childpid && i == argc-2) {
		wait(NULL); // wait all children until terminate

		for(j=0 ; j < argc-2 ; ++j) {
			// open directory, start j+1 because j start from 0
			// 0th index belongs to program name
			if((dStat[j].directory = opendir(argv[j+1])) == NULL) {
				fprintf(stderr,"%s directory not opened",argv[j+1]);
				
				// remove all files in temporary directory
				for(j=0 ; j < tempDirStat.fileNumber ; ++j)
					remove(tempDirStat.fStat[j].filePath);
				
				closedir(tempDirStat.directory); // close temporary directory				
				rmdir(tempDirStat.dirName); // remove temporary directory
				free(dStat); // deallocate dStat pointer
				
				return 1;
			}
		
			// copy directory name
			strcpy(dStat[j].dirName, argv[j+1]);
			// take files attributes of directories
			takeFileAttributes(&dStat[j]);
			takeFileAttributes(&tempDirStat);			
			//sync directories according to optional parameter argv[argc-1]
			syncDirectory(&tempDirStat, &dStat[j], argv[argc-1]);
			// close directory
			closedir(dStat[j].directory);
		}
		
		// remove all files in temporary directory
		//for(j=0 ; j < tempDirStat.fileNumber ; ++j)
			//remove(tempDirStat.fStat[j].filePath);
		
		closedir(tempDirStat.directory); // close temporary directory
		//rmdir(tempDirStat.dirName); // remove temporary directory
		free(dStat); // deallocate dStat pointer
	}

	return 0;
}

// function that show usage of program
// return : -
void usage() {
	fprintf(stderr,"\n%s <directory 1> <directory 2> ... <directory n> option\n","./localSync");
	fprintf(stderr,"option -s : %s\n","size of files of directories");
	fprintf(stderr,"option -d : %s\n\n","last modification date of files of directories");
	return;
}

// function that show help menu of program
// return : - 
void help() {
	fprintf(stderr,"\n%s\n","Description : ");
	fprintf(stderr,"\t%s\n","This program sync two or more directory according to optional parameter");
	fprintf(stderr,"\t%s\n","From 2 to n-1 parameter could be directories' names");
	fprintf(stderr,"\t%s\n","Last parameter could be optional parameter, <-s> or <-d>");
	fprintf(stderr,"\t%s\n","-s is size for directories, -d is last modification date for directories");
	fprintf(stderr,"%s\n","Contact : ");
	fprintf(stderr,"\t%s\n\n","borcunozkablan[at]gmail[dot]com");
	return;
}
