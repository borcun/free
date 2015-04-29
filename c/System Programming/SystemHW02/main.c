#include "Directory.h"

// function prototypes
void usage();
void help();

int main(int argc, char **argv)
{
	struct dir_stat dStat1;
	struct dir_stat dStat2;
	char option[2];
	int i;
	
	// if argc equals 2 and second parameter is --help, call help
	if(argc == 2 && !strcmp("--help",argv[1])) {
		help();
		return 1;
	}
	
	// if argc doesn't equals 4, call usage and terminate program
	if(argc != 4) {
		usage();
		return 1;
	}
	
	// if argc equals 4 but optional parameter doesn't equal -s or -d
	// call usage and terminate program
	if(strcmp(argv[3],"-s") != 0 && strcmp(argv[3],"-d") != 0) {
		usage();
		return 1;
	}
	
	// take option parameter
	strcpy(option, argv[3]);
	
	// open directories
	dStat1.directory = opendir(argv[1]);
	dStat2.directory = opendir(argv[2]);
	
	// check if directories are opened or not
	if(dStat1.directory == NULL || dStat2.directory == NULL) {
		if(errno == EACCES) {
			fprintf(stderr,"%s\n","SEARCH or READ permission is denied.");
			return 1;
		}
		else if(errno == ENOTDIR) {
			fprintf(stderr,"%s\n","Path is not dir path.");
			return 1;
		}
		else {
			fprintf(stderr,"%s\n","Directory not opened.");
			return 1;		
		}
	}
	
	// copy directory names
	strcpy(dStat1.dirName, argv[1]);
	strcpy(dStat2.dirName, argv[2]);
	
	// count files number of directories
	countFileNumber(&dStat1);
	countFileNumber(&dStat2);
	
	// take files attributes of directories
	takeFileAttributes(&dStat1);
	takeFileAttributes(&dStat2);
	
	// sync directories
	syncDirectory(&dStat1, &dStat2, option);
	
	// close directories
	closedir(dStat1.directory);
	closedir(dStat2.directory);
	
	return 0;
};

// function that show usage of program
// return : -
void usage() {
	fprintf(stderr,"\n%s <directory 1> <directory 2> option\n","./localSync");
	fprintf(stderr,"option -s : %s\n","size of files of directories");
	fprintf(stderr,"option -d : %s\n\n","last modification date of files of directories");
	return;
}

// function that show help menu of program
// return : - 
void help() {
	fprintf(stderr,"\n%s\n","Description : ");
	fprintf(stderr,"\t%s\n","This program sync two directory according to optional parameter");
	fprintf(stderr,"\t%s\n","First parameter could be first directory");
	fprintf(stderr,"\t%s\n","Second parameter could be second directory");
	fprintf(stderr,"\t%s\n","Third parameter could be optional parameter, <-s> or <-d>");
	fprintf(stderr,"\t%s\n","-s is size for directories, -d is last modification date for directories");
	fprintf(stderr,"%s\n","Contact : ");
	fprintf(stderr,"\t%s\n\n","borcunozkablan[at]gmail[dot]com");
	return;
}
