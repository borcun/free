#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int countLine(FILE *);
char *returnOverage(FILE *, int);

int main()
{
	pid_t childpid;
	FILE *childFile, *parentFile, *tempFile;
	int pFileLine, cFileLine;
	int i;
	
	// fork
	if((childpid = fork()) == -1) {
		fprintf(stderr,"%s\n","FORK FAILED");
		return 1;
	}
	
	// CHILD PROCESS
	if(childpid == 0) {
		// open child, parent and temp files
		childFile = fopen("file1.txt","r");
		parentFile = fopen("file2.txt","r");
		tempFile = fopen("temp.txt","w+");
	
		// check memory allocation
		if(childFile == NULL || parentFile == NULL || tempFile == NULL) {
			fprintf(stderr,"%s\n","FILE NOT OPENED");
			return 1;
		}
		
		// measure length of every line in childFile and parentFile
		// store different between their lengths in tempFile
		while((cFileLine = countLine(childFile)) != 0 | (pFileLine = countLine(parentFile)) != 0) {
			fprintf(stderr,"child : %d   parent : %d\n",cFileLine,pFileLine);
			fprintf(tempFile,"%d\n",cFileLine-pFileLine);
		}
			
		// close child, parent and temp files
		fclose(childFile);
		fclose(parentFile);
		fclose(tempFile);
	}
	// PARENT PROCESS
	if(childpid > 0) {
		wait(NULL); // wait until child is terminated
		
		// open child, parent and temp files
		childFile = fopen("file1.txt","r+");
		parentFile = fopen("file2.txt","r+");
		tempFile = fopen("temp.txt","r");
	
		// check memory allocation
		if(childFile == NULL || parentFile == NULL || tempFile == NULL) {
			fprintf(stderr,"%s\n","FILE NOT OPENED");
			return 1;
		}
		
		// seek begin of files
		fseek(childFile,0,SEEK_SET);
		fseek(parentFile,0,SEEK_SET);
		fseek(tempFile,0,SEEK_SET);

		while(!feof(tempFile)) {
			// take an integer from tempFile
			fscanf(tempFile,"%d",&i);
			
			// line in parentFile is taller than line in childFile
			if(i < 0) {
				fprintf(stderr,"%s",returnOverage(parentFile,i));
			}
			// line in childFile is taller than line in parentFile
			else if(i > 0) {
				fprintf(stderr,"%s",returnOverage(childFile,i));
			}
			// line in childFile is same length line in parentFile
			else
				continue;		
		}
		
		// close child, parent and temp files
		fclose(childFile);
		fclose(parentFile);
		fclose(tempFile);
	}
	
	return 0;
}

// function that counts characters of a line in a file
int countLine(FILE *fPtr) {
	int counter = 0;
	
	// count until '\n' or EOF character
	while(fgetc(fPtr) != '\n' && !feof(fPtr))
		++counter;

	return counter;
}

// function that return overage of a file
char *returnOverage(FILE *fPtr, int offset) {
	char *overage;
	int seekPoint;
	char ch;
	
	fprintf(stderr,"offset : %d\n",offset);
	// make offset positive
	if(offset < 0)
		offset *= -1;	

	// allocate overage
	overage = (char *)malloc(sizeof(char)*offset);
	
	// check memory allocation
	if(overage == NULL) {
		fprintf(stderr,"%s\n","ALLOCATION ERROR");
		return NULL;
	}
	
	// find position of more characters that are written less line
	seekPoint = countLine(fPtr) - offset;
	// seek this position
	fseek(fPtr,SEEK_CUR,seekPoint);	
	fgets(overage,seekPoint,fPtr);
	fprintf(stderr,"seekPoint : %d\n",seekPoint);
			
	return overage;
}
