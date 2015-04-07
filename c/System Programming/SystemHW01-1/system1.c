#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>

int main()
{
	pid_t childpid;
	int status;
	char *remain, ch;
	long fileSize1, fileSize2;
	FILE *filePtr1, *filePtr2, *filePtr3;
				
	childpid = fork(); // fork
	
	if(childpid >= 0) {
		if(childpid == 0) {
			if((filePtr1 = fopen("file1.txt","r")) == NULL || 
			   (filePtr2 = fopen("file2.txt","r")) == NULL || 
			   (filePtr3 = fopen("file3.txt","w+")) == NULL ) {
				perror("SOURCE FILE NOT OPENED");
				return 1;
			}

			fseek(filePtr1, 0, SEEK_END);
			fileSize1 = ftell(filePtr1);
			fseek(filePtr2, 0, SEEK_END);
			fileSize2 = ftell(filePtr2);
			
			// choose taller file for child process
			if(fileSize1 < fileSize2) {
				// go fileSize1. character of filePtr2
				fseek(filePtr2, fileSize1-1, SEEK_SET);
				// store more characters in filePtr3 array
				while(!feof(filePtr2)) {
					fscanf(filePtr2,"%c",&ch);
					fprintf(filePtr3,"%c",ch);
				}
			}
			else {
				// go fileSize2. character of filePtr1
				fseek(filePtr1, fileSize2-1, SEEK_SET);
				// store more characters in filePtr3 array
				while(!feof(filePtr1)) {
					fscanf(filePtr1,"%c",&ch);
					fprintf(filePtr3,"%c",ch);
				}
			}
			
			fclose(filePtr1);
			fclose(filePtr2);
			fclose(filePtr3);
		}
		else { 
			wait(&status); /* wait for child to exit, and store its status */
			
			if((filePtr1 = fopen("file1.txt","a")) == NULL || 
			   (filePtr2 = fopen("file2.txt","a")) == NULL || 
			   (filePtr3 = fopen("file3.txt","r")) == NULL ) {
				perror("SOURCE FILE NOT OPENED");
				return 1;
			}

			fseek(filePtr1, 0, SEEK_END);
			fileSize1 = ftell(filePtr1);
			fseek(filePtr2, 0, SEEK_END);
			fileSize2 = ftell(filePtr2);
			
			// choose shorter file for parent process
			if(fileSize1 > fileSize2) {
				remain = (char *)malloc(sizeof(char) * (fileSize1-fileSize2)+1);			
				fseek(filePtr2, 0, SEEK_END);
				fseek(filePtr3, 0, SEEK_SET);
				fgets(remain,(fileSize1-fileSize2)+1,filePtr3);
				fprintf(filePtr2,"%s",remain);	
			}
			else {
				remain = (char *)malloc(sizeof(char) * (fileSize2-fileSize1)+1);	
				fseek(filePtr1, 0, SEEK_SET);
				fseek(filePtr3, 0, SEEK_SET);
				fgets(remain,(fileSize2-fileSize1)+1,filePtr3);
				fprintf(filePtr1,"%s",remain);				
			}
			
			free(remain);
			fclose(filePtr1);
			fclose(filePtr2);
			fclose(filePtr3);
		}
	}
	else{
		perror("FORK");
		return 1;
	}

	return 0;
}
