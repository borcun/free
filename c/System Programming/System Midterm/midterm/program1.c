#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <signal.h>
#include <setjmp.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

// defines
#define MATRIX_DIRECTORY "pool"
#define ENTRY_FIFO_NAME "entry.dat"
#define QUIT_FIFO_NAME "quit.dat"
#define QUESTION_FIFO_NAME "question.dat"
#define ABORT_FIFO_NAME "abort.dat"
#define FEEDBACK_FIFO_NAME "feedback.dat"
#define SERVER_LOG_FILE "serverLog.log"
#define REQ_PERMS (S_IRUSR | S_IWUSR | S_IWGRP | S_IWOTH)
#define RES_PERMS (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)
#define MIN_COMPETITOR 3
#define MAX_COMPETITOR 10
#define FILE_NUMBER 10
#define MAX_CHAR 256
#define TWO_MATRIX_SIZE 200
#define MILLION 1000000L
#define NANOSECONDS 1000

// function prototypes
void usage();
void help();
int quitCompetitorFromCompetition(long *, long, int);
int isDirectory(char *);
void freeMemory(char **);
int fillMatrixArray(char *, char *, char *);
long findWinner(long *, long *, int, int);
void closeMatrixFile(int *);
static void chandler(int);

// global variables to handle ctrl+c signal
static sigjmp_buf jmpbuf;
static volatile sig_atomic_t jumpok = 0;

// main function
int main(int argc, char **argv)
{
	FILE *serverLogFile;	      // competition server log file
	int entryFileDescriptor;	  // competition entry file descriptor
	int quitFileDescriptor;		  // competition quit file descriptor
	int questionFileDescriptor;   // competition question file descriptor
	int abortFileDescriptor;	  // competition abort file descriptor
	int feedbackFileDescriptor;   // competition feedback file descriptor
	long *competitorArray;		  // competitor array that store competitors process ID
	long tempCompetitorID;		  // temporary variable that store a competitor ID
	int counter = 0;			  // counter that count competition stages
	static int competitionNumber; // competition number
	static int competitorNumber;  // competitor number
	int i=0, tempInt;		  	  // temporary index and variable
	int fileIndex1, fileIndex2;   // first and second files indexs 
	char **matrixFileName;		  // matrix files number in pool directory
	char matrix[TWO_MATRIX_SIZE]; // integer array that contain two matrixs will be multiplicated 
	char tempArr[MAX_CHAR]; 	  // temporary array that store file names in pool directory
	long finishTime;			  // time that is process time of every competitor
	int matrixFd[FILE_NUMBER];	  // matrix file descriptor
	long tdif;					  // time between start and finish of every stage
	struct sigaction act;		  // sigaction struct that is used for ctrl+c handler 
	struct dirent *direntp;		  // dirent struct that read matrix file in pool directory
	struct timespec slptm;		  // timespec struct for calculating time between start and end a stage
	struct timeval tend, tstart;  // start time and end time of a stage
	DIR *poolDirectory;			  // directory pointer
	long competitorFreq[MAX_COMPETITOR]; // frequency of competitors that show to win stage number

	// help function
	if(argc == 2 && !strcmp(argv[1],"--help")) {
		help();
		return 1;
	}

	// usage function
	if(argc != 3) {
		usage();
		return 1;
	}
	
	// store competition number 
	competitionNumber = atoi(argv[1]);
	// store competitor number
	competitorNumber = MIN_COMPETITOR;
	
	// check main parameters' available
	if(competitionNumber < 1 || competitionNumber > 10 || atoi(argv[2]) < 3 || atoi(argv[2]) > 10) {
		usage();
		return 1;
	}
	
	// set act struct flags
	act.sa_flags = 0;
	act.sa_handler = chandler;
	
	// open server log file
	if((serverLogFile = fopen(SERVER_LOG_FILE,"w+")) == NULL) {
		fprintf(stderr,"%s\n","SERVER LOG FILE NOT OPENED");
		return 1;
	}
	
	// allocate memory for competitorArray as many as MAX_COMPETITOR
	// allocate memory for matrixFileName as many as FILE_NUMBER
	competitorArray = (long *)malloc(MAX_COMPETITOR * sizeof(long));
	matrixFileName = (char **)malloc(FILE_NUMBER * sizeof(char *));
	
	// check if there is memory allocation error or not
	if(competitorArray == NULL || matrixFileName == NULL) {
		fprintf(stderr,"%s\n","MEMORY ALLOCATION ERROR");
		fclose(serverLogFile); // close server log file
		return 1;
	}
	
	// open pool directory that contains matrix files
	if((poolDirectory = opendir(MATRIX_DIRECTORY)) == NULL) {
		fprintf(stderr,"POOL DIRECTORY NOT OPENED");
		free(competitorArray); // deallocate competitorArray
		fclose(serverLogFile); // close server log file
		return 1;
	}
	
	// make competition entry fifo
	if(mkfifo(ENTRY_FIFO_NAME, REQ_PERMS) == -1) {
		fprintf(stderr,"%s\n","ENTRY FIFO NOT CREATED");
		free(competitorArray);   // deallocate competitorArray
		fclose(serverLogFile);   // close server log file
		closedir(poolDirectory); // close pool directory	
		return 1;
	}
	
	// make competition quit fifo
	if(mkfifo(QUIT_FIFO_NAME, RES_PERMS) == -1) {
		fprintf(stderr,"%s\n","QUIT FIFO NOT CREATED");
		unlink(ENTRY_FIFO_NAME); // unlink entry fifo
		free(competitorArray);   // deallocate competitorArray
		fclose(serverLogFile);   // close server log file		
		closedir(poolDirectory); // close pool directory	
		return 1;
	}
	
	// make competition question fifo
	if(mkfifo(QUESTION_FIFO_NAME, RES_PERMS) == -1) {
		fprintf(stderr,"%s\n","QUESTION FIFO NOT CREATED");
		unlink(ENTRY_FIFO_NAME); // unlink entry fifo
		unlink(QUIT_FIFO_NAME);  // unlink quit fifo
		free(competitorArray);   // deallocate competitorArray
		fclose(serverLogFile);   // close server log file
		closedir(poolDirectory); // close pool directory	
		return 1;
	}
	
	// make competition abort fifo
	if(mkfifo(ABORT_FIFO_NAME, REQ_PERMS) == -1) {
		fprintf(stderr,"%s\n","ABORT FIFO NOT CREATED");
		unlink(ENTRY_FIFO_NAME);	// unlink entry fifo
		unlink(QUIT_FIFO_NAME);     // unlink quit fifo
		unlink(QUESTION_FIFO_NAME); // unlik question fifo
		free(competitorArray);      // deallocate competitorArray
		fclose(serverLogFile);      // close server log file
		closedir(poolDirectory); 	// close pool directory	
		return 1;	
	}
	
	// make competition feedback fifo 
	if(mkfifo(FEEDBACK_FIFO_NAME, RES_PERMS) == -1) {
		fprintf(stderr,"%s\n","FEEDBACK FIFO NOT CREATED");
		unlink(ENTRY_FIFO_NAME);	// unlink entry fifo
		unlink(QUIT_FIFO_NAME);     // unlink quit fifo
		unlink(QUESTION_FIFO_NAME); // unlik question fifo
		unlink(ABORT_FIFO_NAME);    // unlik abort fifo
		free(competitorArray);   	// deallocate competitorArray
		fclose(serverLogFile);      // close server log file
		closedir(poolDirectory); 	// close pool directory	
		return 1;		
	}
	
	// open entry fifo for reading and writing as non-block
	if((entryFileDescriptor = open(ENTRY_FIFO_NAME, O_RDWR | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","ENTRY FIFO NOT OPENED");
		unlink(ENTRY_FIFO_NAME);	// unlink entry fifo
		unlink(QUIT_FIFO_NAME);     // unlink quit fifo
		unlink(QUESTION_FIFO_NAME); // unlik question fifo
		unlink(ABORT_FIFO_NAME);	// unlink abort fifo
		free(competitorArray);   	// deallocate competitorArray
		fclose(serverLogFile);      // close server log file
		closedir(poolDirectory); 	// close pool directory	
		return 1;
	}

	// open quit fifo for reading and writing as non-block
	if((quitFileDescriptor = open(QUIT_FIFO_NAME, O_RDWR | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","QUIT FIFO NOT OPENED");
		close(entryFileDescriptor); // close entry file descriptor
		unlink(ENTRY_FIFO_NAME);	// unlink entry fifo
		unlink(QUIT_FIFO_NAME);     // unlink quit fifo
		unlink(QUESTION_FIFO_NAME); // unlik question fifo
		unlink(ABORT_FIFO_NAME);	// unlink abort fifo
		free(competitorArray);   	// deallocate competitorArray
		fclose(serverLogFile);      // close server log file
		closedir(poolDirectory); 	// close pool directory	
		return 1;
	}
	
	// open question fifo for reading and writing as non-block
	if((questionFileDescriptor = open(QUESTION_FIFO_NAME, O_RDWR | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","QUESTION FIFO NOT OPENED");
		close(entryFileDescriptor); // close entry file descriptor
		close(quitFileDescriptor);  // close quit file descriptor
		unlink(ENTRY_FIFO_NAME);	// unlink entry fifo
		unlink(QUIT_FIFO_NAME);     // unlink quit fifo
		unlink(QUESTION_FIFO_NAME); // unlik question fifo
		unlink(ABORT_FIFO_NAME);	// unlink abort fifo
		free(competitorArray);   	// deallocate competitorArray
		fclose(serverLogFile);      // close server log file
		closedir(poolDirectory); 	// close pool directory		
		return 1;
	}

	// open abort fifo for reading and writing as non-block
	if((abortFileDescriptor = open(ABORT_FIFO_NAME, O_RDWR | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","ABORT FIFO NOT OPENED");
		close(entryFileDescriptor);    // close entry file descriptor
		close(quitFileDescriptor);     // close quit file descriptor
		close(questionFileDescriptor); // close question file descriptor
		unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
		unlink(QUIT_FIFO_NAME);        // unlink quit fifo
		unlink(QUESTION_FIFO_NAME);    // unlik question fifo
		unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
		free(competitorArray);   	   // deallocate competitorArray
		fclose(serverLogFile);         // close server log file
		closedir(poolDirectory); 	   // close pool directory	
		return 1;
	}
	
	// open feedback fifo for reading and writing as non-block
	if((feedbackFileDescriptor = open(FEEDBACK_FIFO_NAME, O_RDWR | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","FEEDBACK FIFO NOT OPENED");
		close(entryFileDescriptor);    // close entry file descriptor
		close(quitFileDescriptor);     // close quit file descriptor
		close(questionFileDescriptor); // close question file descriptor
		close(abortFileDescriptor);    // close abort file descriptor
		unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
		unlink(QUIT_FIFO_NAME);        // unlink quit fifo
		unlink(QUESTION_FIFO_NAME);    // unlik question fifo
		unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
		unlink(FEEDBACK_FIFO_NAME);	   // unlink feeback fifo
		free(competitorArray);   	   // deallocate competitorArray
		fclose(serverLogFile);         // close server log file
		closedir(poolDirectory); 	   // close pool directory
		return 1;
	}
	
	// take matrix file names in pool directory 
	while ((direntp = readdir(poolDirectory)) != NULL) {
		// check if file is directory or not
		if(!isDirectory(direntp->d_name)) {
			// take directory + file name
			strcpy(tempArr, MATRIX_DIRECTORY); // take first directory name
			strcat(tempArr, "/"); // add '/' its end
			strcat(tempArr, direntp->d_name); // at last, add file path
			matrixFileName[i] = (char *)malloc(strlen(tempArr) * sizeof(char)); // allocate memory
			
			// check if allocated memory for matrix file name is NULL or not
			if(matrixFileName[i] == NULL) {
				fprintf(stderr,"MEMORY ALLOCATION ERROR\n");
				close(entryFileDescriptor);    // close entry file descriptor
				close(quitFileDescriptor);     // close quit file descriptor
				close(questionFileDescriptor); // close question file descriptor
				close(abortFileDescriptor);    // close abort file descriptor
				unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
				unlink(QUIT_FIFO_NAME);        // unlink quit fifo
				unlink(QUESTION_FIFO_NAME);    // unlik question fifo
				unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
				unlink(FEEDBACK_FIFO_NAME);	   // unlink feeback fifo
				free(competitorArray);   	   // deallocate competitorArray
				fclose(serverLogFile);         // close server log file
				closedir(poolDirectory); 	   // close pool directory
				return 1;
			}
			
			// copy file path to i. element of matrixFileName
			strncpy(matrixFileName[i], tempArr, strlen(tempArr));
			// In end of copy, clear tempArr
			memset(tempArr,'\0',MAX_CHAR);			
			++i; // increase index
		}
	} // end of while
	
		// set sigempty function with signal struct flags
	if ((sigemptyset(&act.sa_mask) == -1) || (sigaction(SIGINT, &act, NULL) == -1)) {
		fprintf(stderr,"%s\n","Failed to set up SIGINT handler");
		return 1;
	}
	
	// if flag equals 1, finishing is done by client
	// in this case, send a message to server about quit from competition
	if(sigsetjmp(jmpbuf, 1)) {
		tempInt = 0;
		// finish competitition and write finish message
		for(i=0 ; i < competitorNumber ; ++i) {
			// write quit fifo message of competition fail
			if(write(quitFileDescriptor,&tempInt,sizeof(int)) == -1)
				break;
		}
		
		// write quit message
		fprintf(stderr,"\n%s\n","I CONCLUDE COMPETITION");
		fprintf(serverLogFile,"\n%s","I CONCLUDE COMPETITION");
		close(entryFileDescriptor);    // close entry file descriptor
		close(quitFileDescriptor);     // close quit file descriptor
		close(questionFileDescriptor); // close question file descriptor
		close(abortFileDescriptor);	   // close abort file descriptor
		close(feedbackFileDescriptor); // close feedback file descriptor
		unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
		unlink(QUIT_FIFO_NAME);        // unlink quit fifo
		unlink(QUESTION_FIFO_NAME);    // unlik question fifo
		unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
		unlink(FEEDBACK_FIFO_NAME);	   // unlink feedback fifo
		free(competitorArray);   	   // deallocate competitorArray
		fclose(serverLogFile);         // close server log file
		freeMemory(matrixFileName);    // deallocate array that contain matrix file name
		closedir(poolDirectory); 	   // close pool directory		
		exit(EXIT_FAILURE); 		   // exit program
	}
	
	jumpok = 1; // set jumpok as 1
	
	i = 0; // set i as 0
	
	// print header information to server log file
	fprintf(serverLogFile,"COMPETITORS\n");
	fprintf(serverLogFile,"-----------\n");
	
	// print header information on screen
	fprintf(stderr,"COMPETITORS\n");
	fprintf(stderr,"-----------\n");
	
	// wait until counter is bigger than MIN_COMPETITOR
	while(counter < MIN_COMPETITOR) {
		// wait until read from fifo
		if(read(entryFileDescriptor,&competitorArray[counter],sizeof(long)) > 0) {
			fprintf(stderr,"%d. Competitor ID : %ld\n",counter+1, competitorArray[counter]);
			fprintf(serverLogFile,"%d. Competitor ID : %ld\n",counter+1, competitorArray[counter]);
			++counter; // increase counter
		}
	}
	
	// set counter as 0
	counter = 0; 

	// initialize tv_sec and tv_nsec of slptm
	slptm.tv_sec = 0;
	slptm.tv_nsec = NANOSECONDS;

	// take start time of stage
	if(gettimeofday(&tstart, NULL) == -1) {
		fprintf(stderr, "Failed to get start time\n");
		return 1;
	}
		
	// start competition
	while(counter < competitionNumber) {		
		// check if any competitor aborts from competition or not
		if(read(abortFileDescriptor,&tempCompetitorID,sizeof(long)) > 0) {
			// abort competitor from competition when there is any run-time error
			if(quitCompetitorFromCompetition(competitorArray, tempCompetitorID, competitorNumber) == -1) {
				// finish competitition and write finish message
				for(i=0 ; i < competitorNumber ; ++i) {
					// write quit fifo message of competition fail
					if(write(quitFileDescriptor,&tempInt,sizeof(int)) == -1) {
						fprintf(stderr,"%s\n","COMPETITION FINISH ERROR");
						close(entryFileDescriptor);    // close entry file descriptor
						close(quitFileDescriptor);     // close quit file descriptor
						close(questionFileDescriptor); // close question file descriptor
						close(abortFileDescriptor);	   // close abort file descriptor
						close(feedbackFileDescriptor); // close feedback file descriptor
						unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
						unlink(QUIT_FIFO_NAME);        // unlink quit fifo
						unlink(QUESTION_FIFO_NAME);    // unlik question fifo
						unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
						unlink(FEEDBACK_FIFO_NAME);	   // unlink feedback fifo
						free(competitorArray);   	   // deallocate competitorArray
						fclose(serverLogFile);         // close server log file
						freeMemory(matrixFileName);    // deallocate array that contain matrix file name
						closedir(poolDirectory); 	   // close pool directory		
						return 1;
					}
				}
			}
			
			if(competitorNumber > 0) {
				// decrease competitor number by one
				--competitorNumber;
			
				// print process ID of competitor who quit on screen
				fprintf(stderr,"\nCompetitor whose ID %ld quit from competition\n",tempCompetitorID);
				fprintf(stderr,"Remain competitor number is %d\n",competitorNumber);
				fprintf(serverLogFile,"\nCompetitor whose ID %ld quit from competition\n",tempCompetitorID);
				fprintf(serverLogFile,"Remain competitor number is %d\n",competitorNumber);
			}
			
			if(competitorNumber < MIN_COMPETITOR && competitorNumber > 0) {
				// print warning message
				fprintf(stderr,"%d competitor can not go on competition\nThere are at least 3 competitors\n",competitorNumber);
				fprintf(stderr,"%d competitor is waiting...\n",MIN_COMPETITOR - competitorNumber);
				
				// wait new competitor
				while(read(entryFileDescriptor,&tempCompetitorID,sizeof(long)) < 0);
				competitorArray[competitorNumber] = tempCompetitorID; // add new competitor to competitionArray
				++competitorNumber; // increase competitor number as one
				
				// print new competitor ID and new competitor(s) number
				fprintf(stderr,"\nCompetitor whose ID %ld enter to competition\n", tempCompetitorID);
				fprintf(stderr,"New competitor number is %d\n",competitorNumber);
				fprintf(serverLogFile,"\nCompetitor whose ID %ld enter to competition\n", tempCompetitorID);
				fprintf(serverLogFile,"New competitor number is %d\n",competitorNumber);
			}			
		} // end of if

		// check if any competitor enters to competition or not
		if(read(entryFileDescriptor,&tempCompetitorID,sizeof(long)) > 0) {
			// if competitor number is smaller than MAX_COMPETITOR
			if(competitorNumber < MAX_COMPETITOR) {
				competitorArray[competitorNumber] = tempCompetitorID; // add new competitor to competitionArray
				++competitorNumber; // increase competitor number as one
				// print new competitor ID and new competitor(s) number
				fprintf(stderr,"\nCompetitor whose ID %ld enter to competition\n", tempCompetitorID);
				fprintf(stderr,"New competitor number is %d\n",competitorNumber);
				fprintf(serverLogFile,"\nCompetitor whose ID %ld enter to competition\n", tempCompetitorID);
				fprintf(serverLogFile,"New competitor number is %d\n",competitorNumber);
			}
			// if competition is full, print warning message on screen
			else {
				fprintf(stderr,"COMPETITION IS FULL\n");
				fprintf(stderr,"Competitor whose ID %ld can not enter competition",tempCompetitorID);
			}
		}
		
		// print info about every stage starting to log file
		fprintf(serverLogFile,"\n%s\n","***********************");
		fprintf(serverLogFile,"Stage %d Started\n",counter+1);
		fprintf(serverLogFile," Competitor Number : %d\n",competitorNumber);
				
		// print info about every stage starting on screen
		fprintf(stderr,"\n%s\n","***********************");
		fprintf(stderr,"Stage %d Started\n",counter+1);
		fprintf(stderr," Competitor Number : %d\n",competitorNumber);	

		srand(time(NULL)); // randomize random number set
		
		// write all question to question fifo			
		// fill two matrix to an array
		// write questions to question file descriptor path as many as competitor number
		fileIndex1 = rand() % FILE_NUMBER;
		fileIndex2 = rand() % FILE_NUMBER;		
		if(fillMatrixArray(matrix, matrixFileName[fileIndex1], matrixFileName[fileIndex2]) == -1)	{
			fprintf(stderr,"%s\n","FILL MATRIX ERROR");
			close(entryFileDescriptor);    // close entry file descriptor
			close(quitFileDescriptor);     // close quit file descriptor
			close(questionFileDescriptor); // close question file descriptor
			close(abortFileDescriptor);	   // close abort file descriptor
			close(feedbackFileDescriptor); // close feedback file descriptor
			unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
			unlink(QUIT_FIFO_NAME);        // unlink quit fifo
			unlink(QUESTION_FIFO_NAME);    // unlik question fifo
			unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
			unlink(FEEDBACK_FIFO_NAME);	   // unlink feedback fifo
			free(competitorArray);   	   // deallocate competitorArray
			fclose(serverLogFile);         // close server log file
			freeMemory(matrixFileName);    // deallocate array that contain matrix file name
			closedir(poolDirectory);	   // close pool directory
			return 1;
		}
		
		fprintf(serverLogFile,"\n Selected Matrix Files\n");
		fprintf(serverLogFile,"  1. %s\n",matrixFileName[fileIndex1]);
		fprintf(serverLogFile,"  2. %s\n\n",matrixFileName[fileIndex2]);
		
		i = 0; // set i as 0
		
		// write question to question fifo
		while(i < competitorNumber) {
			while(write(questionFileDescriptor, matrix, TWO_MATRIX_SIZE) < 0);
			++i; // increase index
		}
		
		memset(matrix, '\0', TWO_MATRIX_SIZE); // clear matrix array
		i = 0; // set i as 0
		tempCompetitorID = 9999; // take a big number to compare first process finish time	
		
		// take finish time from feedback fifo
		while(i < competitorNumber) {
			while(read(feedbackFileDescriptor, &finishTime, sizeof(long)) < 0);
			if(finishTime < tempCompetitorID) {
				tempCompetitorID = finishTime;
				competitorFreq[counter] = competitorArray[i];
			}
			++i;
		}
		
		// print finish stage information to log file
		fprintf(serverLogFile,"Stage %d Finished\n",counter+1);
		fprintf(serverLogFile," Winner of Stage : %ld\n",competitorFreq[counter]);
		fprintf(serverLogFile,"%s\n","***********************");
		
		// print stage and competitor information on screen
		fprintf(stderr,"Stage %d Finished\n",counter+1);
		fprintf(stderr,"%s\n","***********************");
				
		counter++; // increase counter
		usleep(1000000); // sleep to next stage		
	} // end of while

	// take finish time
	if(gettimeofday(&tend, NULL) == -1) {
		fprintf(stderr,"Failed to get end time\n");
		return 1;
	}
	
	// print header information to server log file
	fprintf(serverLogFile,"\nSTAGE WINNER\n");
	fprintf(serverLogFile,"------------\n");
	
	// write results of competition to server log file
	for(i=0 ; i < counter ; ++i)
		fprintf(serverLogFile, "%d. stage winner : %ld\n", i+1, competitorFreq[i]);
	
	// find different between start and end
	tdif = MILLION*(tend.tv_sec - tstart.tv_sec) + tend.tv_usec - tstart.tv_usec;
	
	// print competition time to server log file
	fprintf(serverLogFile,"\nCompetition Time   : %ld microseconds\n", tdif);
	fprintf(serverLogFile,"Competition Winner : %ld\n",findWinner(competitorFreq,competitorArray,competitorNumber,competitionNumber));

	tempInt = 1;
	// finish competitition
	for(i=0 ; i < competitorNumber ; ++i) {
		// write quit fifo message of competition is finished
		// if there is any error, break loop and go to lines that contain deallocations
		if(write(quitFileDescriptor,&tempInt,sizeof(int)) == -1) {
			break;
		}
	}
	
	close(entryFileDescriptor);    // close entry file descriptor
	close(quitFileDescriptor);     // close quit file descriptor
	close(questionFileDescriptor); // close question file descriptor
	close(abortFileDescriptor);	   // close abort file descriptor
	close(feedbackFileDescriptor); // close feedback file descriptor
	unlink(ENTRY_FIFO_NAME);	   // unlink entry fifo
	unlink(QUIT_FIFO_NAME);        // unlink quit fifo
	unlink(QUESTION_FIFO_NAME);    // unlik question fifo
	unlink(ABORT_FIFO_NAME);	   // unlink abort fifo
	unlink(FEEDBACK_FIFO_NAME);	   // unlink feedback fifo
	free(competitorArray);   	   // deallocate competitorArray
	fclose(serverLogFile);         // close server log file
	freeMemory(matrixFileName);    // deallocate array that contain matrix file name
	closeMatrixFile(matrixFd);	   // close matrix files
	closedir(poolDirectory); 	   // close pool directory
	return 0;
}

// usage function
// param path : -
// return     : -
void usage() {
	fprintf(stderr,"./ser <competition number> <competitor number>\n");
	fprintf(stderr,"Max competition and competitor are 10\n");
	fprintf(stderr,"Min competition is 1, min competitor are 3\n");
	return;
}

// help function
// param path : -
// return     : -
void help() {
	fprintf(stderr,"Description\n");
	fprintf(stderr,"\tThis program is a server that need at least 3 clients for make matrix multiplication\n");
	fprintf(stderr,"\tMaximum competition and competitor can be 10\n");
	fprintf(stderr,"\tMinimum competition can be 1, minimum competitor can be 3\n\n");
	fprintf(stderr,"Contact\n");
	fprintf(stderr,"\tborcunozkablan[at]gmail[dot]com");
	return;	
}

// function that quit a competitor from competition
// param competitorArr  : competitor array
// param quitCompetitor : competitor that will be quited from competitor array
// param competitorSize : competitor size
// return     			: if there is error return -1, else return 0
int quitCompetitorFromCompetition(long *competitorArr, long quitCompetitor, int competitorSize) {
	int i, j=0;
	long *tempArr;
	
	// if competitor size equals 0, return -1 
	if(competitorSize == 0)
		return -1;
	
	// if competitorSize equals 1, quit it from competitor array	
	if(competitorSize == 1) {
		if(competitorArr[0] == quitCompetitor) {
			// deallocate competitorArr
			free(competitorArr);
			// allocate competitorArr as many as competitorSize-1
			competitorArr = (long *)malloc(MAX_COMPETITOR * sizeof(long));

			// check if competitorArr is NULL or not
			if(competitorArr == NULL) {
				fprintf(stderr,"MEMORY ALLOCATION ERROR");
				return -1;
			}
			return 0;
		}		
		return -1;
	}		

	// allocate tempArr as many as competitorSize-1
	tempArr = (long *)malloc((competitorSize-1) * sizeof(long));
	
	// check if tempArr is NULL or not
	if(tempArr == NULL) {
		fprintf(stderr,"MEMORY ALLOCATION ERROR");
		return -1;
	}
	
	// copy competitors from competitorArr to tempArr
	for(i=0 ; i < competitorSize ; ++i, ++j) {
		// if element is competitor that is quited, jump it
		if(competitorArr[i] == quitCompetitor)
			--j; // decrease j
		else
			tempArr[j] = competitorArr[i]; // copy competitorArr to tempArr
	}
	
	// deallocate competitorArr
	free(competitorArr);
	// allocate competitorArr as many as competitorSize-1
	competitorArr = (long *)malloc(MAX_COMPETITOR * sizeof(long));
	
	// check if competitorArr is NULL or not
	if(competitorArr == NULL) {
		fprintf(stderr,"MEMORY ALLOCATION ERROR");
		return -1;
	}
	
	// copy competitors from tempArr to competitorArr
	for(i=0 ; i < competitorSize-1 ; ++i)
		competitorArr[i] = tempArr[i];

	// deallocate tempArr
	free(tempArr);
	
	return 0;
}

// function that check if path is directory or not
// param path : path name
// return     : status of path
int isDirectory(char *path) {
	struct stat statbuf;
	
	// if path doesn't belong to a directory, return 0
	if (stat(path, &statbuf) == -1)
		return 0;
	// else, return directory mode
	else
		return S_ISDIR(statbuf.st_mode);
}

// function that deallocate buffer
// param buf  : buffer that will be free
// return     : -
void freeMemory(char **buf) {
	int i;
	
	// deallocate buf alternately
	for(i=0 ; i < FILE_NUMBER ; ++i)
		free(buf[i]);
		
	return;
}

// function that fill two matrixs to buffer
// param buf    : buf is array that will be filled
// param path1  : first file descriptor path
// param path2  : second file descriptor path
// return       : if there is error return -1, else return 0
int fillMatrixArray(char *buf, char *path1, char *path2) {
	int fd1, fd2;
	int i = 0;
	char ch;
	
	// open matrix file
	if((fd1 = open(path1, O_RDONLY)) == -1 || (fd2 = open(path2, O_RDONLY)) == -1) {
		fprintf(stderr,"MATRIX FILE NOT OPENED\n");
		return 1;
	}	
	
	// seek file pointer to begin
	lseek(fd1, 0, SEEK_SET);
	lseek(fd2, 0, SEEK_SET);
	
	while(i < TWO_MATRIX_SIZE) {
		// read first 100 integer from file1
		if(i < TWO_MATRIX_SIZE / 2) {
			if(read(fd1, &ch, sizeof(char)) == -1) {
				fprintf(stderr,"READ FROM FILE IS FAIL\n");
				return -1;
			}
			// if read character from file descriptor is '\n' or ' '
			// it isn't written in matrix array
			if(ch != '\n' && ch != ' ') {
				buf[i] = ch;
				++i;
			}
		}
		// read last 100 integer from file2
		else {
			if(read(fd2, &ch, sizeof(ch)) == -1) {
				fprintf(stderr,"READ FROM FILE IS FAIL\n");
				return -1;
			}
			// if read character from file descriptor is '\n' or ' '
			// it isn't written in matrix array
			if(ch != '\n' && ch != ' ') {
				buf[i] = ch;
				++i;
			}
		}
	}
	// close matrix file
	close(fd1);
	close(fd2);	
	
	return 0;
}

// function that find winner of competition
// param cFreq 			: competitors stage win number
// param cPid			: competitors IDs
// param competitorNum  : competitors number
// param competitionNum : competition(s) number
// return 				: winner process ID
long findWinner(long *cFreq, long *cPid, int competitorNum, int competitionNum) {
	int i, j;
	int counter = 0, temp = 0;
	long winner;
	
	// search all competition winner
	for(i=0 ; i < competitorNum ; ++i) {
		for(j=0 ; j < competitionNum ; ++j) {
			if(cPid[i] == cFreq[j])
				++counter;
		}
		
		// if there is a process that wins more, hold its ID in winner
		if(counter > temp) {
			winner = cPid[i];
			temp = counter;
		}
		
		// set counter as 0 for next search
		counter = 0;
	}
	
	// return winner
	return winner;
}

// close opened matrix files
// param *fd  : file descriptors array
// return     : -
void closeMatrixFile(int *fd) {
	int i;
	
	// close file descriptor alternately
	for(i=0 ; i < FILE_NUMBER ; ++i)
		close(fd[i]);
		
	return;
}

// function that handle ctrl+c
// param signo  : signal number
// return       : -
static void chandler(int signo) {
	if (jumpok == 0) 
		return;

	siglongjmp(jmpbuf, 1);
}
