#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <setjmp.h>

// defines
#define HELP "--help"
#define ENTRY_FIFO_NAME "entry.dat"
#define QUIT_FIFO_NAME "quit.dat"
#define QUESTION_FIFO_NAME "question.dat"
#define ABORT_FIFO_NAME "abort.dat"
#define FEEDBACK_FIFO_NAME "feedback.dat"
#define FIFO_PERM S_IRUSR | S_IWUSR
#define TWO_MATRIX_SIZE 200
#define MATRIX_SIZE 100
#define MILLION 1000000L
#define NANOSECONDS 1000

// function prototypes
void usage();
void help();
void multiplicateTwoMatrix(char *, char *, FILE *);
char *pidToFileName(long, char *);
static void chandler(int);

// global variables to handle ctrl+c signal
static sigjmp_buf jmpbuf;
static volatile sig_atomic_t jumpok = 0;

// main function
int main(int argc, char **argv)
{
	long processID;				  // process ID
	FILE *clientLogFile;		  // client log file
	int entryFileDescriptor;	  // entry file descriptor
	int questionFileDescriptor;   // question file descriptor
	int quitFileDescriptor;		  // quit file descriptor
	int abortFileDescriptor;	  // abort file descriptor
	int feedbackFileDescriptor;   // feedback file descriptor
	char matrix[TWO_MATRIX_SIZE]; // matrix that store two matrixs come from fifo
	char matrix1[MATRIX_SIZE];    // first matrix that store first 100 integers into matrix
	char matrix2[MATRIX_SIZE];	  // second matrix that store last 100 integers into matrix
	static int flag = 0;          // flag
	int counter = 0, i, tempInt;  // temporay counter and helper variables
	long tdif;					  // time of calculate multiplication of two matrix 
	struct sigaction act;		  // sigaction struct that is used for ctrl+c handler 
	struct timespec slptm;		  // timespec struct for calculating time between start and end
	struct timeval tend, tstart;  // start time and end time of multiplication two matrix
	char sPtr[10] = {'\0'};
	
	// help function
	if(argc == 2 && !strcmp(argv[1],HELP)) {
		help();
		return 1;
	}
	
	// usage function
	if(argc != 1) {
		usage();
		return 1;
	}
	
	// set act struct flags
	act.sa_flags = 0;
	act.sa_handler = chandler;
	
	// take process id
	processID = (long)getpid();
	
	// open entry fifo
	if((entryFileDescriptor = open(ENTRY_FIFO_NAME, O_WRONLY)) == -1) {
		fprintf(stderr,"%s\n","SERVER NOT RUN");
		return 1;
	}
	
	// open quit fifo for reading as non-block
	if((quitFileDescriptor = open(QUIT_FIFO_NAME, O_RDONLY | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","QUESTION FIFO NOT OPENED");
		return 1;
	}
	
	// open question fifo for reading as block
	if((questionFileDescriptor = open(QUESTION_FIFO_NAME, O_RDONLY | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","QUESTION FIFO NOT OPENED");
		return 1;
	}
	
	// open abort fifo for writing as block
	if((abortFileDescriptor = open(ABORT_FIFO_NAME, O_WRONLY)) == -1) {
		fprintf(stderr,"%s\n","ABORT FIFO NOT OPENED");
		return 1;
	}
	
	// open feedback fifo for writing as block
	if((feedbackFileDescriptor = open(FEEDBACK_FIFO_NAME, O_WRONLY | O_NONBLOCK)) == -1) {
		fprintf(stderr,"%s\n","FEEDBACK FIFO NOT OPENED");
		return 1;
	}
	
	// open client log gile
	if((clientLogFile = fopen(pidToFileName(processID, sPtr),"w+")) == NULL) {
		fprintf(stderr,"CLIENT LOG FILE NOT OPENED\n");
		return 1;
	}
	
	// write process id to competition entry fifo
	if(write(entryFileDescriptor, &processID, sizeof(long)) == -1) {
		fprintf(stderr,"%s\n","WRITE IS ERROR");
		return 1;
	}

	// set sigempty function with signal struct flags
	if ((sigemptyset(&act.sa_mask) == -1) || (sigaction(SIGINT, &act, NULL) == -1)) {
		fprintf(stderr,"%s\n","Failed to set up SIGINT handler");
		return 1;
	}

	// if flag equals 1, finishing is done by client
	// in this case, send a message to server about quit from competition
	if(sigsetjmp(jmpbuf, 1)) {
		// write abort request to competition abort fifo to send server
		if(write(abortFileDescriptor, &processID, sizeof(long)) == -1) {
			fprintf(stderr,"%s\n","WRITE ERROR");
			return 1;
		}
		
		// write quit message
		fprintf(stderr,"\n%s\n","I QUIT COMPETITION");
		fprintf(clientLogFile,"\n\n%s","I QUIT COMPETITION");
		fclose(clientLogFile); // close file
		exit(EXIT_FAILURE); // exit program
	}
	
	jumpok = 1; // set jumpok as 1
	
	// print process ID to log file	
	fprintf(clientLogFile,"I am process whose ID is %ld\n",processID);
	fprintf(clientLogFile,"I am waiting to start competition\n");
	
	// print process ID on screen
	fprintf(stderr,"I am process whose ID is %ld\n",processID);
	fprintf(stderr,"I am waiting to start competition\n");
	
	// start to compete
	while(1) {
		// read quit fifo if finish flag is send or not
		if(read(quitFileDescriptor,&tempInt,sizeof(int)) > 0) {
			// if tempInt equals 1, this quit is normal
			// competition finished and server send message about finishing
			if(tempInt) {
				fprintf(clientLogFile,"\nCOMPETITION FINISHED\n");
				fprintf(stderr,"\nCOMPETITION FINISHED\n");
			}
			// if tempInt equals o, this quit isn't normal
			// competition is finished by server and server send message about finishing
			else {
				fprintf(clientLogFile,"\nCOMPETITION IS FINISHED by SERVER\n");
				fprintf(stderr,"\nCOMPETITION IS FINISHED by SERVER\n");
			}
			// when finish competition normally or not, break loop and terminate program
			break; 
		}
		
		// take question from question fifo
		// if there is no question, do nothing
		if(read(questionFileDescriptor, matrix, TWO_MATRIX_SIZE) == TWO_MATRIX_SIZE) {		
			// parse two matrix
			for(i=0 ; i < TWO_MATRIX_SIZE ; ++i) {
				// read first 100 integer from matrix to matrix1
				if(i < MATRIX_SIZE)
					matrix1[i] = matrix[i];
				// read last 100 integer from matrix to matrix2
				else
					matrix2[i-MATRIX_SIZE] = matrix[i];
			}
	
			// initialize tv_sec and tv_nsec of slptm
			slptm.tv_sec = 0;
			slptm.tv_nsec = NANOSECONDS;
	
			// print stage start message to log file
			fprintf(clientLogFile,"\n******************************************************\n");
			fprintf(clientLogFile,"Stage %d Start\n",counter+1);
			
			// print stage start message on screen
			fprintf(stderr,"\n*********************\n");
			fprintf(stderr,"Stage %d Start\n",counter+1);			

			// take start time
			if(gettimeofday(&tstart, NULL) == -1) {
				fprintf(stderr, "Failed to get start time\n");
				return 1;
			}
	
			// call multiplicateTwoMatrix function 
			multiplicateTwoMatrix(matrix1, matrix2, clientLogFile);
	
			// take finish time
			if(gettimeofday(&tend, NULL) == -1) {
				fprintf(stderr,"Failed to get end time\n");
				return 1;
			}

			// find different between start and end
			tdif = MILLION*(tend.tv_sec - tstart.tv_sec) + tend.tv_usec - tstart.tv_usec;

			// print stage finish message to log file
			fprintf(clientLogFile," Stage Finish Time : %ld microseconds\n\n",tdif);		
			fprintf(clientLogFile,"Stage %d Finish\n",counter+1);
			fprintf(clientLogFile,"******************************************************\n");

			// print stage finish message on screeen
			fprintf(stderr," Stage Finish Time : %ld\n",tdif);		
			fprintf(stderr,"Stage %d Finish\n",counter+1);
			fprintf(stderr,"**********************\n");

			// write time to feedback fifo
			while(write(feedbackFileDescriptor, &tdif, sizeof(long)) < 0);
	
			++counter; // increase counter
			usleep(1000000); // sleep until next stage
		} // end of if
	} // end of while
	
	// close client log file
	fclose(clientLogFile); // close file
	
	return 0;
}

// function that multiplicate two matrix
// param matrix1 : first matrix
// param matrix2 : second matrix
// param fPtr	 : file pointer is written result
// return		 : -
void multiplicateTwoMatrix(char *matrix1, char *matrix2, FILE *fPtr) {
	int tempMatrix1[10][10];
	int tempMatrix2[10][10];
	int resultMatrix[10][10] = {0};
	int i,j,k;
	
	// detache two matrixs
	for(i=0 ; i < 10 ; ++i) {
		for(j=0 ; j < 10 ; ++j) {
			tempMatrix1[i][j] = ((int)matrix1[(i*10)+j]) - 48;
			tempMatrix2[i][j] = ((int)matrix2[(i*10)+j]) - 48;
		}
	}
	
	// print matrix multiplication header information to log file
	fprintf(fPtr,"\nMULTIPLICATION MATRIXS RESULT\n");
	fprintf(fPtr,"-----------------------------\n");
	
	// multiplicate two matrixs
	for(i=0 ; i < 10 ; ++i) {
		for(j=0 ; j < 10 ; ++j) { 
			for(k=0 ; k < 10 ; ++k) {
				resultMatrix[i][j] += (tempMatrix1[i][k] * tempMatrix2[k][j]);
			}
			// print result of multiplication alternately
			fprintf(fPtr,"%4d ",resultMatrix[i][j]);
		}
		// add new line
		fprintf(fPtr,"\n");
	}
	fprintf(fPtr,"\n");
	
	return;
}

// usage function
// param path : -
// return     : -
void usage() {
	fprintf(stderr,"./cli\n");
	return;
}

// help function
// param path : -
// return     : -
void help() {
	fprintf(stderr,"Description\n");
	fprintf(stderr,"\tThis program is a client that need server to make matrix multiplication\n");
	fprintf(stderr,"Contact\n");
	fprintf(stderr,"\tborcunozkablan[at]gmail[dot]com");
	return;	
}

// function that convert long to string
// param pid  : pid that will be file name 
// param sPtr : char pointer that store file name
// return     : char pointer of long number
char *pidToFileName(long pid, char *sPtr) {
	// take file name
	sprintf(sPtr,"%ld",pid);	
	// add extension of file
	strcat(sPtr,".log");
	
	return sPtr;
}

// function that handle ctrl+c
// param signo  : signal number
// return       : -
static void chandler(int signo) {
	if (jumpok == 0) 
		return;

	siglongjmp(jmpbuf, 1);
}
