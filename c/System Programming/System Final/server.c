/************************************/
/*	Description : Server			*/
/************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>
#include <setjmp.h>
#include <pthread.h>
#include <dirent.h>

#define TEMPDIR "/tmp/temp/" // temporary directory path
#define SIZE 255 // buffer size come from socket connection
#define MAX_CLIENT 100 // maximum client number

// function prototypes
static void chandler(int);
void *recvAndSendMessage(void *);
int isDirectory(char *);
void usage();
void help();

// global variables to handle ctrl+c signal
static sigjmp_buf jmpbuf;
static volatile sig_atomic_t jumpok = 0;

static int clientNumber = 0; // client number
char buf[SIZE];				 // char buffer to read connection socket
int *clientConnSocket;  	 // client connection sockets
int *clientMsgSocket; 		 // client message sockets
pthread_t *tid;				 // threads ids pointer
int CON_SOCKET_PORT; 		 // client connection socket port number
int MSG_SOCKET_PORT; 		 // client message socket port number
int killFlag = 0;			 // kill flag

// main function
int main(int argc, char **argv)
{
	int connectionSocket;		 // connection socket
	int messageSocket;			 // message socket
	int sizeConnSocket;   	     // size of socket attributes struct
	int sizeMsgSocket;   	     // size of socket attributes struct
	int i;					     // counter
	int flag;					 // flag for changing socket block type
	char *tempPtr;				 // temporary char pointer
	struct sockaddr_in connAddr; // connection sockets attributes struct
	struct sockaddr_in msgAddr;  // connection sockets attributes struct
	struct sigaction act;		 // sigaction struct that is used for ctrl+c handler
	FILE *fPtr;					 // offline message file pointer
	struct dirent *direntp;		 // struct dirent
	DIR *offlineDirectory;		 // offline message directory pointer

	// usage
	if(argc != 2) {	
		usage();
		return 1;
	}
	// help
	else if(!strcmp(argv[1],"--help")) {
		help();
		return 1;
	}
	
	// check if there is memory allocation error or not
	if((clientConnSocket = (int *)malloc(MAX_CLIENT * sizeof(int))) == NULL) {
		fprintf(stderr,"MEMORY ALLOCATION ERROR\n");
		return 1;
	}
	
	// check if there is memory allocation error or not
	if((clientMsgSocket = (int *)malloc(MAX_CLIENT * sizeof(int))) == NULL) {
		fprintf(stderr,"MEMORY ALLOCATION ERROR\n");
		free(clientConnSocket);  // deallocate clientConnSocket
		return 1;
	}
	
	// check if there is memory allocation error or not
	if((tid = (pthread_t *)malloc(MAX_CLIENT * sizeof(pthread_t))) == NULL) {
		fprintf(stderr,"MEMORY ALLOCATION ERROR\n");
		free(clientConnSocket);  // deallocate clientConnSocket
		free(clientMsgSocket);   // deallocate clientMsgSocket
		return 1;
	}

	// create temporary directory
	if(mkdir(TEMPDIR, S_IRWXU | S_IRWXG | S_IRWXO) < 0) {
		fprintf(stderr,"TEMPORARY DIRECTORY NOT CREATED\n");
		return 1;	
	}
		
	// open temporary directory 
	if((offlineDirectory = opendir(TEMPDIR)) == NULL) {
		fprintf(stderr,"OFFLINE MESSAGE DIRECTORY NOT OPENED\n");
		free(clientConnSocket);  // deallocate clientConnSocket
		free(clientMsgSocket);   // deallocate clientMsgSocket
		free(tid);				 // deallocate tid
		rmdir(TEMPDIR); 		 // remove offline message directory
		return 1;
	}
		
	// set act struct flags
	act.sa_flags = 0;
	act.sa_handler = chandler;
	
	// set connection and message port
	CON_SOCKET_PORT = atoi(argv[1]);
	MSG_SOCKET_PORT = CON_SOCKET_PORT + 1;
	
	// set connection socket attributes
	memset((char *)&connAddr, 0, sizeof(connAddr));
	connAddr.sin_family = AF_INET;
	connAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	connAddr.sin_port = htons(CON_SOCKET_PORT);
	
	// set connection socket attributes
	memset((char *)&msgAddr, 0, sizeof(msgAddr));
	msgAddr.sin_family = AF_INET;
	msgAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	msgAddr.sin_port = htons(MSG_SOCKET_PORT);

	// create client connection and message connections sockets
	if((connectionSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr,"SOCKET NOT CREATED\n");
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 1;
	}
	
	// create client connection and message connections sockets
	if((messageSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr,"SOCKET NOT CREATED\n");
		close(connectionSocket);	// close connection socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 1;
	}

	// measure size of socket structs
	sizeConnSocket = sizeof(connAddr);
	sizeMsgSocket = sizeof(msgAddr);
	
	// bind local address to connection socket
	if(bind(connectionSocket, (struct sockaddr *)&connAddr, sizeConnSocket)) {
		fprintf(stderr,"CONNECTION SOCKET BIND NOT EXISTED\n");
		close(connectionSocket);	// close connection socket
		close(messageSocket);		// close message socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 1;
	}
	
	// bind local address to message socket
	if(bind(messageSocket, (struct sockaddr *)&msgAddr, sizeMsgSocket)) {
		fprintf(stderr,"MESSAGE SOCKET BIND NOT EXISTED\n");
		close(connectionSocket);	// close connection socket
		close(messageSocket);		// close message socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 1;
	}

    // set size of listen queue
	if(listen(connectionSocket, MAX_CLIENT) == -1) {
		fprintf(stderr,"CONNECTION SOCKET LISTEN IS UNSUCCESS\n");
		close(connectionSocket);	// close connection socket
		close(messageSocket);		// close message socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 1;
	}

    // set size of listen queue
	if(listen(messageSocket, MAX_CLIENT) == -1) {
		fprintf(stderr,"MESSAGE SOCKET LISTEN IS UNSUCCESS\n");
		close(connectionSocket);	// close connection socket
		close(messageSocket);		// close message socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message director);
		rmdir(TEMPDIR);   	 		// remove offline message directory
		return 1;
	}
	
	// print client list header	
	fprintf(stderr,"CLIENT LIST\n");
	fprintf(stderr,"-----------\n");
	
	// change socket block type by non-block
	flag = fcntl(clientConnSocket[clientNumber], F_GETFL, 0);
  	fcntl(clientConnSocket[clientNumber] ,F_SETFL, flag | O_NONBLOCK);	
	flag = fcntl(clientMsgSocket[clientNumber], F_GETFL, 0);
	fcntl(clientMsgSocket[clientNumber], F_SETFL, flag | O_NONBLOCK);
	flag = fcntl(connectionSocket, F_GETFL, 0);
	fcntl(connectionSocket, F_SETFL, flag | O_NONBLOCK);
	flag = fcntl(messageSocket, F_GETFL, 0);
	fcntl(connectionSocket, F_SETFL, flag | O_NONBLOCK);
	
	// set sigempty function with signal struct flags
	if ((sigemptyset(&act.sa_mask) == -1) || (sigaction(SIGINT, &act, NULL) == -1)) {
		fprintf(stderr,"%s\n","Failed to set up SIGINT handler");
		close(connectionSocket);	// close connection socket
		close(messageSocket);		// close message socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 1;
	}
	
	// if flag equals 1, finishing is done by client
	// in this case, send a message to client about terminate
	if(sigsetjmp(jmpbuf, 1)) {
		// close server sockets
		for(i = 0 ; i < clientNumber ; ++i) {
			// send client(s) kill message due to terminating server
			if(write(clientMsgSocket[i], "-k", 2) < 0) {
				fprintf(stderr,"SEND IS FAILED\n");
				return 0;
			}
			
			close(clientConnSocket[i]); // close clientConnSocket sockets
			close(clientMsgSocket[i]);  // close clientMsgSocket sockets
			pthread_cancel(tid[i]); 	// cancel all thread(s)
		}

		// clean temporary files in TEMPDIR
		while((direntp = readdir(offlineDirectory)) != NULL) {
			if(!isDirectory(direntp->d_name)) {
				memset(buf,'\0',SIZE);
				strcpy(buf,TEMPDIR);
				strcat(buf,direntp->d_name);
				fPtr = fopen(buf,"w+");
				fclose(fPtr);
				remove(buf);
			}
		}

		close(connectionSocket);	// close connection socket
		close(messageSocket);		// close message socket
		free(clientConnSocket);  	// deallocate clientConnSocket
		free(clientMsgSocket);   	// deallocate clientMsgSocket
		free(tid);					// deallocate tid
		closedir(offlineDirectory); // close offline message directory
		rmdir(TEMPDIR);    			// remove offline message directory
		return 0;
	}
	
	jumpok = 1; // set jumpok as 1
		
	// Connection and Message Between Clients
	while(1) {
		// accept connection to server connection socket
		if((clientConnSocket[clientNumber] = accept(connectionSocket, (struct sockaddr *)&connAddr, &sizeConnSocket)) > 0) {
			// accept connection to server connection socket
			if((clientMsgSocket[clientNumber] = accept(messageSocket, (struct sockaddr *)&msgAddr, &sizeMsgSocket)) > 0) {
				// check if there is a new client to connect server or not
				if(read(clientConnSocket[clientNumber], buf, SIZE) > 0) {
					buf[strlen(buf)] = '\0'; // add '\0'
					// if buf is connection string, it must be "CLIENT"
					if(!strcmp(buf,"CLIENT")) {
						// send client its client number
						if(write(clientConnSocket[clientNumber], &clientNumber, sizeof(int)) < 0)
							fprintf(stderr,"SEND IS FAILED\n");
						else {
							// print message when a client connected to server
							fprintf(stderr,"Client %d connected.\n", clientNumber+1);
						
							// search offline message directory if there is an offline message(s) file for client
							while((direntp = readdir(offlineDirectory)) != NULL) {
								if(!isDirectory(direntp->d_name)) {
									memset(buf,'\0',SIZE);
									tempPtr = strtok(direntp->d_name,".");

									if(atoi(tempPtr) == clientNumber+1) {
										memset(buf,'\0',SIZE);
										strcpy(buf,TEMPDIR);
										strcat(buf,direntp->d_name);
										strcat(buf,".dat");
										fprintf(stderr,"buf : %s\ntemp : %d\nclientNu : %d\n",buf,atoi(tempPtr),clientNumber);
										// open offline message file
										if((fPtr = fopen(buf,"r")) != NULL) {
											write(clientMsgSocket[clientNumber], "OFFLINE MESSAGE(S)", SIZE);
											write(clientMsgSocket[clientNumber], "------------------", SIZE);
											while(!feof(fPtr)) {
												memset(buf,'\0',SIZE);
												fgets(buf,SIZE,fPtr);
												fprintf(stderr,"%s\n",buf);
												// send client its client number
												if(write(clientMsgSocket[clientNumber], buf, strlen(buf)) < 0)
													fprintf(stderr,"SEND IS FAILED\n");
											}
											// close file
											fclose(fPtr);
										}
									} // end of if
								} // end of if
							} // end of while
							
							// go to begin of offline diretory
							rewinddir(offlineDirectory);
							
							//create a thread for new client
							if(pthread_create(&tid[clientNumber], NULL, recvAndSendMessage, &clientNumber))
								fprintf(stderr,"THREAD CAN NOT CREATED\n");
							else {
								++clientNumber; // increase clientNumber when a client connected
						
								// set next connection socket by NONBLOCK
								flag = fcntl(clientConnSocket[clientNumber], F_GETFL, 0);
								fcntl(clientConnSocket[clientNumber], F_SETFL, flag | O_NONBLOCK);
						
								// set next message socket by NONBLOCK
								flag = fcntl(clientMsgSocket[clientNumber], F_GETFL, 0);
								fcntl(clientMsgSocket[clientNumber], F_SETFL, flag | O_NONBLOCK);
							}							
						} // end of else
					} // end of if
				
					// clear buf
					memset(buf,'\0',SIZE);

				} // end of if
			} // end of if		
		} // end of if
		
		if(killFlag) {
			break;		
		}
		
	} // end of while
	
	// close server sockets
	for(i = 0 ; i < clientNumber ; ++i) {
		close(clientConnSocket[i]); // close clientConnSocket sockets
		close(clientMsgSocket[i]);  // close clientMsınonegSocket sockets
		pthread_join(tid[i], NULL); // join thread(s)
	}
	
	close(connectionSocket);	// close connection socket
	close(messageSocket);		// close message socket
	free(clientConnSocket);  	// deallocate clientConnSocket
	free(clientMsgSocket);   	// deallocate clientMsgSocket
	free(tid);					// deallocate tid
	
	// clean temporary files in TEMPDIR
	while((direntp = readdir(offlineDirectory)) != NULL) {
		// pass current directory name, ..
		if(!isDirectory(direntp->d_name)) {
			memset(buf,'\0',SIZE);
			// set file path
			strcpy(buf,TEMPDIR);
			strcat(buf,direntp->d_name);
			// open file with w+ parameter to erase content
			fPtr = fopen(buf,"w+");
			fclose(fPtr);
			// remove file
			remove(buf);
		}
	}
			
	closedir(offlineDirectory); // close offline message directory
	rmdir(TEMPDIR); 		    // remove offline message directory
	
	return 0; // terminate code
}

// function that behaves a router between client to send and receive all messages
// args   : client id
// return : -
void *recvAndSendMessage(void *args) {
	const int size = 32;     // senderID and fileName size
	char buffer[SIZE];	     // buffer to read socket
	char *messageStr;		 // message string 
	int receiverID;			 // receiver id
	char senderID[size];	 // sender ID
	char fileName[size];	 // offline message file name
	int id = *((int *)args); // args parameter
	int i;					 // counter
	FILE *offlineMsgFile;	 // offline message file pointer

	--id; // decrease id to prevent increment in main function
		
	// check clientMsgSocket if there is a message(s) or not
	while(1) {
		// check if there is message in socket
		if(read(clientMsgSocket[id], buffer, SIZE) > 0) {
			buffer[strlen(buffer)] = '\0';
			// if client terminates with ctrl+c, it sends kill message to server
			if(!strcmp(buffer,"-k")) {
				// clear buffer
				memset(buffer,'\0',SIZE);
				fprintf(stderr,"Client %d terminated.\n",id+1);
				pthread_exit(NULL);
			}
			// if client quits with "-q" message, it sends quit message to server
			else if(!strcmp(buffer,"-q")) {
				// clear buffer
				memset(buffer,'\0',SIZE);
				fprintf(stderr,"Client %d quited.\n",id+1);
				pthread_exit(NULL);			
			}
			// if a client send "kill" message, kill all client(s) and server
			else if(!strcmp(buffer,"kill")) {
				// clear buffer
				memset(buffer,'\0',SIZE);
				// kill all client(s)
				for(i=0 ; i < clientNumber ; ++i)			
					write(clientMsgSocket[i],"-e",2);
				
				// set kill flag and exit thread 
				killFlag = 1;
				pthread_exit(NULL);
			}
			// if there is message, parse message to find receiver ID
			else if(strchr(buffer,':') != NULL) {
				messageStr = strtok(buffer,":");
				receiverID = atoi(messageStr);
			
				// if the receiverID is between 0 and clientNumber
				// this message is online message, go to its owner
				if(receiverID > 0 && receiverID <= clientNumber) {
					--receiverID; // decrease receiverID for index of array
					messageStr = strtok(NULL,":"); // take message part from buffer
					// print message from client to client
					fprintf(stderr,"FROM %d TO %d : %s\n",id+1,receiverID+1,messageStr);
					// add sender id end of message
					sprintf(senderID,"%s%d%c"," (",id+1,')');
					strcat(messageStr,senderID);
					messageStr[strlen(messageStr)] = '\0';
				
					// send client its client number
					if(write(clientMsgSocket[receiverID], messageStr, strlen(messageStr)) < 0) {
						fprintf(stderr,"SEND IS FAILED\n");
						return 0;
					}
				
					// clear buffer and messageStr
					memset(buffer,'\0',SIZE);
					memset(messageStr,'\0',strlen(messageStr));
				}
				// if the receiverID is not between 0 and clientNumber
				// this message is offline message, go to its owner
				else if(receiverID > clientNumber && receiverID < MAX_CLIENT) {
					// set offline message file name
					strcpy(fileName,TEMPDIR);
					strcat(fileName,messageStr);
					strcat(fileName,".dat");
				
					// open offline file
					if((offlineMsgFile = fopen(fileName,"a")) == NULL) {
						fprintf(stderr,"FILE CAN NOT CREATED\n");
					}
					else {
						messageStr = strtok(NULL,":"); // take message part from buffer
					
						// print message from client to client
						fprintf(stderr,"FROM %d TO %d : %s [offline]\n",id+1,receiverID,messageStr);
					
						// add sender id end of message
						sprintf(senderID,"%s%d%c"," (",id+1,')');
						strcat(messageStr,senderID);
						messageStr[strlen(messageStr)] = '\n';
				
						// send client its client number
						if(fprintf(offlineMsgFile, messageStr, strlen(messageStr)) < 0) {
							fprintf(stderr,"SEND IS FAILED\n");
						}
					
						// clear buffer and messageStr
						memset(buffer,'\0',SIZE);
						memset(messageStr,'\0',strlen(messageStr));
						// close file
						fclose(offlineMsgFile);
					} // end of else			
				} // end of else if
			} // end of if
		} // end of if
	} // end of while

	return NULL;
}

// function that handle ctrl+c
// param signo  : signal number
// return       : -
static void chandler(int signo) {
	if (jumpok == 0) 
		return;

	siglongjmp(jmpbuf, 1);
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

// function that show usage of program
// return     : -
void usage() {
	fprintf(stderr,"./server <port number>\n");
	return;
}

// function that show help of program
// return     : -
void help() {
	fprintf(stderr,"\nDescription\n\tThis program is chat program that gets across with clients\n");
	fprintf(stderr,"\tServers duty is being router, it directs clients messages each other\n");
	fprintf(stderr,"\tMessage format is X:message [ X : client number, message : Client message ]\n");
	fprintf(stderr,"\tA client can send \"kill\" message to server, server can kill other clients\n");
	fprintf(stderr,"\tThis code belongs to SERVER\n\n");
	fprintf(stderr,"Author\n\tBurak Orcun Ozkablan, GIT\n\n");
	fprintf(stderr,"Contact\n\tborcunozkablan[at]gmail[dot]com\n\n");
	return;
}
