/************************************/
/*	Description : Client			*/
/************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <pthread.h>
#include <error.h>
#include <signal.h>
#include <setjmp.h>

#define SIZE 255
char buf[SIZE];		 // char buffer array
int CON_SOCKET_PORT; // client connection socket port number
int MSG_SOCKET_PORT; // client message socket port number

// function prototypes
void *sendMessage(void *);
void *recvMessage(void *);
static void chandler(int);
void usage();
void help();

// global variables to handle ctrl+c signal
static sigjmp_buf jmpbuf;
static volatile sig_atomic_t jumpok = 0;
int messageSocket;   	 // message socket
pthread_t tid[2];	 	 // thread ids
static int killFlag = 0; // kill flag

int main(int argc, char **argv)
{
	int connectionSocket;     		// connection socket
	char *connectionIP;	  		    // server connection IP address
	unsigned short connectionPort;  // server connection port number
	char *clName = "CLIENT";  	    // process ID that is send to server
	int clientID;	  	  			// client ID
	int flag, i;					// flag and counter 
	struct sockaddr_in connAddr;    // connection socket attributes struct
	struct sockaddr_in msgAddr;     // message socket attributes struct
	struct sigaction act;		    // sigaction struct that is used for ctrl+c handler

	// help
	if(argc == 2 && !strcmp(argv[1],"--help")) {
		help();
		return 1;
	}

	// usage
	if(argc != 3) {
		usage();
		return 1;
	}
	
	// set act struct flags
	act.sa_flags = 0;
	act.sa_handler = chandler;
	
	connectionIP = argv[1]; // take server IP number
	CON_SOCKET_PORT = atoi(argv[2]); // take server port number
	MSG_SOCKET_PORT = CON_SOCKET_PORT + 1;
	
	// set connection socket attributes with 0
	// set client socket attributes with necessary values
	memset((char *)&connAddr, 0, sizeof(connAddr)); // clear connAddr structure
	connAddr.sin_family = AF_INET; // set family to Internet
	connAddr.sin_addr.s_addr = inet_addr(connectionIP); // set server connection IP number
	connAddr.sin_port = htons(CON_SOCKET_PORT); // set server connection port number
	
	// set message socket attributes
	memset((char *)&msgAddr, 0, sizeof(msgAddr)); // clear msgAddr structure
	msgAddr.sin_family = AF_INET; // set family to Internet
	msgAddr.sin_addr.s_addr = inet_addr(connectionIP); // set server message IP number
	msgAddr.sin_port = htons(MSG_SOCKET_PORT); // set server message port number
	
	// open connection socket
	if((connectionSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr,"%s\n","SOCKET NOT OPENED");
		close(connectionSocket); // close client socket
		return 1;
	}
	
	// open message socket
	if((messageSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr,"%s\n","SOCKET NOT OPENED");
		close(connectionSocket); // close client socket
		close(messageSocket);	 // close message socket
		return 1;
	}
	
	// set socket by NONBLCOK
	flag = fcntl(messageSocket, F_GETFL, 0);
	fcntl(messageSocket, F_SETFL, flag | O_NONBLOCK);
	flag = fcntl(connectionSocket, F_GETFL, 0);
	fcntl(connectionSocket, F_SETFL, flag | O_NONBLOCK);

	// connect to server, if server is not ready, wait until connect
 	while(connect(connectionSocket, (struct sockaddr *)&connAddr, sizeof(connAddr)) < 0);
 	while(connect(messageSocket, (struct sockaddr *)&msgAddr, sizeof(msgAddr)) < 0);
 	// send to server process ID
	while(write(connectionSocket, clName, strlen(clName)) < 0);
	// receive server client ID
	while(read(connectionSocket, &clientID, sizeof(int)) < 0);
	// print client ID on screen
	fprintf(stderr,"My Client ID : %d\n", clientID+1);
	
	// set sigempty function with signal struct flags
	if((sigemptyset(&act.sa_mask) == -1) || (sigaction(SIGINT, &act, NULL) == -1)) {
		fprintf(stderr,"%s\n","Failed to set up SIGINT handler");
		return 1;
	}
	
	// if flag equals 1, finishing is done by client
	// in this case, send a message to client about terminate
	if(sigsetjmp(jmpbuf, 1)) {
		for(i=0 ; i < 2 ; ++i)
			pthread_cancel(tid[i]);
		
		write(messageSocket, "-k", 2); // terminate client
		close(connectionSocket); 	   // close connection socket
		close(messageSocket);	 	   // close message socket
		
		return 0;
	}
	
	jumpok = 1; // set jumpok as 1
	
	// create thread to receive message
	if(pthread_create(&tid[0], NULL, recvMessage, &messageSocket)) {
		fprintf(stderr, "Failed to create thread\n");
		return 0;
	}
	
	// create thread to send message
	if(pthread_create(&tid[1], NULL, sendMessage, &messageSocket)) {
		fprintf(stderr, "Failed to create thread\n");
		return 0;
	}

	// join threads
	for(i=0 ; i < 2 ; ++i)
		pthread_join(tid[i], NULL);
	
	// close client sockets
	close(connectionSocket); // close connection socket
	close(messageSocket);	 // close message socket

	return 0;
}

// function that receive message from client(s)
// param args   : -
// return       : -
void *recvMessage(void *args) {
	while(1) {
		// if message is in message socket, receive it from message socket
		if(read(messageSocket, buf, SIZE) > 0) {
			buf[strlen(buf)] = '\0'; // add '\0'
			
			// if server is closed, server sends "-k" message to all clients
			// when "-k" message is arrived, client cancels its threads
			if(!strcmp(buf,"-k")) {
				fprintf(stderr,"SERVER IS CLOSED\n");
				write(messageSocket, "-k", 2);
				// cancel tid[1] that is sender
				// next, exit itself when server is closed
				while(pthread_cancel(tid[1]) != 0);
				pthread_exit(NULL);
				
				memset(buf,'\0',SIZE); // clear buf
				return NULL;
			}
			// if client wanna kill itself by "-k" message,
			// this message isn't used by writing message, so do nothing
			else if(!strcmp(buf,"-e")) {
				if(!killFlag)
					write(messageSocket, "-k", 2); // terminate client
				else
					sleep(1);
				// cancel tid[1] that is sender
				// next, exit itself when server is closed
				while(pthread_cancel(tid[1]) != 0);
				// clear buf
				memset(buf,'\0',SIZE);
				pthread_exit(NULL);
			}
			// if message is not "-k", thread go on receiving message			
			else {
				fprintf(stderr,"%s\n\n",buf);
				fprintf(stderr,"%s"," > ");
			}
			
			memset(buf,'\0',SIZE); // clear buf
		}
	} // end of while
	
	return NULL;
}

// function that send message to client(s)
// param args   : -
// return       : -
void *sendMessage(void *args) {
	
	printf("\n%s"," > ");
	
	while(1) {
		// get string from stdin
		if(fgets(buf, SIZE-1, stdin) != NULL) {
			buf[strlen(buf)-1] = '\0';
			// if server wanna quit, send "-q" to server
			if(!strcmp(buf,"-q")) {
				// cancel tid[0] that is sender
				// next, exit itself when server is closed
				write(messageSocket, "-q", 2);
				while(pthread_cancel(tid[0]) != 0);
				// clear buf
				memset(buf,'\0',SIZE);				
				pthread_exit(NULL);
			}
			// if client wanna kill server and all client(s),
			// send "kill" message to server
			else if(!strcmp(buf,"kill")) {
				write(messageSocket, "kill", 4);
				memset(buf,'\0',SIZE); // clear buf
				killFlag = 1;
			}
			// if client wanna kill itself by "-k" message,
			// this message isn't used by writing message, so do nothing
			else if(!strcmp(buf,"-k")) {
				// do nothing
				fprintf(stderr,"\n%s"," > ");
			}
			// send message to server
			else if(write(messageSocket, buf, strlen(buf)) > 0) {
				fprintf(stderr,"\n%s"," > ");
				// clear buf
				memset(buf,'\0',SIZE);
			}
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

// function that show usage of program
// return     : -
void usage() {
	fprintf(stderr,"./client <IP number> <port number>\n");
	return;
}

// function that show help of program
// return     : -
void help() {
	fprintf(stderr,"\nDescription\n\tThis program is chat program that gets across with clients\n");
	fprintf(stderr,"\tServers duty is being router, it directs clients messages each other\n");
	fprintf(stderr,"\tMessage format is X:message [ X : client number, message : Client message ]\n");
	fprintf(stderr,"\tA client can send \"kill\" message to server, server can kill other clients\n");
	fprintf(stderr,"\tThis code belongs to CLIENT\n\n");
	fprintf(stderr,"Author\n\tBurak Orcun Ozkablan, GIT\n\n");
	fprintf(stderr,"Contact\n\tborcunozkablan[at]gmail[dot]com\n\n");
	return;
}
