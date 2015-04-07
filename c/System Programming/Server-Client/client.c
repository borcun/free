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

#define SIZE 255
#define FILEPATH "/home/borcun/Masaüstü/source.txt"

int main(int argc, char **argv)
{
	int clientSocket;				  // client socket
	struct sockaddr_in clientAddress; // client socket attributes struct
	unsigned short serverPort;		  // server port number
	char *serverIP;		      		  // server IP address
	FILE *fPtr;						  // file pointer
	char buf[SIZE];				      // string send to server

	// Usage
	if(argc != 3) {
		fprintf(stderr,"Usage : %s <Server IP> <Server Port>\n",argv[0]);
		return 1;
	}
	
	// open file
	if((fPtr = fopen(FILEPATH,"r")) == NULL) {
		fprintf(stderr,"%s\n","FILE NOT OPENED");
		return 1;
	}

	serverIP = argv[1]; // take server IP number
	serverPort = atoi(argv[2]); // take server port number

	// initialize client socket attributes with 0
	// set client socket attributes with necessary values
	memset((char *)&clientAddress,0,sizeof(clientAddress)); // clear clientAddress structure
	clientAddress.sin_family = AF_INET; // set family to Internet
	clientAddress.sin_addr.s_addr = inet_addr(serverIP); // set server IP number
	clientAddress.sin_port = htons(serverPort); // set port number

	// open socket
	if((clientSocket = socket(AF_INET,SOCK_STREAM,0)) == -1) {
		fprintf(stderr,"%s\n","SOCKET NOT OPENED");
		close(clientSocket); // close client socket
		return 1;
	}

	// connect to server
 	if(connect(clientSocket, (struct sockaddr *)&clientAddress, sizeof(clientAddress)) < 0 ) {
 		fprintf(stderr,"%s\n","CONNECTION IS UNSUCCES");
 		close(clientSocket);
 		return 1;
 	}

	fseek(fPtr, 0, SEEK_SET);
	// write buf to client socket from file
	while(!feof(fPtr)) {
		fgets(buf, SIZE, fPtr); // get string from file
		
		// send to server
		if(send(clientSocket,buf,strlen(buf),0) < 0) {
			close(clientSocket); // close client socket
			return 1;
		}
		printf("%s",buf);
		usleep(1);
		// initialize buf with 0
		bzero(buf,SIZE);
	}

	// finish connection
	write(clientSocket,"FINISH_CONNECTION",17);
	close(clientSocket); // close client socket
	fclose(fPtr); // close file pointer

	return 0;
}
