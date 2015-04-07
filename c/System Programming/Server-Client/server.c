/************************************/
/*	Description : Server			*/
/************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// buffer size come from socket connection 
#define SIZE 255
// target file path
#define FILEPATH "/home/borcun/Masaüstü/target.txt"

int main(int argc, char **argv)
{
	int serverSocket, readerSocket; // server and reader sockets
	struct sockaddr_in serverAddr;  // server socket attributes struct
	int serverPort;				    // server port number
	char buf[SIZE];				    // buffer read stream from server socket 
	int sizeSocketStruct;   	    // size of socket attributes struct
	FILE *fPtr;
	int i;

	// Usage
	if(argc != 2) {
		fprintf(stderr,"Usage %s <Server Port>\n",argv[0]);
		return 1;
	}
	
	if((fPtr = fopen(FILEPATH,"w+")) == NULL) {
		fprintf(stderr,"%s\n","FILE NOT OPENED");
		return 1;
	}

	// take server port number
	serverPort = atoi(argv[1]);

	// initialize server socket attributes with 0
	// set server scoket attributes with necessary values
	memset((char *)&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = htons(serverPort);

	// open socket
	if((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr,"SOCKET NOT OPENED\n");
		return 1;
	}

	// bind local address to socket
	if( bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0 ) {
		fprintf(stderr,"BIND NOT EXISTED\n");
		close(serverSocket); // close server socket
		return 1;
	}

    // set size of listen queue
	if( listen(serverSocket,1) == -1 ) {
		fprintf(stderr,"LISTEN IS UNSUCCESS\n");
		close(serverSocket); // close server socket
		return 1;
	}

	// measure size of socket struct
	sizeSocketStruct = sizeof(serverAddr);
	
	// accept connection to server socket
	if((readerSocket = accept(serverSocket,(struct sockaddr *)&serverAddr, &sizeSocketStruct)) < 0) {
		fprintf(stderr,"ACCEPTION IS UNSUCCESS\n");
		close(serverSocket); // close server socket
		return 1;
	}

	// initialize buf with 0
	bzero(buf,SIZE);

	// read from socket and print on screen
	while( strcmp(buf,"FINISH_CONNECTION") ) {
		// if read is unsuccess, print error
		if( recv(readerSocket, buf, SIZE, 0) < 0 ) {
			fprintf(stderr,"Read is unsuccesful\n");
			close(serverSocket); // close server socket
			close(readerSocket); // close reader socket
			return 1;
		}
		else {
			// check if string is finish_connection or not 
			if(!strcmp(buf,"FINISH_CONNECTION")) {
				close(serverSocket); // close server socket
				close(readerSocket); // close reader socket
				fclose(fPtr); // close file pointer	
				return 1;
			}
			else {
				i = 0; // set i with 0 for next loop
				// write characters to file
				while(buf[i] != '\n')
					fputc((int)buf[i++], fPtr);
	
				fputc('\n', fPtr); // add '\n' to file
				usleep(1);
			}
		}
		printf("%s",buf);
		// initialize buf with 0
		bzero(buf,SIZE);
	}
	return 0; // terminate code
}
