#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 10000
#define MAX_BUF_SIZE 1024

int main() {
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[MAX_BUF_SIZE];
    socklen_t len = sizeof(client_addr);

    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Fill server information
    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Receive message from client
    for (int i = 0; i < 10; ++i) {
	int n = recvfrom(sockfd, (char*)buffer, MAX_BUF_SIZE, 0, (struct sockaddr*) &client_addr, &len);
	buffer[n] = '\0';

	char sender_ip[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &client_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);
	int sender_port = ntohs(client_addr.sin_port);

	printf("Message from sender %s:%d: %s\n", sender_ip, sender_port, buffer);
    }
    
    // Close the socket
    close(sockfd);

    return 0;
}
