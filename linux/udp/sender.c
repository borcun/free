#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 10000
#define MAX_BUF_SIZE 1024

int main() {
    int sockfd;
    struct sockaddr_in server_addr;
    char buffer[MAX_BUF_SIZE] = "Hello, UDP Receiver!";

    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Fill server information
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    // Send message to server
    sendto(sockfd, buffer, strlen(buffer), 0, (const struct sockaddr*)&server_addr, sizeof(server_addr));

    printf("Message sent to the server.\n");

    // Close the socket
    close(sockfd);

    return 0;
}
