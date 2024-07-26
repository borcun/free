// C program to implement one side of FIFO
// This side writes first, then reads
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char **argv) {
    int fd;
    char arr[64];
    int i = 0;

    if (2 != argc) {
	return -1;
    }
    
    // Creating the named file(FIFO)
    // mkfifo(<pathname>, <permission>)
    mkfifo(argv[1], 0666);
    
    // Open FIFO for write only
    fd = open(argv[1], O_WRONLY);

    if (fd <= 0) {
	printf("%s\n", "could not open fifo");
	return -1;
    }
    
    for (i = 0; i < 3; ++i) {
	printf("\n%s: ", "enter text");
	memset(arr, 0x00, 64);
	// Take an input arr2ing from user.
	// 80 is maximum length
	fgets(arr, 64, stdin);

	// Write the input arr2ing on FIFO
	// and close it
	write(fd, arr, strlen(arr) + 1);
    }

    close(fd);
    
    return 0;
}
