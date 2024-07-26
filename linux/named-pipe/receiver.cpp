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
    int x;
    
    if (2 != argc) {
	return -1;
    }
        
    // Open FIFO for write only
    fd = open(argv[1], O_RDONLY);

    if (fd <= 0) {
	printf("%s\n", "could not open fifo");
	return -1;
    }

    x = getchar();
    
    for (i = 0; i < 3; ++i) {
	memset(arr, 0x00, 64);
	read(fd, arr, 64);
	printf("received text: %s\n", arr);
    }

    close(fd);
    
    return 0;
}
