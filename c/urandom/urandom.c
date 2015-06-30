#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define SIZE 6 

int main() {
	int fd;
	int i;
	unsigned char arr[ SIZE ] = { 0 };

	if( -1 == (fd = open( "/dev/urandom", O_RDONLY ) ) ) {
		fprintf( stderr, "%s\n", "file not opened" );
		return -1;
	}

	if( SIZE != read( fd, arr, SIZE ) ) {
		fprintf( stderr, "%s\n", "read function error" );
		return -1;
	}

	for( i=0 ; i < SIZE ; ++i )
		printf( "%d ", ( arr[ i ] % 45 ) + 1 );
	printf( "\n" );

	close( fd );

	return 0;
}
