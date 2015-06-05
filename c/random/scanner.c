#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main() {
	int i;
	int number;

	srand( time( NULL ) );
	number = rand() % 20;

	scanf( "%d", &i );

	if( i == number )	
		printf( "you find random number %d\n", i );

	return 0;
}
