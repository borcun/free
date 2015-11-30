#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main( int argc, char **argv ) {
  int random;

  srand( time( NULL ) );
  random = rand() % 50;

  if( atoi( argv[1] ) == random )
    printf( "Congrulations !\nPassword is cracked.\n" );
  else
    printf( "try again: %d %d\n", random, atoi( argv[1] ) );

  return 0;
}
