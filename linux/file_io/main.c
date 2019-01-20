#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

int main( int argc, char **argv ) {
  int fd = 0;
  char msg[] = "Hello World";
  int msg_len = sizeof( msg );
  
  if( 2 != argc ) {
    printf( "usage: ./main <file name>\n" );
    return -1;
  }

  // open file as READ and WRITE permission
  // if file is not existed, create it. Otherwise, truncate its content
  // READ and WRITE permissions are for file owner only, nothing for others
  fd = open( argv[1], O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR );
  
  if( -1 == fd ) {
    perror( "open" );
    return -1;
  }

  printf( " > %s file is opened\n", argv[1] );

  // write a text into opened file, then check written data length
  if( msg_len != write( fd, msg, msg_len ) ) {
    perror( "write" );
    close( fd );
    
    return -1;
  }

  printf( " > \'%s\' text is written into %s file\n", msg, argv[1] );

  // after the text is written into the file, the file pointer
  // moves toward end of file, so rewind the file pointer to
  // point beginning of the file
  if( -1 == lseek( fd, 0, SEEK_SET ) ) {
    perror( "lseek" );
    close( fd );
    
    return -1;
  }
  
  memset( msg, 0x00, msg_len );
  msg_len = 5;

  // after the file pointer is moved and message length is
  // updated, read five bytes from beginning of the file
  if( msg_len != read( fd, msg, msg_len ) ) {
    perror( "read" );
    close( fd );
    
    return -1;
  }

  printf( " > read from %s file in %d bytes, text is \'%s\'\n",
	  argv[1], msg_len, msg );

  // finally, close the file descriptor
  if( -1 == close( fd ) ) {
    perror( "close" );
    return -1;
  }

  printf( " > %s file is closed\n", argv[1] );
    
  return 0;
}
