#ifndef ETFTP_MSG_H
#define ETFTP_MSG_H

#include <iostream>
#include <cstring>

//! maximum file name length
#define MAX_FILENAME_LEN (255)

//! maximum mode name length
#define MAX_MODE_LEN (16)
#define MODE_NETASCII ("netascii")
#define MODE_OCTET ("octet")

//! maximum data length
#define MAX_DATA_LEN (32757)

//! maximum error message length
#define MAX_ERR_MSG_LEN (128)

//! op code index in all messages
#define OPCODE_INDEX (1)

typedef unsigned short ushort;
typedef unsigned char uchar;

enum OpCode {
  OC_RRQ   = 0x01,
  OC_WRQ   = 0x02,
  OC_DATA  = 0x03,
  OC_ACK   = 0x04,
  OC_ERROR = 0x05,
  OC_OACK  = 0x06
};

enum ErrorCode {
  EC_NOT_DEFINED,
  EC_FILE_NOT_FOUND,
  EC_ACCESS_VIOLATION,
  EC_DISK_FULL_OR_ALLOC_EXC,
  EC_ILLEGAL_TFTP_OPS,
  EC_UNKNOWN_TRANSFER_ID,
  EC_FILE_ALREADY_EXISTS,
  EC_NO_SUCH_USER
};

struct ETFTPMessage {
  ushort opcode;
};

/// @struct ETFTPRRQ
struct ETFTPRRQ : ETFTPMessage {  
  char filename[ MAX_FILENAME_LEN ];
  uchar filename_len;
  char mode[ MAX_MODE_LEN ];
  uchar mode_len;
  unsigned int len;

  ETFTPRRQ( void ) {
    opcode = OC_RRQ;
    memset( filename, 0x00, MAX_FILENAME_LEN );
    filename_len = MAX_FILENAME_LEN;
    memset( mode, 0x00, MAX_MODE_LEN );
    mode_len = MAX_MODE_LEN;

    len = sizeof( opcode ) + filename_len +
      sizeof( filename_len ) + mode_len +
      sizeof( mode_len );
  }
  
  unsigned int serialize( char *stream ) {
    len = 0;
    
    if( NULL == stream ) {
      return len;
    }   
    
    stream[ len++ ] = 0x00;
    stream[ len++ ] = OC_RRQ;

    memcpy( &stream[ len ], filename, filename_len );
    len += filename_len;

    stream[ len++ ] = 0x00;

    memcpy( &stream[ len ], mode, mode_len );
    len += mode_len;

    stream[ len++ ] = 0x00;

    return len;
  }
};

/// @struct ETFTPWRQ
struct ETFTPWRQ : ETFTPMessage {  
  char filename[ MAX_FILENAME_LEN ];
  uchar filename_len;
  char mode[ MAX_MODE_LEN ];
  uchar mode_len;
  unsigned int len;

  ETFTPWRQ( void ) {
    opcode = OC_WRQ;
    memset( filename, 0x00, MAX_FILENAME_LEN );
    filename_len = MAX_FILENAME_LEN;
    memset( mode, 0x00, MAX_MODE_LEN );
    mode_len = MAX_MODE_LEN;

    len = sizeof( opcode ) + filename_len +
      sizeof( filename_len ) + mode_len +
      sizeof( mode_len );
  }
  
  unsigned int serialize( char *stream ) {
    len = 0;
    
    if( NULL == stream ) {
      return len;
    }   
    
    stream[ len++ ] = 0x00;
    stream[ len++ ] = OC_WRQ;

    memcpy( &stream[ len ], filename, filename_len );
    len += filename_len;

    stream[ len++ ] = 0x00;

    memcpy( &stream[ len ], mode, mode_len );
    len += mode_len;

    stream[ len++ ] = 0x00;

    return len;
  }
};

/// @struct
struct ETFTPData : ETFTPMessage {
  ushort block_num;
  char data[ MAX_DATA_LEN ];
  ushort data_len;
  unsigned int len;
  
  ETFTPData( void ) {
    opcode = OC_DATA;
    block_num = 1;
    memset( data, 0x00, MAX_DATA_LEN );
    data_len = MAX_DATA_LEN;

    len = sizeof( opcode ) + sizeof( block_num ) +
      data_len + sizeof( data_len );
  }

  ushort getBlockNumber( const char *stream ) {
    ushort bn = 0x0000;

    bn = static_cast< ushort >( stream[2] );
    bn <<= 8;
    bn |= static_cast< ushort >( stream[3] );

    return bn;
  }

  char *getData( char *stream ) {
    return &stream[4];
  }
  
  unsigned int serialize( char *stream ) {
    len = 0;
    
    if( NULL == stream ) {
      return len;
    }   

    stream[ len++ ] = 0x00;
    stream[ len++ ] = OC_DATA;
    stream[ len++ ] = ( block_num & 0xFF00 ) >> 8;
    stream[ len++ ] = ( block_num & 0x00FF );

    memcpy( &stream[ len ], data, data_len );
    len += data_len;

    return len;
  }
};

/// @struct
struct ETFTPAck : ETFTPMessage {
  ushort block_num;
  unsigned int len;
  
  ETFTPAck( void ) {
    opcode = OC_ACK;
    block_num = 1;

    len = sizeof( opcode ) + sizeof( block_num );
  }

  ushort getBlockNumber( const char *stream ) {
    ushort bn = 0x0000;

    bn = static_cast< ushort >( stream[2] );
    bn <<= 8;
    bn |= static_cast< ushort >( stream[3] );

    return bn;
  }
  
  unsigned int serialize( char *stream ) {
    len = 0;
    
    if( NULL == stream ) {
      return len;
    }   

    stream[ len++ ] = 0x00;
    stream[ len++ ] = OC_ACK;
    stream[ len++ ] = ( block_num & 0xFF00 ) >> 8;
    stream[ len++ ] = ( block_num & 0x00FF );

    return len;
  }
};

/// @struct
struct ETFTPError : ETFTPMessage {
  ushort err_code;
  char err_msg[ MAX_ERR_MSG_LEN ];
  uchar err_msg_len;
  unsigned int len;

  ETFTPError( void ) {
    opcode = OC_ERROR;
    err_code = EC_NOT_DEFINED;
    memset( err_msg, 0x00, MAX_ERR_MSG_LEN );
    err_msg_len = MAX_ERR_MSG_LEN;
    
    len = sizeof( opcode ) + sizeof( err_code ) +
      err_msg_len + sizeof( err_msg_len );
  }

  ushort getCode( const char *stream ) {
    ushort ec = 0x0000;

    ec = static_cast< ushort >( stream[2] );
    ec <<= 8;
    ec |= static_cast< ushort >( stream[3] );

    return ec;
  }

  char *getMessage( char *stream ) {
    return &stream[4];
  }
  
  unsigned int serialize( char *stream ) {
    len = 0;
    
    if( NULL == stream ) {
      return len;
    }   

    stream[ len++ ] = 0x00;
    stream[ len++ ] = OC_ERROR;

    memcpy( &stream[ len ], err_msg, err_msg_len );
    len += err_msg_len;

    stream[ len++ ] = err_msg_len;

    return len;
  }
};

struct ETFTPOAck : ETFTPMessage {
  ushort block_num;
  unsigned int len;
  
  ETFTPOAck( void ) {
    opcode = OC_OACK;
    block_num = 1;

    len = sizeof( opcode ) + sizeof( block_num );
  }

  ushort getBlockNumber( const char *stream ) {
    ushort bn = 0x0000;

    bn = static_cast< ushort >( stream[2] );
    bn <<= 8;
    bn |= static_cast< ushort >( stream[3] );

    return bn;
  }

  unsigned int serialize( char *stream ) {
    len = 0;
    
    if( NULL == stream ) {
      return len;
    }   

    stream[ len++ ] = 0x00;
    stream[ len++ ] = OC_OACK;
    stream[ len++ ] = ( block_num & 0xFF00 ) >> 8;
    stream[ len++ ] = ( block_num & 0x00FF );

    return len;
  }
};

#endif
