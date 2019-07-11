#include "etftp_msg.h"

int main( int argc, char **argv ) {
  ETFTPRRQ rrq;
  ETFTPWRQ wrq;
  ETFTPAck ack;
  ETFTPError err;
  ETFTPData data;
  ETFTPOAck oack;

  std::cout << "RRQ Op Code  : " << rrq.opcode  << std::endl;
  std::cout << "WRQ Op Code  : " << wrq.opcode  << std::endl;
  std::cout << "Data Op Code : " << data.opcode << std::endl;
  std::cout << "ACK Op Code  : " << ack.opcode  << std::endl;
  std::cout << "Error Op Code: " << err.opcode  << std::endl;
  std::cout << "OACK Op Code : " << oack.opcode << std::endl;
  std::cout << "**************************" << std::endl;
  std::cout << "RRQ Length  : " << rrq.len  << std::endl;
  std::cout << "WRQ Length  : " << wrq.len  << std::endl;
  std::cout << "Data Length : " << data.len << std::endl;
  std::cout << "ACK Length  : " << ack.len  << std::endl;
  std::cout << "Error Length: " << err.len  << std::endl;
  std::cout << "OACK Length : " << oack.len << std::endl;
  std::cout << "**************************" << std::endl;
  
  return 0;
}
