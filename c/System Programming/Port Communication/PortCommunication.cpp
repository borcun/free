#include "PortCommunication.h"

// constructor
PortCommunication::PortCommunication() {
}

// destructor
PortCommunication::~PortCommunication() {
	close(fileDescription); // close file description
}

// function that opens port
bool PortCommunication::openPort() {
	// open all ttySX file to open by one by
	if((fileDescription = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY)) != -1) {
		puts("/dev/ttyS0 is opened");
		return true;
	}
	else if ((fileDescription = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY)) != -1) {
		puts("/dev/ttyS1 is opened");
		return true;
	}
	else if ((fileDescription = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY)) != -1) {
		puts("/dev/ttyS2 is opened");
		return true;
	}
	else if ((fileDescription = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY)) != -1) {
		puts("/dev/ttyS3 is opened");
		return true;
	}
	
	// if file can not opened, give error message
	if(fileDescription == -1) {
		perror("cannot open any port");
		return false;
	} 
	else {
		// set file status flags
		fcntl(fileDescription, F_SETFL, 0);
		return false;
	}
} // end of openPort function

// function that writes to port
bool PortCommunication::writePort(char *buf) {
	int len = strlen(buf);
	buf[len] = 0x0d; // stick a <CR> after the command
	buf[len+1] = 0x00; // terminate the string properly
	
	if( write(fileDescription, buf, strlen(buf)) < 0 )
		return false;
	
	return true;
} // end of writePort function

// function that reads from port
bool PortCommunication::readPort(char *buf) {
	int dataLen = read(fileDescription, buf, 4);
	
	buf[dataLen-1] = 0x00;
	
	if (dataLen < 0) {
		if (errno == EAGAIN) {
			perror("SERIAL EAGAIN ERROR\n");
			return false;
		} else {
			printf("SERIAL read error %d %s\n", errno, strerror(errno));
			return false;
		}
	}                    
	return true;
}

// function that gets BAUD
int PortCommunication::getBAUD() {
	struct termios termAttr;
	int inputSpeed = -1;
	speed_t baudRate;
	
	// get file description attributes
	tcgetattr(fileDescription, &termAttr);
	// Get the input speed
	baudRate = cfgetispeed(&termAttr);
	
	// find BAUD
	switch (baudRate) {
		case B0:      inputSpeed = 0; break;
		case B50:     inputSpeed = 50; break;
		case B110:    inputSpeed = 110; break;
		case B134:    inputSpeed = 134; break;
		case B150:    inputSpeed = 150; break;
		case B200:    inputSpeed = 200; break;
		case B300:    inputSpeed = 300; break;
		case B600:    inputSpeed = 600; break;
		case B1200:   inputSpeed = 1200; break;
		case B1800:   inputSpeed = 1800; break;
		case B2400:   inputSpeed = 2400; break;
		case B4800:   inputSpeed = 4800; break;
		case B9600:   inputSpeed = 9600; break;
		case B19200:  inputSpeed = 19200; break;
		case B38400:  inputSpeed = 38400; break;
		case B115200: inputSpeed = 115200; break;
	}
	
	bRate = inputSpeed;
	
	return inputSpeed;
} // end of getBAUD function

// function that initialize port
void PortCommunication::initPort() {
	// Get the current options for the port
	tcgetattr(fileDescription, &options);
	// Set the baud rates to bRate
	cfsetispeed(&options, bRate);
	cfsetospeed(&options, bRate);
	// Enable the receiver and set local mode
	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Set the new options for the port
	tcsetattr(fileDescription, TCSANOW, &options);
	
	return;
} // end of initPort function
