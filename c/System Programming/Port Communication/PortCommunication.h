#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

class PortCommunication
{
public:
	PortCommunication();
	~PortCommunication();
	bool openPort();
	bool writePort(char *);
	bool readPort(char *);
	int getBAUD();
	void initPort();
	
private:
	int fileDescription;
	struct termios options;
	int bRate;
};
