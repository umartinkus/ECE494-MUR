#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

int open_port(void)
{
	int fd;  // file descriptor for the port
	
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		// opening port failed
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
	} else {
		fcntl(fd, F_SETFL, 0);
	}

	return fd;
}

int main()
{
	int fd; 
	fd = open_port();
	return 0;
}
