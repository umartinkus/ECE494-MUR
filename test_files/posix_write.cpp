#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

int open_port(void)
{
	int fd;  // file descriptor for the port
	
  // opening the port
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		// opening port failed
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
	} else {
		fcntl(fd, F_SETFL, FNDELAY);
	}

	return fd;
}

void config_port(int fd) {
  // initializing the options struct
  struct termios options;
  tcgetattr(fd, &options);

  // setting the baud rate
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // enable receiver and set local mode
  options.c_cflag |= (CLOCAL | CREAD);

  // set the character size
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // apply changes now
  tcsetattr(fd, TCSANOW, &options);
}

void listen(int fd) {
  char buf[16];
  size_t nbytes = sizeof(buf);

  while ( true ) {
    read(fd, buf, nbytes);
    std::cout << buf << std::endl;
  }
}

int main()
{
	int fd; 
	fd = open_port();
  config_port(fd);
  listen(fd);
  close(fd);
	return 0;
}
