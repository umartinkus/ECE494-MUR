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
		fcntl(fd, F_SETFL, 0);
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
  size_t buf_size{256};
  char buf[buf_size]{};
  size_t nbytes = sizeof(buf);
  ssize_t bytes_read;

  while ( true ) {
    bytes_read = read(fd, buf, nbytes);
    if ( bytes_read > 0 ) {
      std::cout.write(buf, bytes_read);
      std::cout.flush();
    }
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
