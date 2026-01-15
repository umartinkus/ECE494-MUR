#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SerialPort {
public:
	SerialPort() : port("/dev/ttyUSB0") {
		nbytes = sizeof(buf);
	}

	void open_port() {
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
		if ( fd == -1 ) {
			// open port failed
			perror("open_port: Unable to open /dev/ttyUSB0 ");
		} else {
			fcntl(fd, F_SETFL, 0);
		}
	}

private:
	std::string port;
	int fd;  // file descriptor for port
	char buf[256]{};
	size_t nbytes;
	ssize_t bytes_read;
};

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node
{
public:
	DataPublisher() : Node("data_publisher"), count_(0) {
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
		timer_ = this->create_wall_timer(
        std::chrono::miliseconds(500),
        std::bind(&DataPublisher::timer_callback, this));
	}

	void get_port_data(SerialPort port_obj) {

	}

private:
	void timer_callback() {
		auto message = std_msgs::msg::String();
		message.data = "Hello, world! " + std::to_string(count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
		publisher_->publish(message);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	auto publisher = std::make_shared<DataPublisher>();
	rclcpp::spin(publisher);

	SerialPort esp32;
	esp32.open_port();

	rclcpp::shutdown();
	return 0;
}
