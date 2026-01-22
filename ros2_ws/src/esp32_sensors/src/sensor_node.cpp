#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "marlin_comms/serial_port.hpp"
#include "marlin_comms/ring_buffer.hpp"


void port_listener(SerialPort& sp, ByteRing& br) {
    std::cout << "running" << std::endl;
    // setting the max size and the read timeout
    const std::size_t max_len = 4096*2;
    const std::chrono::milliseconds timeout(500);

    // creating a buffer vector
    std::vector<std::uint8_t> buf;
    buf.reserve(max_len);

    // num of bytes read from buffer
    std::size_t n_read{};
    std::size_t n_written{};

    for (;;) {
        n_read = sp.read(buf.data(), buf.capacity(), timeout);
        n_written = br.write(buf.data(), n_read);
        if (n_written > 0) {
            std::cout << "ugh" << std::endl;
            std::vector<std::uint8_t> tmp;
            br.read(tmp.data(), n_written);
            // for (const auto& val : tmp) {
            //     std::cout << val;
            // }
        }
    }
}


class DataPublisher : public rclcpp::Node
{
public:
	DataPublisher() : Node("data_publisher"), count_(0) {
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
		timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&DataPublisher::timer_callback, this));
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


int main() {
    // initialize serial port obj
    SerialPort sp("/dev/ttyUSB0");
    sp.config_port(B115200);
    
    //initialize ByteRing obj
    ByteRing br(32768);

    std::thread sp_thread(port_listener, std::ref(sp), std::ref(br));

    sp_thread.join();
}
