#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include "marlin_comms/data_struct.hpp"
#include "marlin_comms/ring_buffer.hpp"
#include "marlin_comms/serial_port.hpp"
#include "marlin_comms/uart_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define BUF_SIZE 4096

void port_listener(SerialPort &sp, ByteRing &br) {
    // setting the max size and the read timeout
    const std::size_t max_len = 4096 * 2;
    const std::chrono::milliseconds timeout(500);

    // creating a buffer vector
    std::vector<std::uint8_t> buf(max_len);

    // num of bytes read from buffer
    std::size_t n_read{};

    // poll and write to ring buffer
    // note that the buffer is a drop buffer
    for (;;) {
        n_read = sp.read(buf.data(), buf.size(), timeout);
        br.write(buf.data(), n_read);
    }
    }
    void parser(ByteRing &br) {
    // this is going to be a state machine
    std::vector<std::uint8_t> temp(BUF_SIZE);
    std::size_t n_read{0};

    // create an instance of the state machine
    UartParser parser_object;

    // consume bytes and pass into the state machine
    for (;;) {
        n_read = br.read(temp.data(), BUF_SIZE);
        parser_object.consume(temp.data(), n_read);
    }
}

class DataPublisher : public rclcpp::Node {
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
    // create a condition_variable
    std::condition_variable cv;

    // initialize serial port obj
    SerialPort sp("/dev/ttyTHS1");
    sp.config_port(B115200);

    // initialize ByteRing obj
    ByteRing br(32768);
    Ring<uartPacket_t> dr(BUF_SIZE);

    // instatiate threads
    std::thread producer_thread(port_listener, std::ref(sp), std::ref(br));
    std::thread consumer_thread(parser, std::ref(br));

        // wait until threads are done (which never happens)
    producer_thread.join();
    consumer_thread.join();
}
