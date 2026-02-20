#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <memory>

#include "marlin_comms/data_struct.hpp"
#include "marlin_comms/ring_buffer.hpp"
#include "marlin_comms/serial_port.hpp"
#include "marlin_comms/uart_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"

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

class DualsenseSub : public rclcpp::Node {
public:
    DualsenseSub(SerialPort& sp) : Node("dualsense_sub") , sp_(sp) {
        // get the serial port
        // setting up the subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&DualsenseSub::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Dualsense subscription started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy &msg) {
        // axes[0]: LS x (sway)
        // axes[1]: LS y (surge)
        // axes[2]: LT (heave down)
        // axes[3]: RS x (roll)
        // axes[4]: RS y (pitch)
        // axes[5]: RT (heave up)
        // buttons[4]: LB (yaw negative)
        // buttons[5]: RB (yaw positive)
        
        // set start bits
        uart_out_.start_frameH = 0x55;
        uart_out_.start_frameL = 0x55;

        uart_out_.data_size = sizeof(double) * 6;  // 1 double for each DOF
        uart_out_.device_address = 0x67;

        uart_out_.data[0] = msg.axes[0];  // sway
        uart_out_.data[1] = msg.axes[1];  // surge
        uart_out_.data[2] = (msg.axes[2] - msg.axes[5]) / 2;  // heave
        uart_out_.data[3] = msg.axes[4];  // pitch
        uart_out_.data[4] = msg.axes[3];  // roll

        if (msg.buttons[4] || msg.buttons[5]) {
            uart_out_.data[5] = (msg.buttons[4] - msg.buttons[5]) * 0.3;
        }

        std::uint8_t *bytes_out = reinterpret_cast<std::uint8_t*>(&uart_out_);
        sp_.write(bytes_out, sizeof(uartPacket_t));
    }

    SerialPort& sp_;
    uartPacket_t uart_out_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
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

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualsenseSub>(sp));
    rclcpp::shutdown();

    // wait until threads are done (which never happens)
    producer_thread.join();
    consumer_thread.join();
    return 0;
}
