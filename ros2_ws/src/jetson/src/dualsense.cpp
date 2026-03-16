#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jetson/data_struct.hpp"
#include "custom_interfaces/msg/spi.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#define DOF 6

class DualsenseSub : public rclcpp::Node {
public:
    DualsenseSub(): Node("dualsense_sub"){
        // get the serial port
        // setting up the subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&DualsenseSub::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Dualsense subscription started");

        publisher_ = this->create_publisher<custom_interfaces::msg::SPI>("spi_send", 10);
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
        spi_out_.start_frameH = 0x55;
        spi_out_.start_frameL = 0x55;

        spi_out_.data_size = sizeof(float) * DOF;  // 1 float for each DOF
        spi_out_.device_address = 0x67;

        std::vector<float> wrench(DOF);
        wrench[0] = - static_cast<float>(msg.axes[0]);  // sway
        wrench[1] = static_cast<float>(msg.axes[1]);  // surge
        wrench[2] = static_cast<float>(msg.axes[4] - msg.axes[5]);  // heave
        wrench[3] = - static_cast<float>(msg.axes[3]);  // pitch
        wrench[4] = - static_cast<float>(msg.axes[2]);  // roll

        if (msg.buttons[9] || msg.buttons[10]) {
            wrench[5] = (msg.buttons[9] - msg.buttons[10]) * 0.3;
        }

        auto msg_out = custom_interfaces::msg::SPI();
        msg_out.synch = 0x55;
        msg_out.syncl = 0x55;
        msg_out.size = sizeof(wrench);
        msg_out.address = 0x54;
        std::memcpy(msg_out.data.data(), wrench.data(), msg_out.size);

        publisher_->publish(msg_out);
    }

    packet_t spi_out_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::SPI>::SharedPtr publisher_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualsenseSub>());
    rclcpp::shutdown();
    return 0;
}
