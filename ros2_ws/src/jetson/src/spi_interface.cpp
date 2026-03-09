#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <functional>

#include "jetson/data_struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/spi.hpp"

// class that owns the SPI port 
class SpiDevice {
public:
    SpiDevice() = default;

    ~SpiDevice() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
  }

  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;

  void openPort(const std::string& device, uint32_t speed_hz, uint8_t mode = SPI_MODE_0,
                uint8_t bits_per_word = 8) {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }

        fd_ = ::open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("open(" + device + ") failed: " + std::strerror(errno));
        }

        mode_ = mode;
        bits_per_word_ = bits_per_word;
        speed_hz_ = speed_hz;

        if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0 || ::ioctl(fd_, SPI_IOC_RD_MODE, &mode_) < 0) {
            throw std::runtime_error("Failed to set/get SPI mode: " + std::string(std::strerror(errno)));
        }

        if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0 || ::ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word_) < 0) {
            throw std::runtime_error("Failed to set/get bits per word: " + std::string(std::strerror(errno)));
        }

        if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0 || ::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz_) < 0) {
            throw std::runtime_error("Failed to set/get max speed: " + std::string(std::strerror(errno)));
        }
  }

    void transfer(const std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) const {
        if (fd_ < 0) {
            throw std::runtime_error("SPI port is not open. Call openPort() first.");
        }
        if (tx.empty()) {
            rx.clear();
        return;
        }

        rx.assign(tx.size(), 0);

        spi_ioc_transfer tr{};
        tr.tx_buf = reinterpret_cast<uint64_t>(tx.data());
        tr.rx_buf = reinterpret_cast<uint64_t>(rx.data());
        tr.len = static_cast<uint32_t>(tx.size());
        tr.speed_hz = speed_hz_;
        tr.bits_per_word = bits_per_word_;

        if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1) {
            throw std::runtime_error("SPI transfer failed: " + std::string(std::strerror(errno)));
        }
    }

private:
    int fd_{-1};
    uint8_t mode_{SPI_MODE_0};
    uint8_t bits_per_word_{8};
    uint32_t speed_hz_{500000};
};

class SPI_Interface : public rclcpp::Node {
public:
    SPI_Interface() : Node("spi_interface") {
        spi1_.openPort("/dev/spidev0.0", 1000000);
        
        subscription_ = this->create_subscription<custom_interfaces::msg::SPI>(
            "spi_send", 
            10,
            std::bind(&SPI_Interface::spi_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<custom_interfaces::msg::SPI>("spi_receive", 10);

        RCLCPP_INFO(this->get_logger(), "SPI Node Started");
    }
private:
    void spi_callback(const custom_interfaces::msg::SPI &msg) {
        // send all the data into a single contiguous struct
        uart_out_.start_frameH = msg.synch;
        uart_out_.start_frameL = msg.syncl;
        uart_out_.data_size = msg.size;
        uart_out_.device_address = msg.address;
        uart_out_.crc = msg.crc;

        std::memcpy(uart_out_.data, msg.data.data(), sizeof(uart_out_.data));

        // nifty trick to cast the struct into a vector
        std::vector<uint8_t> spi_out(
            reinterpret_cast<uint8_t*>(&uart_out_),
            reinterpret_cast<uint8_t*>(&uart_out_) + sizeof(uart_out_)
        );

        std::vector<uint8_t> spi_in;

        spi1_.transfer(spi_out, spi_in);

        RCLCPP_INFO(this->get_logger(), "sent spi message");
    }

    uartPacket_t uart_out_{};
    SpiDevice spi1_;
    rclcpp::Subscription<custom_interfaces::msg::SPI>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::SPI>::SharedPtr publisher_;
};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPI_Interface>());
    rclcpp::shutdown();
    return 0;
}
