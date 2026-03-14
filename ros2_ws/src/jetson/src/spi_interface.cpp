#include <custom_interfaces/msg/detail/spi__struct.hpp>
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
#include <array>
#include <functional>
#include <algorithm>

#include "jetson/crc.hpp"
#include "jetson/data_struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/spi.hpp"

#define SYNCH_POS 0
#define SYNCL_POS 1
#define SIZE_POS 2
#define ADDR_POS 3
#define DATA_POS 4
#define CRC1_POS 62
#define CRC2_POS 63

#define SPI_PACKET_SIZE 64

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

  uint32_t openPort(const std::string& device, uint32_t speed_hz, uint8_t mode = SPI_MODE_0,
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

        if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0 
            || ::ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word_) < 0) {
            throw std::runtime_error("Failed to set/get bits per word: " + std::string(std::strerror(errno)));
        }

        if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0 
            || ::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz_) < 0) {
            throw std::runtime_error("Failed to set/get max speed: " + std::string(std::strerror(errno)));
        }

        return speed_hz_;
  }

    void transfer(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) const {
        if (fd_ < 0) {
            throw std::runtime_error("SPI port is not open. Call openPort() first.");
        }

        // make sure it is sending 64 bytes
        if (tx.size() < SPI_PACKET_SIZE) {
            tx.resize(SPI_PACKET_SIZE);
        return;
        }

        rx.assign(tx.size(), 0);

        spi_ioc_transfer tr{};
        tr.tx_buf = reinterpret_cast<uint64_t>(tx.data());
        tr.rx_buf = reinterpret_cast<uint64_t>(rx.data());
        tr.len = SPI_PACKET_SIZE;
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
        const uint32_t speed_hz = spi1_.openPort("/dev/spidev0.0", 5000000);
        RCLCPP_INFO(this->get_logger(), "Configured SPI max speed: %u Hz", speed_hz);
        
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
        spi_out_.start_frameH = msg.synch;
        spi_out_.start_frameL = msg.syncl;
        spi_out_.data_size = std::min<std::size_t>(msg.size, sizeof(spi_out_.data));
        spi_out_.device_address = msg.address;
        
        std::fill(std::begin(spi_out_.data), std::end(spi_out_.data), 0);
        std::memcpy(spi_out_.data, msg.data.data(), spi_out_.data_size);

        auto crc_msg = msg;
        crc_msg.size = spi_out_.data_size;
        spi_out_.crc = encode_crc16(crc_msg);

        RCLCPP_INFO(this->get_logger(), "CRC Sent: %X", spi_out_.crc);

        // nifty trick to cast the struct into a vector
        std::vector<uint8_t> spi_out(
            reinterpret_cast<uint8_t*>(&spi_out_),
            reinterpret_cast<uint8_t*>(&spi_out_) + sizeof(spi_out_)
        );
        spi_out[CRC1_POS] = static_cast<std::uint8_t>(spi_out_.crc & 0xFF);
        spi_out[CRC2_POS] = static_cast<std::uint8_t>((spi_out_.crc >> 8) & 0xFF);

        RCLCPP_INFO(this->get_logger(), "first sync byte: %X, %X", spi_out[0], spi_out[1]);

        std::vector<uint8_t> spi_in;

        spi1_.transfer(spi_out, spi_in);

        // if (std::all_of(spi_in.begin(), spi_in.end(), [](std::uint8_t i) { return i == 0; })) {
        //     RCLCPP_INFO(this->get_logger(), "Received SPI Message was all zeros");
        // } else {
        //     for (std::uint8_t &word : spi_in) {
        //         step_(word);
        //     }
        // }

        auto msg_out = custom_interfaces::msg::SPI();
        msg_out.synch = spi_in[SYNCH_POS];
        msg_out.syncl = spi_in[SYNCL_POS];

        if (msg_out.synch != START_FRAMEH || msg_out.syncl != START_FRAMEL) {
            RCLCPP_INFO(this->get_logger(), "Bad sync bytes");
            return;
        }

        msg_out.size = spi_in[SIZE_POS];
        msg_out.address = spi_in[ADDR_POS];
        if (msg_out.size > msg_out.data.size()) {
            RCLCPP_INFO(this->get_logger(), "Payload too large: %u", msg_out.size);
            return;
        }

        std::fill(msg_out.data.begin(), msg_out.data.end(), 0);
        std::copy(spi_in.begin() + DATA_POS, spi_in.begin() + DATA_POS + msg_out.size, msg_out.data.begin());
        msg_out.crc = static_cast<std::uint16_t>(spi_in[CRC1_POS])
                    | (static_cast<std::uint16_t>(spi_in[CRC2_POS]) << 8);

	for (int i = 0; i < 64; i++) {
		RCLCPP_INFO(this->get_logger(), "\ni: %d, val: %X", i, spi_in[i]);
	}

        if (!check_crc16(msg_out)) {
            // RCLCPP_INFO(this->get_logger(), "Bad crc: %X");
            return;
        }

        publisher_->publish(msg_out);
        return;
    }

    // void step_(std::uint8_t b) {
    //     std::cout << std::hex << static_cast<unsigned int>(b) << " ";
    //     switch (state_) {
    //         case State::WAIT_SYNC:
    //             if (first_sync_ && b == 0x55) {
    //
    //                 // both conditions met, move to next state
    //                 state_ = State::READ_SIZE;
    //                 first_sync_ = false;
    //                 idx_ = 0;
    //
    //             } else if (b == 0x55) {
    //                 first_sync_ = true;
    //             } else {
    //                 first_sync_ = false;
    //             }
    //             break;
    //
    //         case State::READ_SIZE:
    //             data_size_ = static_cast<std::size_t>(b);
    //             msg_out.synch = 0x55;
    //             msg_out.syncl = 0x55;
    //             msg_out.size = b;
    //             state_ = State::READ_ADDR;
    //             break;
    //
    //         case State::READ_ADDR:
    //             msg_out.address = b;
    //             state_ = State::READ_DATA;
    //             break;
    //
    //         case State::READ_DATA:
    //             msg_out.data[idx_++] = b;
    //             if (idx_ == data_size_) {
    //                 state_ = State::CHECK_CRC;
    //                 std::cout << std::endl;
    //                 }
    //             // add some sort of callback to handle the data payload
    //             break;
    //
    //         case State::CHECK_CRC:
    //             if (first_crc_) {
    //                 first_crc_ = false;
    //                 crc_vector[0] = b;
    //             } else {
    //                 first_crc_ = true;
    //                 crc_vector[1] = b;
    //
    //                 // convert the two crc bytes into a single uint16_t
    //                 msg_out.crc = static_cast<std::uint16_t>(crc_vector[0])
    //                     | (static_cast<std::uint16_t>(crc_vector[1]) << 8);
    //                 state_ = State::WAIT_SYNC;
    //
    //                 // check crc
    //                 if (check_crc16(msg_out)) {
    //                     this->publisher_->publish(msg_out);
    //                 }
    //             }
    //             break;
    //     }
    // }

    // enum class State {
    //     WAIT_SYNC,
    //     READ_SIZE,
    //     READ_ADDR,
    //     READ_DATA,
    //     CHECK_CRC
    // };

    // state machine values
    // State state_{State::WAIT_SYNC};
    custom_interfaces::msg::SPI msg_out;
    // bool first_sync_{false};
    // bool first_crc_{true};
    // std::size_t data_size_{0};
    // std::size_t idx_{0};
    // std::array<std::uint8_t, 2> crc_vector{};

    packet_t spi_out_{};
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
