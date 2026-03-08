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

    if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0 ||
        ::ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word_) < 0) {
      throw std::runtime_error("Failed to set/get bits per word: " +
                               std::string(std::strerror(errno)));
    }

    if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0 ||
        ::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz_) < 0) {
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

int main(int argc, char* argv[]) {
  const std::string device = (argc > 1) ? argv[1] : "/dev/spidev0.0";

  try {
    SpiDevice spi;
    spi.openPort(device, 1000000);

    std::vector<uint8_t> tx{0xAA, 0x01, 0x02, 0x03};
    std::vector<uint8_t> rx;
    spi.transfer(tx, rx);

    std::cout << "RX:";
    for (const auto byte : rx) {
      std::cout << " 0x" << std::hex << static_cast<int>(byte);
    }
    std::cout << std::dec << '\n';
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
