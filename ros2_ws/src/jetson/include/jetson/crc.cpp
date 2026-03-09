#include "crc.hpp"
#include "jetson/crc_lookup.hpp"

 void encode_crc16(custom_interfaces::msg::SPI &msg) {
    std::uint16_t crc = 0;
    std::uint8_t div = 0;

    std::vector<std::uint8_t> packet_bytes;
    packet_bytes.reserve(4 + msg.data.size());
    packet_bytes.push_back(msg.synch);
    packet_bytes.push_back(msg.syncl);
    packet_bytes.push_back(msg.size);
    packet_bytes.push_back(msg.address);
    packet_bytes.insert(packet_bytes.end(), msg.data.begin(), msg.data.end());

    for (std::uint8_t &value : packet_bytes) {
        div = (crc >> 8) ^ value;
        crc = (crc << 8) ^ lookup_bytes[div];
    }
    msg.crc = crc;
}

bool check_crc16(const custom_interfaces::msg::SPI &msg) {
    std::uint16_t crc = 0;
    std::uint8_t div = 0;

    std::vector<std::uint8_t> packet_bytes;
    packet_bytes.reserve(4 + msg.data.size());
    packet_bytes.push_back(msg.synch);
    packet_bytes.push_back(msg.syncl);
    packet_bytes.push_back(msg.size);
    packet_bytes.push_back(msg.address);
    packet_bytes.insert(packet_bytes.end(), msg.data.begin(), msg.data.end());

    for (std::uint8_t &value : packet_bytes) {
        div = (crc >> 8) ^ value;
        crc = (crc << 8) ^ lookup_bytes[div];
    }

    return crc == msg.crc;
}
