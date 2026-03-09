#pragma once
#include <cstdint>

#include "crc_lookup.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/spi.hpp"

std::size_t encode_crc16(custom_interfaces::msg::SPI &msg);
