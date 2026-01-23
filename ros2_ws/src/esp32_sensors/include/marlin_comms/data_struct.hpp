#include <cstdint>
#include <vector>

struct DataContainer {
  std::uint8_t address;
  std::uint8_t size;
  std::vector<std::uint8_t> raw;
};
